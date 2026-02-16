#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import argparse, json, time
import numpy as np
from hex_zmq_servers import (
    HEX_LOG_LEVEL,
    hex_log,
    HexRobotHelloClient,
    HexRobotHexarmClient,
)
from hex_robo_utils import (
    HexDynUtil as DynUtil,
    HexRate,
)


def wait_client_working(client, timeout: float = 5.0) -> bool:
    for _ in range(int(timeout * 10)):
        if client.is_working():
            if hasattr(client, "seq_clear"):
                client.seq_clear()
            return True
        else:
            time.sleep(0.1)
    return False


def interp_joint(cur_q, tar_joint, err_limit=0.05):
    err = tar_joint - cur_q
    max_err_fab = np.fabs(err).max()
    if max_err_fab < err_limit:
        return tar_joint, False
    else:
        err_norm = err / max_err_fab
        return cur_q + err_norm * err_limit, True


def interp_arm(cur_q,
               tar_joint,
               grip_tar=None,
               dofs: dict = None,
               err_limit=0.05,
               grip_err_limit=None):
    mid_joint = np.zeros(dofs["sum"])
    mid_joint[:dofs["robot_arm"]], interp_flag = interp_joint(
        cur_q[:dofs["robot_arm"]],
        tar_joint,
        err_limit=err_limit,
    )
    if grip_tar is not None:
        mid_joint[-dofs["robot_gripper"]:], _ = interp_joint(
            cur_q[-dofs["robot_gripper"]:],
            grip_tar,
            err_limit=grip_err_limit
            if grip_err_limit is not None else err_limit,
        )
    return mid_joint, interp_flag


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    try:
        model_path = cfg["model_path"]
        last_link = cfg["last_link"]
        ctrl_cfg = cfg["ctrl_cfg"]
        mit_kp = np.array(ctrl_cfg["mit_kp"])
        mit_kd = np.array(ctrl_cfg["mit_kd"])
        hello_net_cfg = cfg["hello_net_cfg"]
        hexarm_net_cfg = cfg["hexarm_net_cfg"]
    except KeyError as ke:
        missing_key = ke.args[0]
        raise ValueError(f"cfg is not valid, missing key: {missing_key}")

    hello_client = HexRobotHelloClient(net_config=hello_net_cfg)
    hexarm_client = HexRobotHexarmClient(net_config=hexarm_net_cfg)
    dyn_util = DynUtil(model_path, last_link)

    # wait servers to work
    if not wait_client_working(hello_client):
        hex_log(HEX_LOG_LEVEL["err"], "hello server is not working")
        return
    if not wait_client_working(hexarm_client):
        hex_log(HEX_LOG_LEVEL["err"], "hexarm server is not working")
        return

    hello_dof_arr = hello_client.get_dofs()
    hello_dofs = {
        "robot_arm": int(hello_dof_arr[0]),
        "robot_gripper":
        int(hello_dof_arr[1]) if len(hello_dof_arr) > 1 else None,
        "sum": int(hello_dof_arr.sum()),
    }
    hex_log(HEX_LOG_LEVEL["info"], f"hello dofs: {hello_dofs}")

    hexarm_dof_arr = hexarm_client.get_dofs()
    hexarm_dofs = {
        "robot_arm": int(hexarm_dof_arr[0]),
        "robot_gripper":
        int(hexarm_dof_arr[1]) if len(hexarm_dof_arr) > 1 else None,
        "sum": int(hexarm_dof_arr.sum()),
    }
    hex_log(HEX_LOG_LEVEL["info"], f"hexarm dofs: {hexarm_dofs}")
    hexarm_limits = hexarm_client.get_limits()
    hex_log(HEX_LOG_LEVEL["info"], f"hexarm limits: {hexarm_limits.shape}")
    gripper_k, gripper_d = None, None
    if hexarm_dofs["robot_gripper"] is not None:
        gripper_limit = hexarm_limits[-hexarm_dofs["robot_gripper"]:,
                                      0, :].reshape(-1, 2)
        gripper_k = (gripper_limit[:, 1] - gripper_limit[:, 0]) / 2.0
        gripper_d = gripper_limit[:, 1] - gripper_k

    # work loop
    hello_cmds = None
    init_flag = True
    init_limit = 0.03
    runtime_limit = 0.2
    grip_err_limit = 0.5
    hello_client.set_rgbs(np.array([255, 255, 0]))
    grip_cmd, grip_ratio, grip_threshold = None, 0.7, 0.5
    rate = HexRate(500)
    while True:
        # gello
        hello_states_hdr, hello_states = hello_client.get_states()
        if hello_states_hdr is not None:
            hello_cmds = hello_states[:hexarm_dofs["sum"], :-1].copy()

        # hexarm
        hexarm_states_hdr, hexarm_states = hexarm_client.get_states()
        if hexarm_states_hdr is not None:
            cur_q = hexarm_states[:, 0]
            cur_dq = hexarm_states[:, 1]
            arm_q = cur_q[:hexarm_dofs["robot_arm"]]
            arm_dq = cur_dq[:hexarm_dofs["robot_arm"]]

            _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
            tau_comp = np.zeros(hexarm_dofs["sum"])
            tau_comp[:hexarm_dofs["robot_arm"]] = c_mat @ arm_dq + g_vec

            if hello_cmds is not None:
                grip_tar = None
                if hexarm_dofs["robot_gripper"] is not None:
                    if grip_cmd is None:
                        grip_cmd = hello_cmds[-hexarm_dofs["robot_gripper"]:, 0].copy()
                    else:
                        grip_cmd = hello_cmds[-hexarm_dofs["robot_gripper"]:, 0] * grip_ratio + grip_cmd * (1 - grip_ratio)
                    modified_grip_cmds = np.zeros_like(grip_cmd)
                    large_mask = grip_cmd  > grip_threshold
                    small_mask = grip_cmd < -grip_threshold
                    modified_grip_cmds[large_mask] = 1.0
                    modified_grip_cmds[small_mask] = -1.0
                    grip_tar = gripper_d + gripper_k * modified_grip_cmds
                mid_q, interp_flag = interp_arm(
                    cur_q,
                    hello_cmds[:hexarm_dofs["robot_arm"], 0],
                    grip_tar=grip_tar,
                    dofs=hexarm_dofs,
                    err_limit=init_limit if init_flag else runtime_limit,
                    grip_err_limit=grip_err_limit,
                )
                tar_dq = np.zeros(hexarm_dofs["sum"])
                if not interp_flag:
                    if init_flag:
                        init_flag = False
                        print("init finished")
                        hello_client.set_rgbs(np.array([0, 255, 0]))
                    tar_dq[:hexarm_dofs[
                        "robot_arm"]] = hello_cmds[:hexarm_dofs["robot_arm"],
                                                   1].copy()

                cmds = np.zeros((hexarm_dofs["sum"], 5))
                cmds[:, 0] = mid_q
                cmds[:, 1] = tar_dq
                cmds[:, 2] = tau_comp
                cmds[:, 3] = mit_kp[:hexarm_dofs["sum"]]
                cmds[:, 4] = mit_kd[:hexarm_dofs["sum"]]
                hexarm_client.set_cmds(cmds)

        rate.sleep()


if __name__ == '__main__':
    main()
