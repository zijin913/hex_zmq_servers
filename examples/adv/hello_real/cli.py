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
    HexRate,
    HEX_LOG_LEVEL,
    hex_log,
    HexRobotHelloClient,
    HexRobotHexarmClient,
)
from hex_robo_utils import HexDynUtil as DynUtil


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
               grip_flag=True,
               dofs: dict = None,
               err_limit=0.05):
    mid_joint = np.zeros(dofs["sum"])
    mid_joint[:dofs["robot_arm"]], interp_flag = interp_joint(
        cur_q[:dofs["robot_arm"]],
        tar_joint,
        err_limit=err_limit,
    )
    if dofs["robot_gripper"] is not None:
        mid_joint[-dofs["robot_gripper"]:], _ = interp_joint(
            cur_q[-dofs["robot_gripper"]:],
            1.33 if grip_flag else 0.2,
            err_limit=err_limit,
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

    # work loop
    hello_cmds = None
    init_flag = True
    init_limit = 0.03
    runtime_limit = 0.1
    hello_client.set_rgbs(np.array([255, 255, 0]))
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
                mid_q, interp_flag = interp_arm(
                    cur_q,
                    hello_cmds[:hexarm_dofs["robot_arm"], 0],
                    grip_flag=hello_cmds[-1, 0] > 0.5,
                    dofs=hexarm_dofs,
                    err_limit=init_limit if init_flag else runtime_limit,
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
                cmds[:, 3] = mit_kp
                cmds[:, 4] = mit_kd
                hexarm_client.set_cmds(cmds)

        rate.sleep()


if __name__ == '__main__':
    main()
