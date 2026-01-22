#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import argparse, json, time
import cv2
import numpy as np
from hex_zmq_servers import (
    HexRate,
    HEX_LOG_LEVEL,
    hex_log,
    HexRobotHexarmClient,
    HexMujocoArcherY6Client,
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
        mujoco_net_cfg = cfg["mujoco_net_cfg"]
    except KeyError as ke:
        missing_key = ke.args[0]
        raise ValueError(f"cfg is not valid, missing key: {missing_key}")

    hello_client = HexRobotHexarmClient(net_config=hello_net_cfg)
    mujoco_client = HexMujocoArcherY6Client(net_config=mujoco_net_cfg)
    dyn_util = DynUtil(model_path, last_link)

    # wait servers to work
    if not wait_client_working(hello_client):
        hex_log(HEX_LOG_LEVEL["err"], "hello server is not working")
        return
    if not wait_client_working(mujoco_client):
        hex_log(HEX_LOG_LEVEL["err"], "mujoco server is not working")
        return

    dof_arr = hello_client.get_dofs()
    dofs = {
        "robot_arm": dof_arr[0],
        "robot_gripper": dof_arr[1] if len(dof_arr) > 1 else None,
        "sum": dof_arr.sum(),
    }
    hex_log(HEX_LOG_LEVEL["info"], f"dofs: {dofs}")

    dof_arr = mujoco_client.get_dofs()
    dofs = {
        "robot_arm": dof_arr[0],
        "robot_gripper": dof_arr[1] if len(dof_arr) > 1 else None,
        "sum": dof_arr.sum(),
    }
    hex_log(HEX_LOG_LEVEL["info"], f"dofs: {dofs}")

    # work loop
    rate = HexRate(250)
    hello_cmds = None
    try:
        while True:
            # gello
            hello_states_hdr, hello_states = hello_client.get_states()
            if hello_states_hdr is not None:
                hello_cmds = hello_states.copy()

            # robot
            robot_states_hdr, robot_states = mujoco_client.get_states("robot")
            if robot_states_hdr is not None:
                arm_q = robot_states[:dofs["robot_arm"], 0]
                arm_dq = robot_states[:dofs["robot_arm"], 1]

                _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
                tau_comp = np.zeros(dofs["sum"])
                tau_comp[:dofs["robot_arm"]] = c_mat @ arm_dq + g_vec

                if hello_cmds is not None:
                    cmds = np.zeros((dofs["sum"], 5))
                    cmds[:, 0] = hello_cmds[:, 0]
                    cmds[:, 1] = hello_cmds[:, 1]
                    cmds[:, 2] = tau_comp
                    cmds[:, 3] = mit_kp
                    cmds[:, 4] = mit_kd
                    mujoco_client.set_cmds(cmds)

            # rgb
            rgb_hdr, rgb = mujoco_client.get_rgb()
            if rgb_hdr is not None:
                cv2.imshow("rgb_img", rgb)

            # depth
            depth_hdr, depth = mujoco_client.get_depth()
            if depth_hdr is not None:
                depth_values = depth.astype(np.float32)
                depth_norm = np.clip((depth_values - 70) / (1000 - 70), 0.0,
                                     1.0)
                depth_u8 = (depth_norm * 255.0).astype(np.uint8)
                depth_cmap = cv2.applyColorMap(depth_u8, cv2.COLORMAP_JET)
                cv2.imshow("depth_cmap", depth_cmap)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break

            rate.sleep()
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
