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
    HexRobotGelloClient,
    HexMujocoE3DesktopClient,
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
        left_gello_net_cfg = cfg["left_gello_net_cfg"]
        right_gello_net_cfg = cfg["right_gello_net_cfg"]
        mujoco_net_cfg = cfg["mujoco_net_cfg"]
    except KeyError as ke:
        missing_key = ke.args[0]
        raise ValueError(f"cfg is not valid, missing key: {missing_key}")

    left_gello_client = HexRobotGelloClient(net_config=left_gello_net_cfg)
    right_gello_client = HexRobotGelloClient(net_config=right_gello_net_cfg)
    mujoco_client = HexMujocoE3DesktopClient(net_config=mujoco_net_cfg)
    dyn_util = DynUtil(model_path, last_link)

    # wait servers to work
    if not wait_client_working(left_gello_client):
        hex_log(HEX_LOG_LEVEL["err"], "left_gello server is not working")
        return
    if not wait_client_working(right_gello_client):
        hex_log(HEX_LOG_LEVEL["err"], "right_gello server is not working")
        return
    if not wait_client_working(mujoco_client):
        hex_log(HEX_LOG_LEVEL["err"], "mujoco server is not working")
        return

    dof_arr = mujoco_client.get_dofs()
    dofs = {
        "robot_arm": dof_arr[0],
        "robot_gripper": dof_arr[1] if len(dof_arr) > 1 else None,
        "sum": dof_arr.sum(),
    }
    hex_log(HEX_LOG_LEVEL["info"], f"dofs: {dofs}")

    # work loop
    rate = HexRate(250)
    left_gello_cmds = None
    right_gello_cmds = None
    try:
        while True:
            # gello
            left_gello_states_hdr, left_gello_states = left_gello_client.get_states(
            )
            if left_gello_states_hdr is not None:
                left_gello_cmds = left_gello_states.copy()
            right_gello_states_hdr, right_gello_states = right_gello_client.get_states(
            )
            if right_gello_states_hdr is not None:
                right_gello_cmds = right_gello_states.copy()

            # left
            left_states_hdr, left_states = mujoco_client.get_states("left")
            if left_states_hdr is not None:
                arm_q = left_states[:, 0][dofs["robot_arm"]]
                arm_dq = left_states[:, 1][dofs["robot_arm"]]
                _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
                tau_comp = np.zeros(dofs["sum"])
                tau_comp[dofs["robot_arm"]] = c_mat @ arm_dq + g_vec
                if left_gello_cmds is not None:
                    cmds = np.concatenate(
                        (left_gello_cmds.reshape(-1, 1), tau_comp.reshape(
                            -1, 1)),
                        axis=1)
                    mujoco_client.set_cmds(cmds, "left")

            # right
            right_states_hdr, right_states = mujoco_client.get_states("right")
            if right_states_hdr is not None:
                arm_q = right_states[:, 0][dofs["robot_arm"]]
                arm_dq = right_states[:, 1][dofs["robot_arm"]]

                _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
                tau_comp = np.zeros(dofs["sum"])
                tau_comp[dofs["robot_arm"]] = c_mat @ arm_dq + g_vec

                if right_gello_cmds is not None:
                    cmds = np.concatenate((right_gello_cmds.reshape(
                        -1, 1), tau_comp.reshape(-1, 1)),
                                          axis=1)
                    mujoco_client.set_cmds(cmds, "right")

            rate.sleep()
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
