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

    hello_client = HexRobotHexarmClient(net_config=hello_net_cfg)
    hexarm_client = HexRobotHexarmClient(net_config=hexarm_net_cfg)
    dyn_util = DynUtil(model_path, last_link)

    # wait servers to work
    if not wait_client_working(hello_client):
        hex_log(HEX_LOG_LEVEL["err"], "hello server is not working")
        return
    if not wait_client_working(hexarm_client):
        hex_log(HEX_LOG_LEVEL["err"], "hexarm server is not working")
        return

    dof_arr = hexarm_client.get_dofs()
    dofs = {
        "robot_arm": dof_arr[0],
        "robot_gripper": dof_arr[1] if len(dof_arr) > 1 else None,
        "sum": dof_arr.sum(),
    }
    hex_log(HEX_LOG_LEVEL["info"], f"dofs: {dofs}")

    # work loop
    rate = HexRate(500)
    hello_cmds = None
    while True:
        # gello
        hello_states_hdr, hello_states = hello_client.get_states()
        if hello_states_hdr is not None:
            hello_cmds = hello_states[:dofs["sum"], :-1].copy()

        # hexarm
        hexarm_states_hdr, hexarm_states = hexarm_client.get_states()
        if hexarm_states_hdr is not None:
            arm_q = hexarm_states[:dofs["robot_arm"], 0]
            arm_dq = hexarm_states[:dofs["robot_arm"], 1]

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
                hexarm_client.set_cmds(cmds)

        rate.sleep()


if __name__ == '__main__':
    main()
