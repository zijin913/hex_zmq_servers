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

END_POSE = np.array(
    [0.0, 0.0, 0.083, 0.7071068, 0.0, -0.7071068, 0.0],
    dtype=np.float64,
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


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    try:
        model_path = cfg["model_path"]
        last_link = cfg["last_link"]
        use_gripper = cfg["use_gripper"]
        hexarm_net_cfg = cfg["hexarm_net_cfg"]
    except KeyError as ke:
        missing_key = ke.args[0]
        raise ValueError(f"cfg is not valid, missing key: {missing_key}")

    hexarm_client = HexRobotHexarmClient(net_config=hexarm_net_cfg)
    dyn_util = DynUtil(model_path, last_link, end_pose=END_POSE)

    # wait servers to work
    if not wait_client_working(hexarm_client):
        hex_log(HEX_LOG_LEVEL["err"], "hexarm server is not working")
        return

    # work loop
    rate = HexRate(500)
    while True:
        # hexarm
        hexarm_states_hdr, hexarm_states = hexarm_client.get_states()
        if hexarm_states_hdr is not None:
            cur_q = hexarm_states[:, 0]
            cur_dq = hexarm_states[:, 1]
            arm_q = cur_q[:-1] if use_gripper else cur_q
            arm_dq = cur_dq[:-1] if use_gripper else cur_dq
            _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
            tau_comp = c_mat @ arm_dq + g_vec
            if use_gripper:
                tau_comp = np.concatenate((tau_comp, np.zeros(1)), axis=0)
            cmds = np.concatenate(
                (cur_q.reshape(-1, 1), tau_comp.reshape(-1, 1)), axis=1)
            hexarm_client.set_cmds(cmds)

            pos, quat = dyn_util.forward_kinematics(arm_q)[-1]
            print(f"pos: {pos}; quat: {quat}")

        rate.sleep()


if __name__ == '__main__':
    main()
