#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import argparse, json
from hex_zmq_servers import (
    HexRate,
    hex_zmq_ts_now,
    hex_zmq_ts_delta_ms,
    HEX_LOG_LEVEL,
    hex_log,
    HexRobotHexarmClient,
)
from hex_robo_utils import HexPlotUtilPlotJuggler as HexPlotUtil

import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    try:
        net_config = cfg["net"]
    except KeyError as ke:
        missing_key = ke.args[0]
        raise ValueError(f"cfg is not valid, missing key: {missing_key}")

    # robot client
    client = HexRobotHexarmClient(net_config=net_config)
    plot_util = HexPlotUtil()

    dof_arr = client.get_dofs()
    dofs = {
        "robot_arm": int(dof_arr[0]),
        "robot_gripper": int(dof_arr[1]) if len(dof_arr) > 1 else None,
        "sum": int(dof_arr.sum()),
    }
    limits = client.get_limits()
    hex_log(HEX_LOG_LEVEL["info"], f"dofs: {dofs}")
    hex_log(HEX_LOG_LEVEL["info"], f"limits: {limits.shape}")

    rate = HexRate(500)
    while True:
        states_hdr, states = client.get_states()
        if states_hdr is not None:
            curr_ts = hex_zmq_ts_now()
            hex_log(
                HEX_LOG_LEVEL["info"],
                f"states_seq: {states_hdr['args']}; delay: {hex_zmq_ts_delta_ms(curr_ts, states_hdr['ts'])}ms"
            )
            arm_q = states[:dofs['robot_arm'], 0]
            arm_dq = states[:dofs['robot_arm'], 1]
            plot_dict = {
                "pos": arm_q.tolist(),
                "vel": arm_dq.tolist(),
            }
            plot_util.add_data(
                name="hexarm",
                data=plot_dict,
            )
            plot_util.send_data(clear=True)

        cmds = np.array([
            0.2,
            -1.5,
            3.0,
            0.0,
            0.0,
            0.0,
            0.5,
        ])
        client.set_cmds(cmds[:dofs['sum']])

        rate.sleep()


if __name__ == '__main__':
    main()
