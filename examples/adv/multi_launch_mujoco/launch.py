#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import os
from hex_zmq_servers import HexLaunch, HexNodeConfig

# node params
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HEX_ZMQ_SERVERS_DIR = f"{SCRIPT_DIR}/../../../hex_zmq_servers"

LAUNCH_PATH_DICT = {
    "mujoco_archer_y6_0":
    (f"{HEX_ZMQ_SERVERS_DIR}/../examples/basic/mujoco_archer_y6/launch.py", None),
    "mujoco_archer_y6_1":
    (f"{HEX_ZMQ_SERVERS_DIR}/../examples/basic/mujoco_archer_y6/launch.py", None),
    "mujoco_archer_y6_2":
    (f"{HEX_ZMQ_SERVERS_DIR}/../examples/basic/mujoco_archer_y6/launch.py", None),
}

LAUNCH_PARAMS_DICT = {
    "mujoco_archer_y6_0": {
        # cli
        "mujoco_archer_y6_cli": {
            "name": "mujoco_archer_y6_cli_0",
            "cfg": {
                "net": {
                    "ip": "127.0.0.1",
                    "port": 12345,
                },
            },
        },
        # srv
        "mujoco_archer_y6_srv": {
            "name": "mujoco_archer_y6_srv_0",
            "cfg": {
                "net": {
                    "ip": "127.0.0.1",
                    "port": 12345,
                },
                "params": {
                    "control_hz": 500,
                    "mit_kp":
                    [1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0],
                    "mit_kd": [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0],
                    "cam_type": "realsense",
                    "headless": True,
                },
            },
        },
    },
    "mujoco_archer_y6_1": {
        # cli
        "mujoco_archer_y6_cli": {
            "name": "mujoco_archer_y6_cli_1",
            "cfg": {
                "net": {
                    "ip": "127.0.0.1",
                    "port": 12346,
                },
            },
        },
        # srv
        "mujoco_archer_y6_srv": {
            "name": "mujoco_archer_y6_srv_1",
            "cfg": {
                "net": {
                    "ip": "127.0.0.1",
                    "port": 12346,
                },
                "params": {
                    "control_hz": 500,
                    "mit_kp":
                    [1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0],
                    "mit_kd": [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0],
                    "cam_type": "realsense",
                    "headless": True,
                },
            },
        },
    },
    "mujoco_archer_y6_2": {
        # cli
        "mujoco_archer_y6_cli": {
            "name": "mujoco_archer_y6_cli_2",
            "cfg": {
                "net": {
                    "ip": "127.0.0.1",
                    "port": 12347,
                },
            },
        },
        # srv
        "mujoco_archer_y6_srv": {
            "name": "mujoco_archer_y6_srv_2",
            "cfg": {
                "net": {
                    "ip": "127.0.0.1",
                    "port": 12347,
                },
                "params": {
                    "control_hz": 500,
                    "mit_kp":
                    [1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0],
                    "mit_kd": [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0],
                    "cam_type": "realsense",
                    "headless": True,
                },
            },
        },
    },
}


def get_node_cfgs(params_dict: dict = LAUNCH_PARAMS_DICT):
    return HexNodeConfig.get_launch_params_cfgs(
        launch_params_dict=params_dict,
        launch_default_params_dict=LAUNCH_PARAMS_DICT,
        launch_path_dict=LAUNCH_PATH_DICT,
    )


def main():
    node_cfgs = get_node_cfgs()
    launch = HexLaunch(node_cfgs)
    launch.run()


if __name__ == '__main__':
    main()
