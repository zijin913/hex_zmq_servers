#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import os
from hex_zmq_servers import HexLaunch, HexNodeConfig
from hex_zmq_servers import HEX_ZMQ_SERVERS_PATH_DICT, HEX_ZMQ_CONFIGS_PATH_DICT
from hex_zmq_servers import HEXARM_URDF_PATH_DICT

# robot model config
ARM_TYPE = "firefly_y6"
GRIPPER_TYPE = "empty"
if GRIPPER_TYPE == "empty":
    USE_GRIPPER = False
else:
    USE_GRIPPER = True

# device config
DEVICE_IP = "172.18.22.245"
HEXARM_DEVICE_PORT = 8439

# node params
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HEX_ZMQ_SERVERS_DIR = f"{SCRIPT_DIR}/../../../hex_zmq_servers"
MIT_KP = [400.0, 400.0, 500.0, 200.0, 100.0, 100.0]
MIT_KD = [5.0, 5.0, 5.0, 5.0, 2.0, 2.0]
NODE_PARAMS_DICT = {
    # cli
    "traj_axis_cli": {
        "name": "traj_axis_cli",
        "node_path":
        f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/traj_axis/cli.py",
        "cfg_path":
        f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/traj_axis/cli.json",
        "cfg": {
            "model_path": HEXARM_URDF_PATH_DICT[f"{ARM_TYPE}_{GRIPPER_TYPE}"],
            "use_gripper": USE_GRIPPER,
            "ctrl_cfg": {
                "mit_kp": MIT_KP,
                "mit_kd": MIT_KD,
            },
            "net": {
                "ip": "127.0.0.1",
                "port": 12345,
                "realtime_mode": True,
            },
        },
    },
    # srv
    "robot_hexarm_srv": {
        "name": "robot_hexarm_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["robot_hexarm"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["robot_hexarm"],
        "cfg": {
            "net": {
                "ip": "127.0.0.1",
                "port": 12345,
                "realtime_mode": True,
            },
            "params": {
                "device_ip": DEVICE_IP,
                "device_port": HEXARM_DEVICE_PORT,
                "control_hz": 1000,
                "arm_type": ARM_TYPE,
                "use_gripper": USE_GRIPPER,
                "mit_kp": MIT_KP,
                "mit_kd": MIT_KD,
                "sens_ts": True,
            },
        },
    },
}


def get_node_cfgs(node_params_dict: dict = NODE_PARAMS_DICT,
                  launch_arg: dict | None = None):
    return HexNodeConfig.parse_node_params_dict(
        node_params_dict,
        NODE_PARAMS_DICT,
    )


def main():
    node_cfgs = get_node_cfgs()
    launch = HexLaunch(node_cfgs)
    launch.run()


if __name__ == '__main__':
    main()
