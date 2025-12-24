#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import os
from hex_zmq_servers import HexLaunch, HexNodeConfig
from hex_zmq_servers import HEXARM_URDF_PATH_DICT

# node params
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HEX_ZMQ_SERVERS_DIR = f"{SCRIPT_DIR}/../../../hex_zmq_servers"

LAUNCH_PATH_DICT = {
    "zero_gravity_0":
    (f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/zero_gravity/launch.py", None),
    "zero_gravity_1":
    (f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/zero_gravity/launch.py", None),
}

# robot model config
ARM_TYPE = "archer_y6"
GRIPPER_TYPE = "empty"
if GRIPPER_TYPE == "empty":
    USE_GRIPPER = False
else:
    USE_GRIPPER = True

# zero gravity 0
ZERO_GRAVITY_0_SRV_PORT = 12345
ZERO_GRAVITY_0_DEVICE_IP = "172.18.18.92"
ZERO_GRAVITY_0_DEVICE_PORT = 8439
# zero gravity 1
ZERO_GRAVITY_1_SRV_PORT = 12346
ZERO_GRAVITY_1_DEVICE_IP = "172.18.18.92"
ZERO_GRAVITY_1_DEVICE_PORT = 9439

LAUNCH_PARAMS_DICT = {
    "zero_gravity_0": {
        "zero_gravity_cli": {
            "name": "zero_gravity_cli_0",
            "cfg": {
                "model_path":
                HEXARM_URDF_PATH_DICT[f"{ARM_TYPE}_{GRIPPER_TYPE}"],
                "last_link": "link_6",
                "use_gripper": USE_GRIPPER,
                "hexarm_net_cfg": {
                    "port": ZERO_GRAVITY_0_SRV_PORT,
                },
            },
        },
        "robot_hexarm_srv": {
            "name": "robot_hexarm_srv_0",
            "cfg": {
                "net": {
                    "port": ZERO_GRAVITY_0_SRV_PORT,
                },
                "params": {
                    "device_ip": ZERO_GRAVITY_0_DEVICE_IP,
                    "device_port": ZERO_GRAVITY_0_DEVICE_PORT,
                    "control_hz": 500,
                    "arm_type": ARM_TYPE,
                    "use_gripper": USE_GRIPPER,
                    "mit_kp": [0.0] * 7,
                    "mit_kd": [0.0] * 7,
                    "sens_ts": True,
                }
            }
        },
    },
    "zero_gravity_1": {
        "zero_gravity_cli": {
            "name": "zero_gravity_cli_1",
            "cfg": {
                "model_path":
                HEXARM_URDF_PATH_DICT[f"{ARM_TYPE}_{GRIPPER_TYPE}"],
                "last_link": "link_6",
                "use_gripper": USE_GRIPPER,
                "hexarm_net_cfg": {
                    "port": ZERO_GRAVITY_1_SRV_PORT,
                },
            },
        },
        "robot_hexarm_srv": {
            "name": "robot_hexarm_srv_1",
            "cfg": {
                "net": {
                    "port": ZERO_GRAVITY_1_SRV_PORT,
                },
                "params": {
                    "device_ip": ZERO_GRAVITY_1_DEVICE_IP,
                    "device_port": ZERO_GRAVITY_1_DEVICE_PORT,
                    "control_hz": 500,
                    "arm_type": ARM_TYPE,
                    "use_gripper": USE_GRIPPER,
                    "mit_kp": [0.0] * 7,
                    "mit_kd": [0.0] * 7,
                    "sens_ts": True,
                }
            }
        },
    },
}


def get_node_cfgs(params_dict: dict = LAUNCH_PARAMS_DICT,
                  launch_arg: dict | None = None):
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
