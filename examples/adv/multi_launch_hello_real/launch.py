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
    "hello_real_0":
    (f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/hello_real/launch.py", None),
    "hello_real_1":
    (f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/hello_real/launch.py", None),
}

# robot model config
ARM_TYPE = "archer_y6"
GRIPPER_TYPE = "gp80"

# hello_real 0
HELLO_REAL_0_HELLO_SRV_PORT = 12345
HELLO_REAL_0_HEXARM_SRV_PORT = 12346
HELLO_REAL_0_HELLO_DEVICE_IP = "172.18.10.251"
HELLO_REAL_0_HELLO_DEVICE_PORT = 8439
HELLO_REAL_0_HEXARM_DEVICE_IP = "172.18.22.245"
HELLO_REAL_0_HEXARM_DEVICE_PORT = 8439
# hello_real 1
HELLO_REAL_1_HELLO_SRV_PORT = 12347
HELLO_REAL_1_HEXARM_SRV_PORT = 12348
HELLO_REAL_1_HELLO_DEVICE_IP = "172.18.10.251"
HELLO_REAL_1_HELLO_DEVICE_PORT = 9439
HELLO_REAL_1_HEXARM_DEVICE_IP = "172.18.22.245"
HELLO_REAL_1_HEXARM_DEVICE_PORT = 9439

# params
MIT_KP = [200.0, 200.0, 200.0, 75.0, 15.0, 15.0, 20.0]
MIT_KD = [10.0, 10.0, 10.0, 6.0, 0.31, 0.31, 1.0]
LAUNCH_PARAMS_DICT = {
    "hello_real_0": {
        "hello_real_cli": {
            "name": "hello_real_cli_0",
            "cfg": {
                "model_path": HEXARM_URDF_PATH_DICT[f"{ARM_TYPE}_{GRIPPER_TYPE}"],
                "last_link": "link_6",
                "ctrl_cfg": {
                    "mit_kp": MIT_KP,
                    "mit_kd": MIT_KD,
                },
                "hello_net_cfg": {
                    "port": HELLO_REAL_0_HELLO_SRV_PORT,
                },
                "hexarm_net_cfg": {
                    "port": HELLO_REAL_0_HEXARM_SRV_PORT,
                },
            },
        },
        "robot_hello_srv": {
            "name": "robot_hello_srv_0",
            "cfg": {
                "net": {
                    "port": HELLO_REAL_0_HELLO_SRV_PORT,
                },
                "params": {
                    "device_ip": HELLO_REAL_0_HELLO_DEVICE_IP,
                    "device_port": HELLO_REAL_0_HELLO_DEVICE_PORT,
                    "control_hz": 500,
                    "arm_type": ARM_TYPE,
                    "sens_ts": True,
                },
            },
        },
        "robot_hexarm_srv": {
            "name": "robot_hexarm_srv_0",
            "cfg": {
                "net": {
                    "port": HELLO_REAL_0_HEXARM_SRV_PORT,
                },
                "params": {
                    "device_ip": HELLO_REAL_0_HEXARM_DEVICE_IP,
                    "device_port": HELLO_REAL_0_HEXARM_DEVICE_PORT,
                    "control_hz": 500,
                    "arm_type": ARM_TYPE,
                    "mit_kp": MIT_KP,
                    "mit_kd": MIT_KD,
                    "sens_ts": True,
                }
            }
        },
    },
    "hello_real_1": {
        "hello_real_cli": {
            "name": "hello_real_cli_1",
            "cfg": {
                "model_path": HEXARM_URDF_PATH_DICT[f"{ARM_TYPE}_{GRIPPER_TYPE}"],
                "last_link": "link_6",
                "ctrl_cfg": {
                    "mit_kp": MIT_KP,
                    "mit_kd": MIT_KD,
                },
                "hello_net_cfg": {
                    "port": HELLO_REAL_1_HELLO_SRV_PORT,
                },
                "hexarm_net_cfg": {
                    "port": HELLO_REAL_1_HEXARM_SRV_PORT,
                },
            },
        },
        "robot_hello_srv": {
            "name": "robot_hello_srv_1",
            "cfg": {
                "net": {
                    "port": HELLO_REAL_1_HELLO_SRV_PORT,
                },
                "params": {
                    "device_ip": HELLO_REAL_1_HELLO_DEVICE_IP,
                    "device_port": HELLO_REAL_1_HELLO_DEVICE_PORT,
                    "control_hz": 500,
                    "arm_type": ARM_TYPE,
                    "sens_ts": True,
                },
            },
        },
        "robot_hexarm_srv": {
            "name": "robot_hexarm_srv_1",
            "cfg": {
                "net": {
                    "port": HELLO_REAL_1_HEXARM_SRV_PORT,
                },
                "params": {
                    "device_ip": HELLO_REAL_1_HEXARM_DEVICE_IP,
                    "device_port": HELLO_REAL_1_HEXARM_DEVICE_PORT,
                    "control_hz": 500,
                    "arm_type": ARM_TYPE,
                    "mit_kp": MIT_KP,
                    "mit_kd": MIT_KD,
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
