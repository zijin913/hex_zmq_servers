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
    "force_feedback_0":
    (f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/force_feedback/launch.py", None),
    "force_feedback_1":
    (f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/force_feedback/launch.py", None),
}

# robot model config
ARM_TYPE = "archer_y6"
GRIPPER_TYPE = "gp100"

# device config
MASTER_DEVICE_IP = "172.18.20.196"
SLAVE_DEVICE_IP = "172.18.7.47"
# force feedback 0
FORCE_FEEDBACK_0_MASTER_SRV_PORT = 12345
FORCE_FEEDBACK_0_SLAVE_SRV_PORT = 12346
FORCE_FEEDBACK_0_MASTER_DEVICE_IP = MASTER_DEVICE_IP
FORCE_FEEDBACK_0_SLAVE_DEVICE_IP = SLAVE_DEVICE_IP
FORCE_FEEDBACK_0_MASTER_DEVICE_PORT = 8439
FORCE_FEEDBACK_0_SLAVE_DEVICE_PORT = 8439
# force feedback 1
FORCE_FEEDBACK_1_MASTER_SRV_PORT = 12347
FORCE_FEEDBACK_1_SLAVE_SRV_PORT = 12348
FORCE_FEEDBACK_1_MASTER_DEVICE_IP = MASTER_DEVICE_IP
FORCE_FEEDBACK_1_SLAVE_DEVICE_IP = SLAVE_DEVICE_IP
FORCE_FEEDBACK_1_MASTER_DEVICE_PORT = 9439
FORCE_FEEDBACK_1_SLAVE_DEVICE_PORT = 9439

LAUNCH_PARAMS_DICT = {
    "force_feedback_0": {
        "force_feedback_cli": {
            "name": "force_feedback_cli_0",
            "cfg": {
                "model_path":
                HEXARM_URDF_PATH_DICT[f"{ARM_TYPE}_{GRIPPER_TYPE}"],
                "last_link": "link_6",
                "hexarm_master_net_cfg": {
                    "port": FORCE_FEEDBACK_0_MASTER_SRV_PORT,
                },
                "hexarm_slave_net_cfg": {
                    "port": FORCE_FEEDBACK_0_SLAVE_SRV_PORT,
                },
            },
        },
        "robot_hexarm_master_srv": {
            "name": "robot_hexarm_master_srv_0",
            "cfg": {
                "net": {
                    "port": FORCE_FEEDBACK_0_MASTER_SRV_PORT,
                },
                "params": {
                    "device_ip": FORCE_FEEDBACK_0_MASTER_DEVICE_IP,
                    "device_port": FORCE_FEEDBACK_0_MASTER_DEVICE_PORT,
                    "control_hz": 500,
                    "arm_type": ARM_TYPE,
                    "mit_kp": [0.0] * 7,
                    "mit_kd": [0.0] * 7,
                    "sens_ts": True,
                }
            }
        },
        "robot_hexarm_slave_srv": {
            "name": "robot_hexarm_slave_srv_0",
            "cfg": {
                "net": {
                    "port": FORCE_FEEDBACK_0_SLAVE_SRV_PORT,
                },
                "params": {
                    "device_ip": FORCE_FEEDBACK_0_SLAVE_DEVICE_IP,
                    "device_port": FORCE_FEEDBACK_0_SLAVE_DEVICE_PORT,
                    "control_hz": 500,
                    "arm_type": ARM_TYPE,
                    "mit_kp": [200.0, 200.0, 200.0, 75.0, 15.0, 15.0, 20.0],
                    "mit_kd": [10.0, 10.0, 10.0, 6.0, 0.31, 0.31, 1.0],
                    "sens_ts": True,
                }
            }
        },
    },
    "force_feedback_1": {
        "force_feedback_cli": {
            "name": "force_feedback_cli_1",
            "cfg": {
                "model_path":
                HEXARM_URDF_PATH_DICT[f"{ARM_TYPE}_{GRIPPER_TYPE}"],
                "last_link": "link_6",
                "hexarm_master_net_cfg": {
                    "port": FORCE_FEEDBACK_1_MASTER_SRV_PORT,
                },
                "hexarm_slave_net_cfg": {
                    "port": FORCE_FEEDBACK_1_SLAVE_SRV_PORT,
                },
            },
        },
        "robot_hexarm_master_srv": {
            "name": "robot_hexarm_master_srv_1",
            "cfg": {
                "net": {
                    "port": FORCE_FEEDBACK_1_MASTER_SRV_PORT,
                },
                "params": {
                    "device_ip": FORCE_FEEDBACK_1_MASTER_DEVICE_IP,
                    "device_port": FORCE_FEEDBACK_1_MASTER_DEVICE_PORT,
                    "control_hz": 500,
                    "arm_type": ARM_TYPE,
                    "mit_kp": [0.0] * 7,
                    "mit_kd": [0.0] * 7,
                    "sens_ts": True,
                }
            }
        },
        "robot_hexarm_slave_srv": {
            "name": "robot_hexarm_slave_srv_1",
            "cfg": {
                "net": {
                    "port": FORCE_FEEDBACK_1_SLAVE_SRV_PORT,
                },
                "params": {
                    "device_ip": FORCE_FEEDBACK_1_SLAVE_DEVICE_IP,
                    "device_port": FORCE_FEEDBACK_1_SLAVE_DEVICE_PORT,
                    "control_hz": 500,
                    "arm_type": ARM_TYPE,
                    "mit_kp": [200.0, 200.0, 200.0, 75.0, 15.0, 15.0, 20.0],
                    "mit_kd": [10.0, 10.0, 10.0, 6.0, 0.31, 0.31, 1.0],
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
