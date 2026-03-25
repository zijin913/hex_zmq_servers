#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# SO-101 Leader → Real HexArm L6Y Teleoperation
# Cartesian space mapping: SO-101 FK → workspace scale → HexArm IK
################################################################

import os
from hex_zmq_servers import HexLaunch, HexNodeConfig
from hex_zmq_servers import HEX_ZMQ_SERVERS_PATH_DICT, HEX_ZMQ_CONFIGS_PATH_DICT
from hex_zmq_servers import HEXARM_URDF_PATH_DICT

# robot model config
ARM_TYPE = "archer_l6y"
GRIPPER_TYPE = "gp100"
if GRIPPER_TYPE == "empty":
    USE_GRIPPER = False
else:
    USE_GRIPPER = True

# server ports
SO101_SRV_PORT = 12345
HEXARM_SRV_PORT = 12346

# device config
SO101_DEVICE = "/dev/ttyACM0"
DEVICE_IP = "10.42.0.45"
HEXARM_DEVICE_PORT = 8439

# paths
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HEX_ZMQ_SERVERS_DIR = f"{SCRIPT_DIR}/../../../hex_zmq_servers"
SO101_URDF_PATH = f"{HEX_ZMQ_SERVERS_DIR}/robot/so101/urdf/so101.urdf"

# node params
NODE_PARAMS_DICT = {
    "so101_real_cli": {
        "name": "so101_real_cli",
        "node_path":
        f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/so101_real/cli.py",
        "cfg_path":
        f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/so101_real/cli.json",
        "cfg": {
            "so101_urdf_path": SO101_URDF_PATH,
            "hexarm_model_path":
            HEXARM_URDF_PATH_DICT[f"{ARM_TYPE}_{GRIPPER_TYPE}"],
            "last_link": "link_6",
            "use_gripper": USE_GRIPPER,
            "workspace_scale": 1.5,
            "gripper_scale": 1.0,
            "so101_net_cfg": {
                "ip": "127.0.0.1",
                "port": SO101_SRV_PORT,
            },
            "hexarm_net_cfg": {
                "ip": "127.0.0.1",
                "port": HEXARM_SRV_PORT,
            },
        },
    },
    "robot_so101_srv": {
        "name": "robot_so101_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["robot_so101"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["robot_so101"],
        "cfg": {
            "net": {
                "port": SO101_SRV_PORT,
            },
            "params": {
                "idxs": [1, 2, 3, 4, 5, 6],
                "invs": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
                "range_midpoints": [2029, 2025, 1938, 2095, 2047, 2069],
                "limits": [
                    [-2.7, 2.7],
                    [-2.09, 2.09],
                    [-3.14, 3.14],
                    [-1.57, 1.57],
                    [-3.14, 3.14],
                    [0.0, 1.0],
                ],
                "device": SO101_DEVICE,
                "baudrate": 1000000,
                "max_retries": 3,
                "sens_ts": True,
            },
        },
    },
    "robot_hexarm_srv": {
        "name": "robot_hexarm_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["robot_hexarm"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["robot_hexarm"],
        "cfg": {
            "net": {
                "port": HEXARM_SRV_PORT,
            },
            "params": {
                "device_ip": DEVICE_IP,
                "device_port": HEXARM_DEVICE_PORT,
                "control_hz": 500,
                "arm_type": ARM_TYPE,
                "use_gripper": USE_GRIPPER,
                "mit_kp": [200.0, 200.0, 200.0, 75.0, 15.0, 15.0, 20.0],
                "mit_kd": [10.0, 10.0, 10.0, 6.0, 0.31, 0.31, 1.0],
                "sens_ts": True,
            }
        }
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
