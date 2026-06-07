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
ARM_TYPE = "archer_y6"
GRIPPER_TYPE = "gp100_p050"

# server ports
HELLO_SRV_PORT = 12345
MUJOCO_SRV_PORT = 12346

# device config
DEVICE_IP = "172.18.22.245"
HELLO_DEVICE_PORT = 9439

# node params
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HEX_ZMQ_SERVERS_DIR = f"{SCRIPT_DIR}/../../../hex_zmq_servers"
MIT_KP = [400.0, 400.0, 500.0, 200.0, 100.0, 100.0, 100.0]
MIT_KD = [5.0, 5.0, 5.0, 5.0, 2.0, 2.0, 2.0]
NODE_PARAMS_DICT = {
    "hello_sim_cli": {
        "name": "hello_sim_cli",
        "node_path": f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/hello_sim/cli.py",
        "cfg_path":
        f"{HEX_ZMQ_SERVERS_DIR}/../examples/adv/hello_sim/cli.json",
        "cfg": {
            "model_path": HEXARM_URDF_PATH_DICT[f"{ARM_TYPE}_{GRIPPER_TYPE}"],
            "last_link": "link_6",
            "ctrl_cfg": {
                "mit_kp": MIT_KP,
                "mit_kd": MIT_KD,
            },
            "hello_net_cfg": {
                "port": HELLO_SRV_PORT,
            },
            "mujoco_net_cfg": {
                "port": MUJOCO_SRV_PORT,
            },
        },
    },
    "robot_hello_srv": {
        "name": "robot_hello_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["robot_hello"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["robot_hello"],
        "cfg": {
            "net": {
                "port": HELLO_SRV_PORT,
            },
            "params": {
                "device_ip": DEVICE_IP,
                "device_port": HELLO_DEVICE_PORT,
                "control_hz": 500,
                "arm_type": ARM_TYPE,
                "sens_ts": True,
            },
        },
    },
    "mujoco_archer_y6_srv": {
        "name": "mujoco_archer_y6_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["mujoco_archer_y6"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["mujoco_archer_y6"],
        "cfg": {
            "net": {
                "port": MUJOCO_SRV_PORT,
            },
            "params": {
                "states_rate": 500,
                "img_rate": 30,
                "tau_ctrl": False,
                "mit_kp": MIT_KP,
                "mit_kd": MIT_KD,
                "cam_type": "empty",
                "headless": False,
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
