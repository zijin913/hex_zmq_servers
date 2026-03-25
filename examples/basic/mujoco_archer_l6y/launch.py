#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
MuJoCo Archer L6Y Launch Script
"""

import os
from hex_zmq_servers import HexLaunch, HexNodeConfig
from hex_zmq_servers import HEX_ZMQ_SERVERS_PATH_DICT, HEX_ZMQ_CONFIGS_PATH_DICT

# node params
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HEX_ZMQ_SERVERS_DIR = f"{SCRIPT_DIR}/../../../hex_zmq_servers"
NODE_PARAMS_DICT = {
    # srv only (no cli for basic test)
    "mujoco_archer_l6y_srv": {
        "name": "mujoco_archer_l6y_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["mujoco_archer_l6y"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["mujoco_archer_l6y"],
        "cfg": {
            "net": {
                "ip": "127.0.0.1",
                "port": 12345,
            },
            "params": {
                "mit_kp":
                [1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0],
                "mit_kd": [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0],
                "cam_type": "realsense",
                # 场景切换: "scene"(默认,物体场景) 或 "scene_table"(桌子场景)
                "scene_name": "scene",
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
