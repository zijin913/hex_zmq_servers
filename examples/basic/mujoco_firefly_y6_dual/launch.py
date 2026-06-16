#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""
MuJoCo Dual-Arm Firefly Y6 Launch Script

Launches a single MuJoCo simulation with two Firefly Y6 arms (left + right)
sharing one physics scene. Both arms are controlled via the same ZMQ port.

Usage:
    source hex_zmq_servers/.venv/bin/activate
    python hex_zmq_servers/examples/basic/mujoco_firefly_y6_dual/launch.py
"""

import os
from hex_zmq_servers import HexLaunch, HexNodeConfig
from hex_zmq_servers import HEX_ZMQ_SERVERS_PATH_DICT, HEX_ZMQ_CONFIGS_PATH_DICT

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
NODE_PARAMS_DICT = {
    "mujoco_firefly_y6_dual_srv": {
        "name": "mujoco_firefly_y6_dual_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["mujoco_firefly_y6_dual"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["mujoco_firefly_y6_dual"],
        "cfg": {
            "net": {
                "ip": "127.0.0.1",
                "port": 12345,
            },
            "params": {
                "mit_kp":
                [1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0, 1500.0],
                "mit_kd": [20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0],
                "cam_type": ["realsense", "realsense"],
                "scene_name": "scene_dual",
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
