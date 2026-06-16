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

# device config
DEVICE_IP = "172.18.24.90"
HELLO_DEVICE_PORT = 8439

# node params
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HEX_ZMQ_SERVERS_DIR = f"{SCRIPT_DIR}/../../../hex_zmq_servers"
NODE_PARAMS_DICT = {
    # cli
    "robot_hello_cli": {
        "name": "robot_hello_cli",
        "node_path":
        f"{HEX_ZMQ_SERVERS_DIR}/../examples/basic/robot_hello/cli.py",
        "cfg_path":
        f"{HEX_ZMQ_SERVERS_DIR}/../examples/basic/robot_hello/cli.json",
        "cfg": {
            "net": {
                "ip": "127.0.0.1",
                "port": 12345,
            },
        },
    },
    # srv
    "robot_hello_srv": {
        "name": "robot_hello_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["robot_hello"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["robot_hello"],
        "cfg": {
            "net": {
                "ip": "127.0.0.1",
                "port": 12345,
            },
            "params": {
                "device_ip": DEVICE_IP,
                "device_port": HELLO_DEVICE_PORT,
                "control_hz": 500,
                "arm_type": "archer_y6",
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
