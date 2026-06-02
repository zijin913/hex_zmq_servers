#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-15
################################################################

from .hex_launch import HexLaunch, HexNodeConfig, HEX_LOG_LEVEL, hex_dict_str, hex_log, hex_err

from .device_base import HexDeviceBase
from .zmq_base import hex_zmq_ts_to_ns, ns_to_hex_zmq_ts, hex_ns_now, hex_zmq_ts_now, hex_zmq_ts_delta_ms
from .zmq_base import HexRate, HexZMQClientBase, HexZMQServerBase, hex_server_helper
from .zmq_base import HexZMQDummyClient, HexZMQDummyServer

from .robot import HexRobotBase, HexRobotClientBase, HexRobotServerBase
from .robot import HexRobotHexarm, HexRobotHexarmClient, HexRobotHexarmServer, HEXARM_URDF_PATH_DICT

from .cam import HexCamBase, HexCamClientBase, HexCamServerBase

import os

file_dir = os.path.dirname(os.path.abspath(__file__))
HEX_ZMQ_SERVERS_PATH_DICT = {
    "robot_hexarm": f"{file_dir}/robot/hexarm/robot_hexarm_srv.py",
}
HEX_ZMQ_CONFIGS_PATH_DICT = {
    "robot_hexarm": f"{file_dir}/config/robot_hexarm.json",
}

__all__ = [
    # version
    "__version__",

    # path
    "HEX_ZMQ_SERVERS_PATH_DICT",
    "HEX_ZMQ_CONFIGS_PATH_DICT",
    "HEXARM_URDF_PATH_DICT",

    # launch
    "HexLaunch",
    "HexNodeConfig",
    "HEX_LOG_LEVEL",
    "hex_dict_str",
    "hex_log",
    "hex_err",

    # base
    "HexDeviceBase",
    "HexRate",
    "hex_zmq_ts_to_ns",
    "ns_to_hex_zmq_ts",
    "hex_ns_now",
    "hex_zmq_ts_now",
    "hex_zmq_ts_delta_ms",
    "HexZMQClientBase",
    "HexZMQServerBase",
    "hex_server_helper",
    "HexZMQDummyClient",
    "HexZMQDummyServer",

    # robot
    "HexRobotBase",
    "HexRobotClientBase",
    "HexRobotServerBase",
    "HexRobotHexarm",
    "HexRobotHexarmClient",
    "HexRobotHexarmServer",

    # camera
    "HexCamBase",
    "HexCamClientBase",
    "HexCamServerBase",
]

# Check optional dependencies availability
from importlib.util import find_spec

_HAS_REALSENSE = find_spec("pyrealsense2") is not None
_HAS_MUJOCO = find_spec("mujoco") is not None

# Optional: realsense
if _HAS_REALSENSE:
    from .cam import HexCamRealsense, HexCamRealsenseClient, HexCamRealsenseServer
    HEX_ZMQ_SERVERS_PATH_DICT.update({
        "cam_realsense":
        f"{file_dir}/cam/realsense/cam_realsense_srv.py",
    })
    HEX_ZMQ_CONFIGS_PATH_DICT.update({
        "cam_realsense":
        f"{file_dir}/config/cam_realsense.json",
    })
    __all__.extend([
        "HexCamRealsense",
        "HexCamRealsenseClient",
        "HexCamRealsenseServer",
    ])

# Optional: mujoco
if _HAS_MUJOCO:
    from .mujoco import HexMujocoBase, HexMujocoClientBase, HexMujocoServerBase
    from .mujoco import HexMujocoFireflyY6Dual, HexMujocoFireflyY6DualClient, HexMujocoFireflyY6DualServer
    HEX_ZMQ_SERVERS_PATH_DICT.update({
        "mujoco_firefly_y6_dual":
        f"{file_dir}/mujoco/firefly_y6/mujoco_firefly_y6_dual_srv.py",
    })
    HEX_ZMQ_CONFIGS_PATH_DICT.update({
        "mujoco_firefly_y6_dual":
        f"{file_dir}/config/mujoco_firefly_y6_dual.json",
    })
    __all__.extend([
        # mujoco
        "HexMujocoBase",
        "HexMujocoClientBase",
        "HexMujocoServerBase",
        "HexMujocoFireflyY6Dual",
        "HexMujocoFireflyY6DualClient",
        "HexMujocoFireflyY6DualServer",
    ])

# print("#### Thanks for using hex_zmq_servers :D ####")
