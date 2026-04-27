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
from .robot import HexRobotDummy, HexRobotDummyClient, HexRobotDummyServer
from .robot import HexRobotGello, HexRobotGelloClient, HexRobotGelloServer
from .robot import HexRobotHexarm, HexRobotHexarmClient, HexRobotHexarmServer, HEXARM_URDF_PATH_DICT

from .cam import HexCamBase, HexCamClientBase, HexCamServerBase
from .cam import HexCamDummy, HexCamDummyClient, HexCamDummyServer
from .cam import HexCamRGB, HexCamRGBClient, HexCamRGBServer

import os

file_dir = os.path.dirname(os.path.abspath(__file__))
HEX_ZMQ_SERVERS_PATH_DICT = {
    "zmq_dummy": f"{file_dir}/zmq_base.py",
    "robot_dummy": f"{file_dir}/robot/dummy/robot_dummy_srv.py",
    "robot_gello": f"{file_dir}/robot/gello/robot_gello_srv.py",
    "robot_hexarm": f"{file_dir}/robot/hexarm/robot_hexarm_srv.py",
    "cam_dummy": f"{file_dir}/cam/dummy/cam_dummy_srv.py",
    "cam_rgb": f"{file_dir}/cam/rgb/cam_rgb_srv.py",
}
HEX_ZMQ_CONFIGS_PATH_DICT = {
    "zmq_dummy": f"{file_dir}/config/zmq_dummy.json",
    "robot_dummy": f"{file_dir}/config/robot_dummy.json",
    "robot_gello": f"{file_dir}/config/robot_gello.json",
    "robot_hexarm": f"{file_dir}/config/robot_hexarm.json",
    "cam_dummy": f"{file_dir}/config/cam_dummy.json",
    "cam_rgb": f"{file_dir}/config/cam_rgb.json",
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
    "HexRobotDummy",
    "HexRobotDummyClient",
    "HexRobotDummyServer",
    "HexRobotGello",
    "HexRobotGelloClient",
    "HexRobotGelloServer",
    "HexRobotHexarm",
    "HexRobotHexarmClient",
    "HexRobotHexarmServer",

    # camera
    "HexCamBase",
    "HexCamClientBase",
    "HexCamServerBase",
    "HexCamDummy",
    "HexCamDummyClient",
    "HexCamDummyServer",
    "HexCamRGB",
    "HexCamRGBClient",
    "HexCamRGBServer",
]

# Check optional dependencies availability
from importlib.util import find_spec

_HAS_BERXEL = find_spec("berxel_py_wrapper") is not None
_HAS_REALSENSE = find_spec("pyrealsense2") is not None
_HAS_MUJOCO = find_spec("mujoco") is not None
_HAS_SCSERVO = find_spec("scservo_sdk") is not None

# Optional: berxel
if _HAS_BERXEL:
    from .cam import HexCamBerxel, HexCamBerxelClient, HexCamBerxelServer
    HEX_ZMQ_SERVERS_PATH_DICT.update({
        "cam_berxel":
        f"{file_dir}/cam/berxel/cam_berxel_srv.py",
    })
    HEX_ZMQ_CONFIGS_PATH_DICT.update({
        "cam_berxel":
        f"{file_dir}/config/cam_berxel.json",
    })
    __all__.extend([
        "HexCamBerxel",
        "HexCamBerxelClient",
        "HexCamBerxelServer",
    ])

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
    from .mujoco import HexMujocoArcherY6, HexMujocoArcherY6Client, HexMujocoArcherY6Server
    from .mujoco import HexMujocoArcherL6Y, HexMujocoArcherL6YClient, HexMujocoArcherL6YServer
    from .mujoco import HexMujocoArcherL6YDual, HexMujocoArcherL6YDualClient, HexMujocoArcherL6YDualServer
    from .mujoco import HexMujocoFireflyY6Dual, HexMujocoFireflyY6DualClient, HexMujocoFireflyY6DualServer
    from .mujoco import HexMujocoE3Desktop, HexMujocoE3DesktopClient, HexMujocoE3DesktopServer
    HEX_ZMQ_SERVERS_PATH_DICT.update({
        "mujoco_archer_y6":
        f"{file_dir}/mujoco/archer_y6/mujoco_archer_y6_srv.py",
        "mujoco_archer_l6y":
        f"{file_dir}/mujoco/archer_l6y/mujoco_archer_l6y_srv.py",
        "mujoco_archer_l6y_dual":
        f"{file_dir}/mujoco/archer_l6y/mujoco_archer_l6y_dual_srv.py",
        "mujoco_firefly_y6_dual":
        f"{file_dir}/mujoco/firefly_y6/mujoco_firefly_y6_dual_srv.py",
        "mujoco_e3_desktop":
        f"{file_dir}/mujoco/e3_desktop/mujoco_e3_desktop_srv.py",
    })
    HEX_ZMQ_CONFIGS_PATH_DICT.update({
        "mujoco_archer_y6":
        f"{file_dir}/config/mujoco_archer_y6.json",
        "mujoco_archer_l6y":
        f"{file_dir}/config/mujoco_archer_l6y.json",
        "mujoco_archer_l6y_dual":
        f"{file_dir}/config/mujoco_archer_l6y_dual.json",
        "mujoco_firefly_y6_dual":
        f"{file_dir}/config/mujoco_firefly_y6_dual.json",
        "mujoco_e3_desktop":
        f"{file_dir}/config/mujoco_e3_desktop.json",
    })
    __all__.extend([
        # mujoco
        "HexMujocoBase",
        "HexMujocoClientBase",
        "HexMujocoServerBase",
        "HexMujocoArcherY6",
        "HexMujocoArcherY6Client",
        "HexMujocoArcherY6Server",
        "HexMujocoArcherL6Y",
        "HexMujocoArcherL6YClient",
        "HexMujocoArcherL6YServer",
        "HexMujocoArcherL6YDual",
        "HexMujocoArcherL6YDualClient",
        "HexMujocoArcherL6YDualServer",
        "HexMujocoFireflyY6Dual",
        "HexMujocoFireflyY6DualClient",
        "HexMujocoFireflyY6DualServer",
        "HexMujocoE3Desktop",
        "HexMujocoE3DesktopClient",
        "HexMujocoE3DesktopServer",
    ])

# Optional: SO-101 (requires scservo_sdk / feetech-servo-sdk)
if _HAS_SCSERVO:
    from .robot import HexRobotSO101, HexRobotSO101Client, HexRobotSO101Server
    HEX_ZMQ_SERVERS_PATH_DICT.update({
        "robot_so101":
        f"{file_dir}/robot/so101/robot_so101_srv.py",
    })
    HEX_ZMQ_CONFIGS_PATH_DICT.update({
        "robot_so101":
        f"{file_dir}/config/robot_so101.json",
    })
    __all__.extend([
        "HexRobotSO101",
        "HexRobotSO101Client",
        "HexRobotSO101Server",
    ])

# print("#### Thanks for using hex_zmq_servers :D ####")
