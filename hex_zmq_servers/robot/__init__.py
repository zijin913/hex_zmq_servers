#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-12
################################################################

from .robot_base import HexRobotBase, HexRobotClientBase, HexRobotServerBase
from .dummy import HexRobotDummy, HexRobotDummyClient, HexRobotDummyServer
from .hexarm import HexRobotHexarm, HexRobotHexarmClient, HexRobotHexarmServer, HEXARM_URDF_PATH_DICT

__all__ = [
    # path
    "HEXARM_URDF_PATH_DICT",

    # base
    "HexRobotBase",
    "HexRobotClientBase",
    "HexRobotServerBase",

    # dummy
    "HexRobotDummy",
    "HexRobotDummyClient",
    "HexRobotDummyServer",

    # hexarm
    "HexRobotHexarm",
    "HexRobotHexarmClient",
    "HexRobotHexarmServer",
]

# Check optional dependencies availability
from importlib.util import find_spec

_HAS_DYNAMIXEL = find_spec("dynamixel-sdk") is not None

# Optional: dynamixel
if _HAS_DYNAMIXEL:
    from .gello import HexRobotGello, HexRobotGelloClient, HexRobotGelloServer
    __all__.extend([
        "HexRobotGello",
        "HexRobotGelloClient",
        "HexRobotGelloServer",
    ])
