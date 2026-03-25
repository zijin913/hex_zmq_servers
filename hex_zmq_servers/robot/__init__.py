#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-12
################################################################

from .robot_base import HexRobotBase, HexRobotClientBase, HexRobotServerBase
from .dummy import HexRobotDummy, HexRobotDummyClient, HexRobotDummyServer
from .gello import HexRobotGello, HexRobotGelloClient, HexRobotGelloServer
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

    # gello
    "HexRobotGello",
    "HexRobotGelloClient",
    "HexRobotGelloServer",

    # hexarm
    "HexRobotHexarm",
    "HexRobotHexarmClient",
    "HexRobotHexarmServer",
]

# Optional: SO-101 (requires scservo_sdk / feetech-servo-sdk)
try:
    from .so101 import HexRobotSO101, HexRobotSO101Client, HexRobotSO101Server
    __all__.extend([
        "HexRobotSO101",
        "HexRobotSO101Client",
        "HexRobotSO101Server",
    ])
except ImportError:
    pass
