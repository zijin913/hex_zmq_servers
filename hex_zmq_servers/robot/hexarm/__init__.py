#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-12
################################################################

from .robot_hexarm import HexRobotHexarm
from .robot_hexarm_cli import HexRobotHexarmClient
from .robot_hexarm_srv import HexRobotHexarmServer

import os

urdf_dir = os.path.join(os.path.dirname(__file__), "urdf")
HEXARM_URDF_PATH_DICT = {
    "firefly_y6_empty":
    f"{urdf_dir}/firefly_y6/empty.urdf",
    "firefly_y6_gr100":
    f"{urdf_dir}/firefly_y6/gr100.urdf",
    "y6_gr100":
    f"{urdf_dir}/firefly_y6/y6_gr100.urdf",
}

__all__ = [
    # path
    "HEXARM_URDF_PATH_DICT",

    # robot
    "HexRobotHexarm",
    "HexRobotHexarmClient",
    "HexRobotHexarmServer",
]
