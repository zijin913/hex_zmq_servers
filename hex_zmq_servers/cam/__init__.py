#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-12
################################################################

from .cam_base import HexCamBase, HexCamClientBase, HexCamServerBase

__all__ = [
    # base
    "HexCamBase",
    "HexCamClientBase",
    "HexCamServerBase",
]

# Check optional dependencies availability
from importlib.util import find_spec

_HAS_REALSENSE = find_spec("pyrealsense2") is not None

# Optional: realsense
if _HAS_REALSENSE:
    from .realsense import HexCamRealsense, HexCamRealsenseClient, HexCamRealsenseServer
    __all__.extend([
        "HexCamRealsense",
        "HexCamRealsenseClient",
        "HexCamRealsenseServer",
    ])
