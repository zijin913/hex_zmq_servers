#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-15
################################################################

from .mujoco_base import HexMujocoBase, HexMujocoClientBase, HexMujocoServerBase
from .archer_y6 import HexMujocoArcherY6, HexMujocoArcherY6Client, HexMujocoArcherY6Server
from .archer_l6y import HexMujocoArcherL6Y, HexMujocoArcherL6YClient, HexMujocoArcherL6YServer
from .archer_l6y import HexMujocoArcherL6YDual, HexMujocoArcherL6YDualClient, HexMujocoArcherL6YDualServer
from .firefly_y6 import HexMujocoFireflyY6Dual, HexMujocoFireflyY6DualClient, HexMujocoFireflyY6DualServer
from .e3_desktop import HexMujocoE3Desktop, HexMujocoE3DesktopClient, HexMujocoE3DesktopServer

__all__ = [
    # base
    "HexMujocoBase",
    "HexMujocoClientBase",
    "HexMujocoServerBase",

    # archer_y6
    "HexMujocoArcherY6",
    "HexMujocoArcherY6Client",
    "HexMujocoArcherY6Server",

    # archer_l6y
    "HexMujocoArcherL6Y",
    "HexMujocoArcherL6YClient",
    "HexMujocoArcherL6YServer",
    "HexMujocoArcherL6YDual",
    "HexMujocoArcherL6YDualClient",
    "HexMujocoArcherL6YDualServer",

    # firefly_y6 (dual-arm only; single-arm uses archer_l6y)
    "HexMujocoFireflyY6Dual",
    "HexMujocoFireflyY6DualClient",
    "HexMujocoFireflyY6DualServer",

    # e3_desktop
    "HexMujocoE3Desktop",
    "HexMujocoE3DesktopClient",
    "HexMujocoE3DesktopServer",
]
