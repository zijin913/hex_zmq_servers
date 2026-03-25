#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# SO-101 Leader Arm ZMQ Client
################################################################

from ..robot_base import HexRobotClientBase

NET_CONFIG = {
    "ip": "127.0.0.1",
    "port": 12345,
    "realtime_mode": False,
    "deque_maxlen": 10,
    "client_timeout_ms": 200,
    "server_timeout_ms": 1_000,
    "server_num_workers": 4,
}


class HexRobotSO101Client(HexRobotClientBase):

    def __init__(
        self,
        net_config: dict = NET_CONFIG,
    ):
        HexRobotClientBase.__init__(self, net_config)
        self._wait_for_working()
