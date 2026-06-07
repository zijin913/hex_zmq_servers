#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-14
################################################################

from ..robot_base import HexRobotClientBase
from hex_robo_utils import (
    HexRate,
    hex_ts_now,
)

import numpy as np
from collections import deque

NET_CONFIG = {
    "ip": "127.0.0.1",
    "port": 12345,
    "realtime_mode": False,
    "deque_maxlen": 10,
    "client_timeout_ms": 200,
    "server_timeout_ms": 1_000,
    "server_num_workers": 4,
}

ROBOT_CONFIG = {
    "device_ip": "172.18.8.161",
    "device_port": 8439,
    "control_hz": 250,
    "sens_ts": True,
}


class HexRobotHelloClient(HexRobotClientBase):

    def __init__(
        self,
        net_config: dict = NET_CONFIG,
    ):
        HexRobotClientBase.__init__(self, net_config)
        self._rgbs_seq = 0
        self._rgbs_queue = deque(maxlen=1)
        self._wait_for_working()

    def set_rgbs(self, rgbs: np.ndarray):
        self._rgbs_queue.append(rgbs)

    def _set_rgbs_inner(self, rgbs: np.ndarray) -> bool:
        hdr, rgbs = self.request(
            {
                "cmd": "set_rgbs",
                "ts": hex_ts_now(),
                "args": self._rgbs_seq,
            },
            rgbs,
        )
        # print(f"set_rgbs seq: {self._rgbs_seq}")
        try:
            cmd = hdr["cmd"]
            if cmd == "set_rgbs_ok":
                self._rgbs_seq = (self._rgbs_seq + 1) % self._max_seq_num
                return True
            else:
                return False
        except KeyError:
            print(f"\033[91m{hdr['cmd']} requires `cmd`\033[0m")
            return False
        except Exception as e:
            print(f"\033[91mset_rgbs failed: {e}\033[0m")
            return False

    def _recv_loop(self):
        rate = HexRate(2000)
        while self._recv_flag:
            hdr, states = self._get_states_inner()
            if hdr is not None:
                self._states_queue.append((hdr, states))

            try:
                rgbs = self._rgbs_queue[-1]
                _ = self._set_rgbs_inner(rgbs)
            except IndexError:
                pass

            rate.sleep()
