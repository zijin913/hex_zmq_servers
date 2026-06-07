#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-12
################################################################

from ..cam_base import HexCamClientBase

from hex_robo_utils import HexRate

NET_CONFIG = {
    "ip": "127.0.0.1",
    "port": 12345,
    "realtime_mode": False,
    "deque_maxlen": 10,
    "client_timeout_ms": 200,
    "server_timeout_ms": 1_000,
    "server_num_workers": 4,
}


class HexCamRGBClient(HexCamClientBase):

    def __init__(
        self,
        net_config: dict = NET_CONFIG,
    ):
        HexCamClientBase.__init__(self, net_config)
        self._wait_for_working()

    def get_intri(self):
        intri_hdr, intri = self.request({"cmd": "get_intri"})
        return intri_hdr, intri

    def _recv_loop(self):
        rate = HexRate(200)
        while self._recv_flag:
            hdr, img = self._get_rgb_inner()
            if hdr is not None:
                self._rgb_queue.append((hdr, img))
            rate.sleep()
