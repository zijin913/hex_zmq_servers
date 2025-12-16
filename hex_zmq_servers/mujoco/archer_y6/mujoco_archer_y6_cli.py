#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-12
################################################################

from ..mujoco_base import HexMujocoClientBase
from ...zmq_base import HexRate

NET_CONFIG = {
    "ip": "127.0.0.1",
    "port": 12345,
    "realtime_mode": False,
    "deque_maxlen": 10,
    "client_timeout_ms": 200,
    "server_timeout_ms": 1_000,
    "server_num_workers": 4,
}

RECV_CONFIG = {
    "rgb": True,
    "depth": True,
    "obj": True,
}


class HexMujocoArcherY6Client(HexMujocoClientBase):

    def __init__(
        self,
        net_config: dict = NET_CONFIG,
        recv_config: dict = RECV_CONFIG,
    ):
        HexMujocoClientBase.__init__(self, net_config)
        self.__recv_config = recv_config
        self._wait_for_working()

    def _recv_loop(self):
        rate = HexRate(2000)
        image_trig_cnt = 0
        while self._recv_flag:
            hdr, states = self._get_states_inner("robot")
            if hdr is not None:
                self._states_queue["robot"].append((hdr, states))
            if self.__recv_config["obj"]:
                hdr, obj_pose = self._get_states_inner("obj")
                if hdr is not None:
                    self._states_queue["obj"].append((hdr, obj_pose))

            image_trig_cnt += 1
            if image_trig_cnt >= 10:
                image_trig_cnt = 0
                if self.__recv_config["rgb"]:
                    hdr, img = self._get_rgb_inner()
                    if hdr is not None:
                        self._camera_queue["rgb"].append((hdr, img))
                if self.__recv_config["depth"]:
                    hdr, img = self._get_depth_inner()
                    if hdr is not None:
                        self._camera_queue["depth"].append((hdr, img))

            try:
                cmds = self._cmds_queue[-1]
                _ = self._set_cmds_inner(cmds)
            except IndexError:
                pass

            rate.sleep()
