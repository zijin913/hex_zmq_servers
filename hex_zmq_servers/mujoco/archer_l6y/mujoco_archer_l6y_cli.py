#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-12-30
################################################################

from collections import deque

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


class HexMujocoArcherL6YClient(HexMujocoClientBase):

    def __init__(
        self,
        net_config: dict = NET_CONFIG,
        recv_config: dict = RECV_CONFIG,
    ):
        HexMujocoClientBase.__init__(self, net_config)
        self.__recv_config = recv_config
        self._side_camera_seq = {"rgb": 0, "depth": 0}
        self._side_used_camera_seq = {"rgb": 0, "depth": 0}
        self._side_camera_queue = {
            "rgb": deque(maxlen=self._deque_maxlen),
            "depth": deque(maxlen=self._deque_maxlen),
        }
        self._wait_for_working()

    def _get_side_frame_inner(self, depth_flag: bool = False):
        key = "depth" if depth_flag else "rgb"
        req_cmd = f"get_side_{'depth' if depth_flag else 'rgb'}"
        hdr, img = self.request({
            "cmd": req_cmd,
            "args": (1 + self._side_camera_seq[key]) % self._max_seq_num,
        })
        try:
            if hdr["cmd"] == f"{req_cmd}_ok":
                self._side_camera_seq[key] = hdr["args"]
                return hdr, img
            else:
                return None, None
        except Exception:
            return None, None

    def get_side_rgb(self, newest: bool = False):
        try:
            if self._realtime_mode or newest:
                hdr, img = self._side_camera_queue["rgb"][-1]
                if self._side_used_camera_seq["rgb"] != hdr["args"]:
                    self._side_used_camera_seq["rgb"] = hdr["args"]
                    return hdr, img
                else:
                    return None, None
            else:
                return self._side_camera_queue["rgb"].popleft()
        except IndexError:
            return None, None

    def get_side_depth(self, newest: bool = False):
        try:
            if self._realtime_mode or newest:
                hdr, img = self._side_camera_queue["depth"][-1]
                if self._side_used_camera_seq["depth"] != hdr["args"]:
                    self._side_used_camera_seq["depth"] = hdr["args"]
                    return hdr, img
                else:
                    return None, None
            else:
                return self._side_camera_queue["depth"].popleft()
        except IndexError:
            return None, None

    def get_side_intri(self):
        intri_hdr, intri = self.request({"cmd": "get_side_intri"})
        return intri_hdr, intri

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
                # side camera (always poll when image_trig fires)
                hdr, img = self._get_side_frame_inner(False)
                if hdr is not None:
                    self._side_camera_queue["rgb"].append((hdr, img))
                hdr, img = self._get_side_frame_inner(True)
                if hdr is not None:
                    self._side_camera_queue["depth"].append((hdr, img))

            try:
                cmds = self._cmds_queue[-1]
                _ = self._set_cmds_inner(cmds)
            except IndexError:
                pass

            rate.sleep()
