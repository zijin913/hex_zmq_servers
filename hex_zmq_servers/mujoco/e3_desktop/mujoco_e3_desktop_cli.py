#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-12
################################################################

import numpy as np
from collections import deque
from ..mujoco_base import HexMujocoClientBase

from hex_robo_utils import (
    HexRate,
    hex_ts_now,
)

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
    "head_rgb": True,
    "head_depth": True,
    "left_rgb": True,
    "left_depth": True,
    "right_rgb": True,
    "right_depth": True,
    "obj": True,
}


class HexMujocoE3DesktopClient(HexMujocoClientBase):

    def __init__(
        self,
        net_config: dict = NET_CONFIG,
        recv_config: dict = RECV_CONFIG,
    ):
        HexMujocoClientBase.__init__(self, net_config)
        self.__recv_config = recv_config
        self._states_seq = {
            "left": 0,
            "right": 0,
            "obj": 0,
        }
        self._used_states_queue = {
            "left": 0,
            "right": 0,
            "obj": 0,
        }
        self._states_queue = {
            "left": deque(maxlen=self._deque_maxlen),
            "right": deque(maxlen=self._deque_maxlen),
            "obj": deque(maxlen=self._deque_maxlen),
        }
        self._camera_seq = {
            "head_rgb": 0,
            "head_depth": 0,
            "left_rgb": 0,
            "left_depth": 0,
            "right_rgb": 0,
            "right_depth": 0,
        }
        self._used_camera_seq = {
            "head_rgb": 0,
            "head_depth": 0,
            "left_rgb": 0,
            "left_depth": 0,
            "right_rgb": 0,
            "right_depth": 0,
        }
        self._camera_queue = {
            "head_rgb": deque(maxlen=self._deque_maxlen),
            "head_depth": deque(maxlen=self._deque_maxlen),
            "left_rgb": deque(maxlen=self._deque_maxlen),
            "left_depth": deque(maxlen=self._deque_maxlen),
            "right_rgb": deque(maxlen=self._deque_maxlen),
            "right_depth": deque(maxlen=self._deque_maxlen),
        }
        self._cmds_seq = {
            "left": 0,
            "right": 0,
        }
        self._cmds_queue = {
            "left": deque(maxlen=1),
            "right": deque(maxlen=1),
        }
        self._wait_for_working()

    def reset(self):
        HexMujocoClientBase.reset(self)
        self._states_seq = {
            "left": 0,
            "right": 0,
            "obj": 0,
        }
        self._used_states_queue = {
            "left": 0,
            "right": 0,
            "obj": 0,
        }
        self._camera_seq = {
            "head_rgb": 0,
            "head_depth": 0,
            "left_rgb": 0,
            "left_depth": 0,
            "right_rgb": 0,
            "right_depth": 0,
        }
        self._used_camera_seq = {
            "head_rgb": 0,
            "head_depth": 0,
            "left_rgb": 0,
            "left_depth": 0,
            "right_rgb": 0,
            "right_depth": 0,
        }
        self._cmds_seq = {
            "left": 0,
            "right": 0,
        }

    def get_rgb(self, camera_name: str | None = None, newest: bool = False):
        name = f"{camera_name}_rgb"
        try:
            if self._realtime_mode or newest:
                hdr, img = self._camera_queue[name][-1]
                if self._used_camera_seq[name] != hdr["args"]:
                    self._used_camera_seq[name] = hdr["args"]
                    return hdr, img
                else:
                    return None, None
            else:
                return self._camera_queue[name].popleft()
        except IndexError:
            return None, None
        except KeyError:
            print(f"\033[91munknown camera name: {name}\033[0m")
            return None, None

    def get_depth(self, camera_name: str | None = None, newest: bool = False):
        name = f"{camera_name}_depth"
        try:
            if self._realtime_mode or newest:
                hdr, img = self._camera_queue[name][-1]
                if self._used_camera_seq[name] != hdr["args"]:
                    self._used_camera_seq[name] = hdr["args"]
                    return hdr, img
                else:
                    return None, None
            else:
                return self._camera_queue[name].popleft()
        except IndexError:
            return None, None
        except KeyError:
            print(f"\033[91munknown camera name: {name}\033[0m")
            return None, None

    def set_cmds(
        self,
        cmds: np.ndarray,
        robot_name: str | None = None,
    ):
        self._cmds_queue[robot_name].append(cmds)

    def _process_frame(
        self,
        camera_name: str | None = None,
        depth_flag: bool = False,
    ):
        if camera_name is None:
            raise ValueError("camera_name is required")

        req_cmd = f"get_{'depth' if depth_flag else 'rgb'}_{camera_name}"
        seq_key = f"{camera_name}_{'depth' if depth_flag else 'rgb'}"

        hdr, img = self.request({
            "cmd":
            req_cmd,
            "args": (1 + self._camera_seq[seq_key]) % self._max_seq_num,
        })

        try:
            cmd = hdr["cmd"]
            if cmd == f"{req_cmd}_ok":
                self._camera_seq[seq_key] = hdr["args"]
                return hdr, img
            else:
                return None, None
        except KeyError:
            print(f"\033[91m{hdr['cmd']} requires `cmd`\033[0m")
            return None, None
        except Exception as e:
            print(f"\033[91m__process_frame failed: {e}\033[0m")
            return None, None

    def _set_cmds_inner(
        self,
        cmds: np.ndarray,
        robot_name: str | None = None,
    ) -> bool:
        req_cmd = "set_cmds"
        if robot_name is not None:
            req_cmd += f"_{robot_name}"
        hdr, _ = self.request(
            {
                "cmd": req_cmd,
                "ts": hex_ts_now(),
                "args": self._cmds_seq[robot_name],
            },
            cmds,
        )
        # print(f"{req_cmd} seq: {self._cmds_seq[robot_name]}")
        try:
            cmd = hdr["cmd"]
            if cmd == f"{req_cmd}_ok":
                self._cmds_seq[robot_name] = (self._cmds_seq[robot_name] +
                                              1) % self._max_seq_num
                return True
            else:
                return False
        except KeyError:
            print(f"\033[91m{hdr['cmd']} requires `cmd`\033[0m")
            return False
        except Exception as e:
            print(f"\033[91m{req_cmd} failed: {e}\033[0m")
            return False

    def _recv_loop(self):
        rate = HexRate(2000)
        image_trig_cnt = 0
        while self._recv_flag:
            hdr, states = self._get_states_inner("left")
            if hdr is not None:
                self._states_queue["left"].append((hdr, states))
            hdr, states = self._get_states_inner("right")
            if hdr is not None:
                self._states_queue["right"].append((hdr, states))
            if self.__recv_config["obj"]:
                hdr, obj_pose = self._get_states_inner("obj")
                if hdr is not None:
                    self._states_queue["obj"].append((hdr, obj_pose))

            image_trig_cnt += 1
            if image_trig_cnt >= 10:
                image_trig_cnt = 0
                if self.__recv_config["head_rgb"]:
                    hdr, img = self._get_rgb_inner("head")
                    if hdr is not None:
                        self._camera_queue["head_rgb"].append((hdr, img))
                if self.__recv_config["head_depth"]:
                    hdr, img = self._get_depth_inner("head")
                    if hdr is not None:
                        self._camera_queue["head_depth"].append((hdr, img))
                if self.__recv_config["left_rgb"]:
                    hdr, img = self._get_rgb_inner("left")
                    if hdr is not None:
                        self._camera_queue["left_rgb"].append((hdr, img))
                if self.__recv_config["left_depth"]:
                    hdr, img = self._get_depth_inner("left")
                    if hdr is not None:
                        self._camera_queue["left_depth"].append((hdr, img))
                if self.__recv_config["right_rgb"]:
                    hdr, img = self._get_rgb_inner("right")
                    if hdr is not None:
                        self._camera_queue["right_rgb"].append((hdr, img))
                if self.__recv_config["right_depth"]:
                    hdr, img = self._get_depth_inner("right")
                    if hdr is not None:
                        self._camera_queue["right_depth"].append((hdr, img))

            try:
                cmds = self._cmds_queue["left"][-1]
                _ = self._set_cmds_inner(cmds, "left")
            except IndexError:
                pass
            try:
                cmds = self._cmds_queue["right"][-1]
                _ = self._set_cmds_inner(cmds, "right")
            except IndexError:
                pass

            rate.sleep()
