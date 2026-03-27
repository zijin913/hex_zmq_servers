#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Dual-arm Archer L6Y MuJoCo client.
# Based on HexMujocoE3DesktopClient dual-arm pattern.
################################################################

import numpy as np
from collections import deque

from ..mujoco_base import HexMujocoClientBase
from ...zmq_base import HexRate, hex_zmq_ts_now

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
    "left_rgb": True,
    "left_depth": True,
    "right_rgb": True,
    "right_depth": True,
    "obj": True,
}


class HexMujocoArcherL6YDualClient(HexMujocoClientBase):

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
            "left_rgb": 0,
            "left_depth": 0,
            "right_rgb": 0,
            "right_depth": 0,
        }
        self._used_camera_seq = {
            "left_rgb": 0,
            "left_depth": 0,
            "right_rgb": 0,
            "right_depth": 0,
        }
        self._camera_queue = {
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
        # Side camera
        self._side_camera_seq = {"rgb": 0, "depth": 0}
        self._side_used_camera_seq = {"rgb": 0, "depth": 0}
        self._side_camera_queue = {
            "rgb": deque(maxlen=self._deque_maxlen),
            "depth": deque(maxlen=self._deque_maxlen),
        }
        self._wait_for_working()

    def reset(self):
        HexMujocoClientBase.reset(self)
        self._states_seq = {"left": 0, "right": 0, "obj": 0}
        self._used_states_queue = {"left": 0, "right": 0, "obj": 0}
        self._camera_seq = {
            "left_rgb": 0,
            "left_depth": 0,
            "right_rgb": 0,
            "right_depth": 0,
        }
        self._used_camera_seq = {
            "left_rgb": 0,
            "left_depth": 0,
            "right_rgb": 0,
            "right_depth": 0,
        }
        self._cmds_seq = {"left": 0, "right": 0}

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
        except (IndexError, KeyError):
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
        except (IndexError, KeyError):
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
            if hdr["cmd"] == f"{req_cmd}_ok":
                self._camera_seq[seq_key] = hdr["args"]
                return hdr, img
            else:
                return None, None
        except Exception:
            return None, None

    def _set_cmds_inner(
        self,
        cmds: np.ndarray,
        robot_name: str | None = None,
    ) -> bool:
        req_cmd = f"set_cmds_{robot_name}"
        hdr, _ = self.request(
            {
                "cmd": req_cmd,
                "ts": hex_zmq_ts_now(),
                "args": self._cmds_seq[robot_name],
            },
            cmds,
        )
        try:
            if hdr["cmd"] == f"{req_cmd}_ok":
                self._cmds_seq[robot_name] = (self._cmds_seq[robot_name] +
                                               1) % self._max_seq_num
                return True
            else:
                return False
        except Exception:
            return False

    def _get_side_frame_inner(self, depth_flag: bool = False):
        key = "depth" if depth_flag else "rgb"
        req_cmd = f"get_side_{'depth' if depth_flag else 'rgb'}"
        hdr, img = self.request({
            "cmd":
            req_cmd,
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
            # States for both arms
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
                # Side camera
                hdr, img = self._get_side_frame_inner(False)
                if hdr is not None:
                    self._side_camera_queue["rgb"].append((hdr, img))
                hdr, img = self._get_side_frame_inner(True)
                if hdr is not None:
                    self._side_camera_queue["depth"].append((hdr, img))

            # Commands for both arms
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
