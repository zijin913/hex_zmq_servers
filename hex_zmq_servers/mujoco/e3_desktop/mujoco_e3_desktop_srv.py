#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-16
################################################################

import numpy as np
from collections import deque

try:
    from ..mujoco_base import HexMujocoServerBase
    from .mujoco_e3_desktop import HexMujocoE3Desktop
except (ImportError, ValueError):
    import sys
    from pathlib import Path
    this_file = Path(__file__).resolve()
    project_root = this_file.parents[3]
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))
    from hex_zmq_servers.mujoco.mujoco_base import HexMujocoServerBase
    from hex_zmq_servers.mujoco.e3_desktop.mujoco_e3_desktop import HexMujocoE3Desktop

NET_CONFIG = {
    "ip": "127.0.0.1",
    "port": 12345,
    "realtime_mode": False,
    "deque_maxlen": 10,
    "client_timeout_ms": 200,
    "server_timeout_ms": 1_000,
    "server_num_workers": 4,
}

MUJOCO_CONFIG = {
    "states_rate": 1000,
    "img_rate": 30,
    "headless": False,
    "sens_ts": True,
}


class HexMujocoE3DesktopServer(HexMujocoServerBase):

    def __init__(
        self,
        net_config: dict = NET_CONFIG,
        params_config: dict = MUJOCO_CONFIG,
    ):
        HexMujocoServerBase.__init__(self, net_config)

        # mujoco
        self._device = HexMujocoE3Desktop(
            params_config, net_config.get("realtime_mode", False))

        # values
        self._cmds_left_seq = -1
        self._cmds_right_seq = -1
        self._states_left_queue = deque(maxlen=self._deque_maxlen)
        self._states_right_queue = deque(maxlen=self._deque_maxlen)
        self._states_obj_queue = deque(maxlen=self._deque_maxlen)
        self._cmds_left_queue = deque(maxlen=self._deque_maxlen)
        self._cmds_right_queue = deque(maxlen=self._deque_maxlen)
        self._rgb_head_queue = deque(maxlen=self._deque_maxlen)
        self._depth_head_queue = deque(maxlen=self._deque_maxlen)
        self._rgb_left_queue = deque(maxlen=self._deque_maxlen)
        self._depth_left_queue = deque(maxlen=self._deque_maxlen)
        self._rgb_right_queue = deque(maxlen=self._deque_maxlen)
        self._depth_right_queue = deque(maxlen=self._deque_maxlen)

    def work_loop(self):
        try:
            self._device.work_loop([
                self._states_left_queue,
                self._states_right_queue,
                self._states_obj_queue,
                self._cmds_left_queue,
                self._cmds_right_queue,
                self._rgb_head_queue,
                self._depth_head_queue,
                self._rgb_left_queue,
                self._depth_left_queue,
                self._rgb_right_queue,
                self._depth_right_queue,
                self._stop_event,
            ])
        finally:
            self._device.close()

    def _get_states(self, recv_hdr: dict):
        try:
            seq = recv_hdr["args"]
        except KeyError:
            print(f"\033[91m{recv_hdr['cmd']} requires `args`\033[0m")
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

        # get robot name
        robot_name = recv_hdr["cmd"].split("_")[2]
        if robot_name == "left":
            queue = self._states_left_queue
        elif robot_name == "right":
            queue = self._states_right_queue
        elif robot_name == "obj":
            queue = self._states_obj_queue
        else:
            raise ValueError(f"unknown robot name: {robot_name}")

        try:
            ts, count, states = queue[
                -1] if self._realtime_mode else queue.popleft()
        except IndexError:
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None
        except Exception as e:
            print(f"\033[91m{recv_hdr['cmd']} failed: {e}\033[0m")
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

        delta = (count - seq) % self._max_seq_num
        if delta >= 0 and delta < 1e6:
            return {
                "cmd": f"{recv_hdr['cmd']}_ok",
                "ts": ts,
                "args": count
            }, states
        else:
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

    def _set_cmds(self, recv_hdr: dict, recv_buf: np.ndarray):
        seq = recv_hdr.get("args", None)
        if self._seq_clear_flag:
            self._seq_clear_flag = False
            self._cmds_left_seq = -1
            self._cmds_right_seq = -1
            return self.no_ts_hdr(recv_hdr, False), None

        # get robot name
        robot_name = recv_hdr["cmd"].split("_")[2]
        if robot_name == "left":
            queue = self._cmds_left_queue
            cmds_seq = self._cmds_left_seq
        elif robot_name == "right":
            queue = self._cmds_right_queue
            cmds_seq = self._cmds_right_seq
        else:
            raise ValueError(f"unknown robot name: {robot_name}")

        if seq is not None and seq > cmds_seq:
            delta = (seq - cmds_seq) % self._max_seq_num
            if delta >= 0 and delta < 1e6:
                if robot_name == "left":
                    self._cmds_left_seq = seq
                elif robot_name == "right":
                    self._cmds_right_seq = seq
                queue.append((recv_hdr["ts"], seq, recv_buf))
                return self.no_ts_hdr(recv_hdr, True), None
            else:
                return self.no_ts_hdr(recv_hdr, False), None
        else:
            return self.no_ts_hdr(recv_hdr, False), None

    def _get_frame(self, recv_hdr: dict):
        try:
            seq = recv_hdr["args"]
        except KeyError:
            print(f"\033[91m{recv_hdr['cmd']} requires `args`\033[0m")
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

        # get camera config
        split_cmd = recv_hdr["cmd"].split("_")
        depth_flag = split_cmd[1] == "depth"
        camera_name = split_cmd[2]
        if camera_name == "head":
            queue = self._rgb_head_queue if not depth_flag else self._depth_head_queue
        elif camera_name == "left":
            queue = self._rgb_left_queue if not depth_flag else self._depth_left_queue
        elif camera_name == "right":
            queue = self._rgb_right_queue if not depth_flag else self._depth_right_queue
        else:
            raise ValueError(f"unknown camera name: {camera_name}")

        try:
            ts, count, img = queue[
                -1] if self._realtime_mode else queue.popleft()
        except IndexError:
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None
        except Exception as e:
            print(f"\033[91m{recv_hdr['cmd']} failed: {e}\033[0m")
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

        delta = (count - seq) % self._max_seq_num
        if delta >= 0 and delta < 1e6:
            return {
                "cmd": f"{recv_hdr['cmd']}_ok",
                "ts": ts,
                "args": count
            }, img
        else:
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

    def _process_request(self, recv_hdr: dict, recv_buf: np.ndarray):
        if recv_hdr["cmd"] == "is_working":
            return self.no_ts_hdr(recv_hdr, self._device.is_working()), None
        elif recv_hdr["cmd"] == "seq_clear":
            return self.no_ts_hdr(recv_hdr, self._seq_clear()), None
        elif recv_hdr["cmd"] == "reset":
            return self.no_ts_hdr(recv_hdr, self._device.reset()), None
        elif recv_hdr["cmd"] == "get_dofs":
            dofs = self._device.get_dofs()
            return self.no_ts_hdr(recv_hdr, dofs is not None), dofs
        elif recv_hdr["cmd"] == "get_limits":
            limits = self._device.get_limits()
            return self.no_ts_hdr(recv_hdr, limits is not None), limits
        elif (recv_hdr["cmd"] == "set_cmds_left") or (recv_hdr["cmd"]
                                                      == "set_cmds_right"):
            return self._set_cmds(recv_hdr, recv_buf)
        elif (recv_hdr["cmd"] == "get_states_left") or (
                recv_hdr["cmd"]
                == "get_states_right") or (recv_hdr["cmd"]
                                           == "get_states_obj"):
            return self._get_states(recv_hdr)
        elif recv_hdr["cmd"] == "get_intri":
            intri = self._device.get_intri()
            return self.no_ts_hdr(recv_hdr, intri is not None), intri
        elif (recv_hdr["cmd"]
              == "get_rgb_head") or (recv_hdr["cmd"] == "get_depth_head") or (
                  recv_hdr["cmd"] == "get_rgb_left") or (
                      recv_hdr["cmd"] == "get_depth_left") or (
                          recv_hdr["cmd"]
                          == "get_rgb_right") or (recv_hdr["cmd"]
                                                  == "get_depth_right"):
            return self._get_frame(recv_hdr)
        else:
            raise ValueError(f"unknown command: {recv_hdr['cmd']}")


if __name__ == "__main__":
    import argparse, json
    from hex_zmq_servers.zmq_base import hex_server_helper

    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    hex_server_helper(cfg, HexMujocoE3DesktopServer)
