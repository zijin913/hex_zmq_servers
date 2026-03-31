#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Dual-arm Archer L6Y MuJoCo server.
# Based on HexMujocoE3DesktopServer dual-arm pattern.
################################################################

import numpy as np
from collections import deque

try:
    from ..mujoco_base import HexMujocoServerBase
    from .mujoco_archer_l6y_dual import HexMujocoArcherL6YDual
except (ImportError, ValueError):
    import sys
    from pathlib import Path
    this_file = Path(__file__).resolve()
    project_root = this_file.parents[3]
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))
    from hex_zmq_servers.mujoco.mujoco_base import HexMujocoServerBase
    from hex_zmq_servers.mujoco.archer_l6y.mujoco_archer_l6y_dual import HexMujocoArcherL6YDual

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


class HexMujocoArcherL6YDualServer(HexMujocoServerBase):

    def __init__(
        self,
        net_config: dict = NET_CONFIG,
        params_config: dict = MUJOCO_CONFIG,
    ):
        HexMujocoServerBase.__init__(self, net_config)

        # mujoco
        self._device = HexMujocoArcherL6YDual(
            params_config, net_config.get("realtime_mode", False))

        # values
        self._cmds_left_seq = -1
        self._cmds_right_seq = -1
        self._states_left_queue = deque(maxlen=self._deque_maxlen)
        self._states_right_queue = deque(maxlen=self._deque_maxlen)
        self._states_obj_queue = deque(maxlen=self._deque_maxlen)
        self._cmds_left_queue = deque(maxlen=self._deque_maxlen)
        self._cmds_right_queue = deque(maxlen=self._deque_maxlen)
        self._rgb_left_queue = deque(maxlen=self._deque_maxlen)
        self._depth_left_queue = deque(maxlen=self._deque_maxlen)
        self._rgb_right_queue = deque(maxlen=self._deque_maxlen)
        self._depth_right_queue = deque(maxlen=self._deque_maxlen)
        self._side_rgb_queue = deque(maxlen=self._deque_maxlen)
        self._side_depth_queue = deque(maxlen=self._deque_maxlen)

    def work_loop(self):
        try:
            self._device.work_loop([
                self._states_left_queue,
                self._states_right_queue,
                self._states_obj_queue,
                self._cmds_left_queue,
                self._cmds_right_queue,
                self._rgb_left_queue,
                self._depth_left_queue,
                self._rgb_right_queue,
                self._depth_right_queue,
                self._side_rgb_queue,
                self._side_depth_queue,
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

        robot_name = recv_hdr["cmd"].split("_")[2]
        if robot_name == "left":
            queue = self._states_left_queue
        elif robot_name == "right":
            queue = self._states_right_queue
        elif robot_name == "obj":
            queue = self._states_obj_queue
        else:
            raise ValueError(
                f"unknown robot name: {robot_name} in {recv_hdr['cmd']}")

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

        split_cmd = recv_hdr["cmd"].split("_")
        depth_flag = split_cmd[1] == "depth"
        camera_name = split_cmd[2]
        if camera_name == "left":
            queue = self._depth_left_queue if depth_flag else self._rgb_left_queue
        elif camera_name == "right":
            queue = self._depth_right_queue if depth_flag else self._rgb_right_queue
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

    def _get_side_frame(self, recv_hdr: dict):
        try:
            seq = recv_hdr["args"]
        except KeyError:
            print(f"\033[91m{recv_hdr['cmd']} requires `args`\033[0m")
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

        depth_flag = "depth" in recv_hdr["cmd"]
        queue = self._side_depth_queue if depth_flag else self._side_rgb_queue
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
        cmd = recv_hdr["cmd"]
        if cmd == "is_working":
            return self.no_ts_hdr(recv_hdr,
                                  self._device.is_working()), None
        elif cmd == "seq_clear":
            return self.no_ts_hdr(recv_hdr, self._seq_clear()), None
        elif cmd == "reset":
            return self.no_ts_hdr(recv_hdr, self._device.reset()), None
        elif cmd == "get_dofs":
            dofs = self._device.get_dofs()
            return self.no_ts_hdr(recv_hdr, dofs is not None), dofs
        elif cmd == "get_limits":
            limits = self._device.get_limits()
            return self.no_ts_hdr(recv_hdr, limits is not None), limits
        elif cmd in ("set_cmds_left", "set_cmds_right"):
            return self._set_cmds(recv_hdr, recv_buf)
        elif cmd in ("get_states_left", "get_states_right", "get_states_obj"):
            return self._get_states(recv_hdr)
        elif cmd == "get_intri":
            intri = self._device.get_intri()
            return self.no_ts_hdr(recv_hdr, intri is not None), intri
        elif cmd in ("get_rgb_left", "get_depth_left", "get_rgb_right",
                     "get_depth_right"):
            return self._get_frame(recv_hdr)
        elif cmd == "get_side_intri":
            side_intri = self._device.get_side_intri()
            return self.no_ts_hdr(recv_hdr,
                                  side_intri is not None), side_intri
        elif cmd in ("get_side_rgb", "get_side_depth"):
            return self._get_side_frame(recv_hdr)
        else:
            raise ValueError(f"unknown command: {cmd}")


if __name__ == "__main__":
    import argparse, json
    from hex_zmq_servers.zmq_base import hex_server_helper

    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    hex_server_helper(cfg, HexMujocoArcherL6YDualServer)
