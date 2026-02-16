#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-16
################################################################

import threading
import numpy as np
from collections import deque
from abc import abstractmethod

from ..device_base import HexDeviceBase
from ..zmq_base import HexZMQClientBase, HexZMQServerBase

from hex_robo_utils import (
    HexRate,
    hex_zmq_ts_now,
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

TAU = 2 * np.pi


class HexRobotBase(HexDeviceBase):

    def __init__(self, realtime_mode: bool = False):
        HexDeviceBase.__init__(self, realtime_mode)
        self._dofs = None
        self._limits = None
        self._seq_clear_flag = False

    def __del__(self):
        HexDeviceBase.__del__(self)

    def is_working(self) -> bool:
        return self._working.is_set()

    def get_dofs(self) -> np.ndarray:
        self._wait_for_working()
        return np.array(self._dofs, dtype=np.uint8)

    def get_limits(self) -> np.ndarray:
        self._wait_for_working()
        return self._limits

    @staticmethod
    def _rads_normalize(rads: np.ndarray) -> np.ndarray:
        return (rads + np.pi) % TAU - np.pi

    @staticmethod
    def _apply_pos_limits(
        rads: np.ndarray,
        lower_bound: np.ndarray,
        upper_bound: np.ndarray,
    ) -> np.ndarray:
        normed_rads = HexRobotBase._rads_normalize(rads)
        outside = (normed_rads < lower_bound) | (normed_rads > upper_bound)
        if not np.any(outside):
            return normed_rads

        lower_dist = np.fabs(
            HexRobotBase._rads_normalize((normed_rads - lower_bound)[outside]))
        upper_dist = np.fabs(
            HexRobotBase._rads_normalize((normed_rads - upper_bound)[outside]))
        choose_lower = lower_dist < upper_dist
        choose_upper = ~choose_lower

        outside_full = np.flatnonzero(outside)
        outside_lower = outside_full[choose_lower]
        outside_upper = outside_full[choose_upper]
        normed_rads[outside_lower] = lower_bound[outside_lower]
        normed_rads[outside_upper] = upper_bound[outside_upper]

        return normed_rads

    @abstractmethod
    def work_loop(self, hex_queues: list[deque | threading.Event]):
        raise NotImplementedError(
            "`work_loop` should be implemented by the child class")

    @abstractmethod
    def close(self):
        raise NotImplementedError(
            "`close` should be implemented by the child class")


class HexRobotClientBase(HexZMQClientBase):

    def __init__(self, net_config: dict = NET_CONFIG):
        HexZMQClientBase.__init__(self, net_config)
        self._states_seq = 0
        self._used_states_seq = 0
        self._cmds_seq = 0
        self._states_queue = deque(maxlen=self._deque_maxlen)
        self._cmds_queue = deque(maxlen=1)

    def __del__(self):
        HexZMQClientBase.__del__(self)

    def seq_clear(self):
        clear_hdr, _ = self.request({"cmd": "seq_clear"})
        return clear_hdr

    def get_dofs(self):
        _, dofs = self.request({"cmd": "get_dofs"})
        return dofs

    def get_limits(self):
        _, limits = self.request({"cmd": "get_limits"})
        return limits

    def get_states(self, newest: bool = False):
        try:
            if self._realtime_mode or newest:
                hdr, states = self._states_queue[-1]
                if self._used_states_seq != hdr["args"]:
                    self._used_states_seq = hdr["args"]
                    return hdr, states
                else:
                    return None, None
            else:
                return self._states_queue.popleft()
        except IndexError:
            return None, None

    def set_cmds(self, cmds: np.ndarray):
        self._cmds_queue.append(cmds)

    def _get_states_inner(self):
        hdr, states = self.request({
            "cmd":
            "get_states",
            "args": (1 + self._states_seq) % self._max_seq_num,
        })
        try:
            cmd = hdr["cmd"]
            if cmd == "get_states_ok":
                self._states_seq = hdr["args"]
                return hdr, states
            else:
                return None, None
        except KeyError:
            print(f"\033[91m{hdr['cmd']} requires `cmd`\033[0m")
            return None, None
        except Exception as e:
            print(f"\033[91mget_states failed: {e}\033[0m")
            return None, None

    def _set_cmds_inner(self, cmds: np.ndarray) -> bool:
        hdr, _ = self.request(
            {
                "cmd": "set_cmds",
                "ts": hex_zmq_ts_now(),
                "args": self._cmds_seq,
            },
            cmds,
        )
        # print(f"set_cmds seq: {self._cmds_seq}")
        try:
            cmd = hdr["cmd"]
            if cmd == "set_cmds_ok":
                self._cmds_seq = (self._cmds_seq + 1) % self._max_seq_num
                return True
            else:
                return False
        except KeyError:
            print(f"\033[91m{hdr['cmd']} requires `cmd`\033[0m")
            return False
        except Exception as e:
            print(f"\033[91mset_cmds failed: {e}\033[0m")
            return False

    def _recv_loop(self):
        rate = HexRate(2000)
        while self._recv_flag:
            hdr, states = self._get_states_inner()
            if hdr is not None:
                self._states_queue.append((hdr, states))

            try:
                cmds = self._cmds_queue[-1]
                _ = self._set_cmds_inner(cmds)
            except IndexError:
                pass

            rate.sleep()


class HexRobotServerBase(HexZMQServerBase):

    def __init__(self, net_config: dict = NET_CONFIG):
        HexZMQServerBase.__init__(self, net_config)
        self._device: HexDeviceBase = None
        self._states_queue = deque(maxlen=self._deque_maxlen)
        self._cmds_queue = deque(maxlen=1)
        self._cmds_seq = -1
        self._seq_clear_flag = False

    def __del__(self):
        HexZMQServerBase.__del__(self)
        self._device.close()

    def work_loop(self):
        try:
            self._device.work_loop([
                self._states_queue,
                self._cmds_queue,
                self._stop_event,
            ])
        finally:
            self._device.close()

    def _seq_clear(self):
        self._seq_clear_flag = True
        return True

    def _get_states(self, recv_hdr: dict):
        try:
            seq = recv_hdr["args"]
        except KeyError:
            print(f"\033[91m{recv_hdr['cmd']} requires `args`\033[0m")
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

        try:
            ts, count, states = self._states_queue[
                -1] if self._realtime_mode else self._states_queue.popleft()
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
            self._cmds_seq = -1
            return self.no_ts_hdr(recv_hdr, False), None

        if seq is not None:
            delta = (seq - self._cmds_seq) % self._max_seq_num
            if delta >= 0 and delta < 1e6:
                self._cmds_seq = seq
                self._cmds_queue.append((recv_hdr["ts"], seq, recv_buf))
                return self.no_ts_hdr(recv_hdr, True), None
            else:
                return self.no_ts_hdr(recv_hdr, False), None
        else:
            return self.no_ts_hdr(recv_hdr, False), None

    @abstractmethod
    def _process_request(self, recv_hdr: dict, recv_buf: np.ndarray):
        raise NotImplementedError(
            "`_process_request` should be implemented by the child class")
