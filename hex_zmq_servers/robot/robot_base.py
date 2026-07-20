#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-16
################################################################

import threading
import time
import numpy as np
from collections import deque
from abc import abstractmethod

from ..device_base import HexDeviceBase
from ..zmq_base import HexZMQClientBase, HexZMQServerBase

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

    # _rads_normalize / _apply_pos_limits now live in HexDeviceBase — single shared
    # source of truth so the real (this) and sim devices clamp joint targets
    # IDENTICALLY. (The old version here wrapped to [-pi,pi) then snapped to the
    # nearest bound, which flipped a near-+pi target on joints like joint_3=[0,3.14]
    # to the opposite bound — a max-torque shove. The shared version wraps about the
    # range center, which is monotone and flip-free.)

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
        self._recv_loop_hz = net_config.get("recv_loop_hz", 2000)
        # Demand-driven recv cadence, mirroring the camera client
        # (cam/cam_base.py): poll the arm server at recv_loop_hz only while a
        # consumer is actually reading (get_states / set_cmds within
        # recv_idle_after seconds); otherwise drop to recv_idle_hz so an
        # UNUSED client stops hammering localhost ZMQ. Every iteration is a
        # blocking REQ/REP round-trip plus a msgpack decode and an
        # np.frombuffer/reshape, and the cost lands twice — here, and in the
        # server's worker threads. An idle client at the 2000 Hz default cost
        # ~26% CPU per arm on soda-can, plus the matching server-side load,
        # for data nobody read.
        #
        # UNLIKE the camera client this does NOT free-run at a low idle rate:
        # for arms the REQ path is the FALLBACK for control (callers may retry
        # only a few times, milliseconds apart), so a fixed 2 Hz idle tick like
        # the camera's would make the fallback unusable. Instead the idle sleep
        # waits on an event that any read/write sets, so demand resumes full
        # cadence on the NEXT iteration rather than after an idle period. The
        # idle rate is therefore only a keepalive ceiling, not a latency floor.
        self._recv_idle_hz = net_config.get("recv_idle_hz", 20.0)
        self._recv_idle_after = net_config.get("recv_idle_after", 1.0)
        self._last_read = 0.0
        self._demand_evt = threading.Event()
        self._last_sent_cmds_id = -1  # Track to avoid resending same command

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
        # Demand signal for _recv_loop's cadence. Stamped unconditionally —
        # including when the queue is empty — so a caller polling an idle
        # client wakes the loop on its first attempt rather than after it has
        # already given up.
        self._last_read = time.monotonic()
        self._demand_evt.set()
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
        # Commands are delivered by _recv_loop's second half, so a writer is
        # demand too — otherwise a client that only sends would sit at the idle
        # cadence and its commands would be paced by that instead.
        self._last_read = time.monotonic()
        self._demand_evt.set()
        self._cmds_queue.append(cmds)

    def set_control_mode(self, mode: str) -> bool:
        """One-shot control-mode switch (position | joint_impedance | torque).
        Not part of the streamed command path — a direct REQ/REP."""
        hdr, _ = self.request({"cmd": "set_control_mode", "args": mode})
        return isinstance(hdr, dict) and hdr.get("cmd") == "set_control_mode_ok"

    def clear_fault(self) -> bool:
        """One-shot fault clear (REQ/REP): clears a latched parking-stop + re-enters MIT."""
        hdr, _ = self.request({"cmd": "clear_fault"})
        return isinstance(hdr, dict) and hdr.get("cmd") == "clear_fault_ok"

    def get_fault(self):
        """Device fault buffer float64 [active, remotely_clearable] (or None)."""
        _, fault = self.request({"cmd": "get_fault"})
        return fault

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
                "ts": hex_ts_now(),
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
        rate = HexRate(self._recv_loop_hz)   # active cadence while in use
        idle_period = 1.0 / self._recv_idle_hz if self._recv_idle_hz > 0 else 0.0
        while self._recv_flag:
            hdr, states = self._get_states_inner()
            if hdr is not None:
                self._states_queue.append((hdr, states))

            try:
                cmds = self._cmds_queue[-1]
                _ = self._set_cmds_inner(cmds)
            except IndexError:
                pass

            # Demand-driven cadence: full rate only while a consumer read or
            # wrote recently; otherwise idle on the event so the very next
            # get_states/set_cmds resumes full cadence immediately.
            # recv_idle_hz <= 0 disables idling (legacy always-on behaviour).
            if (idle_period <= 0.0
                    or time.monotonic() - self._last_read < self._recv_idle_after):
                self._demand_evt.clear()
                rate.sleep()
            else:
                self._demand_evt.wait(idle_period)
                self._demand_evt.clear()


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
