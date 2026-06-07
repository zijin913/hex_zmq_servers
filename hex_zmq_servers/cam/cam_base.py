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


class HexCamBase(HexDeviceBase):

    def __init__(self, realtime_mode: bool = False):
        HexDeviceBase.__init__(self, realtime_mode)

    def __del__(self):
        HexDeviceBase.__del__(self)

    @abstractmethod
    def work_loop(self, hex_queues: list[deque | threading.Event]):
        raise NotImplementedError(
            "`work_loop` should be implemented by the child class")

    @abstractmethod
    def close(self):
        raise NotImplementedError(
            "`close` should be implemented by the child class")


class HexCamClientBase(HexZMQClientBase):

    def __init__(self, net_config: dict = NET_CONFIG):
        HexZMQClientBase.__init__(self, net_config)
        self._rgb_seq = 0
        self._used_rgb_seq = 0
        self._depth_seq = 0
        self._used_depth_seq = 0
        self._rgbd_seq = 0
        self._used_rgbd_seq = 0
        self._rgb_queue = deque(maxlen=self._deque_maxlen)
        self._depth_queue = deque(maxlen=self._deque_maxlen)
        self._rgbd_queue = deque(maxlen=self._deque_maxlen)

    def __del__(self):
        HexZMQClientBase.__del__(self)

    def get_rgb(self, newest: bool = False):
        try:
            if self._realtime_mode or newest:
                hdr, img = self._rgb_queue[-1]
                if self._used_rgb_seq != hdr["args"]:
                    self._used_rgb_seq = hdr["args"]
                    return hdr, img
                else:
                    return None, None
            else:
                return self._rgb_queue.popleft()
        except IndexError:
            return None, None

    def get_depth(self, newest: bool = False):
        try:
            if self._realtime_mode or newest:
                hdr, img = self._depth_queue[-1]
                if self._used_depth_seq != hdr["args"]:
                    self._used_depth_seq = hdr["args"]
                    return hdr, img
                else:
                    return None, None
            else:
                return self._depth_queue.popleft()
        except IndexError:
            return None, None

    def get_rgbd(self, newest: bool = False):
        """Get both RGB and depth frames together (synchronized)."""
        try:
            if self._realtime_mode or newest:
                hdr, rgb, depth = self._rgbd_queue[-1]
                if self._used_rgbd_seq != hdr.get("args", 0):
                    self._used_rgbd_seq = hdr.get("args", 0)
                    return hdr, rgb, depth
                else:
                    return None, None, None
            else:
                return self._rgbd_queue.popleft()
        except IndexError:
            return None, None, None

    def _get_rgbd_inner(self):
        """Internal method to fetch RGBD from server (called by _recv_loop)."""
        hdr, buf = self.request({"cmd": "get_rgbd"})

        try:
            if hdr is None:
                return None, None, None
            cmd = hdr.get("cmd", "")
            if cmd != "get_rgbd_ok":
                # Silently fail for expected failures
                if cmd == "get_rgbd_failed":
                    return None, None, None
                # Log unexpected responses
                print(f"\033[93mget_rgbd unexpected: {cmd}\033[0m")
                return None, None, None

            # Unpack RGB and depth from combined buffer
            args = hdr["args"]
            rgb_shape = tuple(args["rgb_shape"])
            rgb_dtype = np.dtype(args["rgb_dtype"])
            depth_shape = tuple(args["depth_shape"])
            depth_dtype = np.dtype(args["depth_dtype"])

            rgb_size = int(np.prod(rgb_shape) * rgb_dtype.itemsize)

            rgb = np.frombuffer(buf[:rgb_size], dtype=rgb_dtype).reshape(rgb_shape).copy()
            depth = np.frombuffer(buf[rgb_size:], dtype=depth_dtype).reshape(depth_shape).copy()

            return hdr, rgb, depth
        except Exception as e:
            print(f"\033[91mget_rgbd unpack failed: {e}\033[0m")
            return None, None, None

    def _get_rgb_inner(self):
        return self._process_frame(False)

    def _get_depth_inner(self):
        return self._process_frame(True)

    def _process_frame(self, depth_flag: bool):
        req_cmd = f"get_{'depth' if depth_flag else 'rgb'}"
        hdr, img = self.request({
            "cmd":
            req_cmd,
            "args": (1 + (self._depth_seq if depth_flag else self._rgb_seq)) %
            self._max_seq_num,
        })

        try:
            cmd = hdr["cmd"]
            if cmd == f"{req_cmd}_ok":
                if depth_flag:
                    self._depth_seq = hdr["args"]
                else:
                    self._rgb_seq = hdr["args"]
                return hdr, img
            else:
                return None, None
        except KeyError:
            print(f"\033[91m{hdr['cmd']} requires `cmd`\033[0m")
            return None, None
        except Exception as e:
            print(f"\033[91m__process_frame failed: {e}\033[0m")
            return None, None

    def _recv_loop(self):
        rate = HexRate(200)
        while self._recv_flag:
            # Use get_rgbd for synchronized frames (1 request instead of 2)
            hdr, rgb, depth = self._get_rgbd_inner()
            if hdr is not None:
                self._rgbd_queue.append((hdr, rgb, depth))
                # Also populate separate queues for backwards compatibility
                # Create separate headers with correct cmd for legacy code
                rgb_hdr = {**hdr, "cmd": "get_rgb_ok"}
                depth_hdr = {**hdr, "cmd": "get_depth_ok"}
                self._rgb_queue.append((rgb_hdr, rgb))
                self._depth_queue.append((depth_hdr, depth))
            rate.sleep()


class HexCamServerBase(HexZMQServerBase):

    def __init__(self, net_config: dict = NET_CONFIG):
        HexZMQServerBase.__init__(self, net_config)
        self._device: HexDeviceBase = None
        self._rgb_queue = deque(maxlen=self._deque_maxlen)
        self._depth_queue = deque(maxlen=self._deque_maxlen)

    def __del__(self):
        HexZMQServerBase.__del__(self)
        self._device.close()

    def work_loop(self):
        try:
            self._device.work_loop([
                self._rgb_queue,
                self._depth_queue,
                self._stop_event,
            ])
        finally:
            self._device.close()

    def _get_frame(self, recv_hdr: dict, depth_flag: bool):
        try:
            seq = recv_hdr["args"]
        except KeyError:
            print(f"\033[91m{recv_hdr['cmd']} requires `args`\033[0m")
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

        try:
            if depth_flag:
                ts, count, img = self._depth_queue[
                    -1] if self._realtime_mode else self._depth_queue.popleft(
                    )
            else:
                ts, count, img = self._rgb_queue[
                    -1] if self._realtime_mode else self._rgb_queue.popleft()
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

    def _get_rgbd(self, recv_hdr: dict):
        """Get both RGB and depth frames together."""
        try:
            # Get latest from both queues (they are pushed together by device)
            rgb_ts, rgb_count, rgb = self._rgb_queue[-1] if self._realtime_mode else self._rgb_queue.popleft()
            depth_ts, depth_count, depth = self._depth_queue[-1] if self._realtime_mode else self._depth_queue.popleft()
        except IndexError:
            return {"cmd": "get_rgbd_failed"}, None
        except Exception as e:
            print(f"\033[91mget_rgbd failed: {e}\033[0m")
            return {"cmd": "get_rgbd_failed"}, None

        # Pack RGB and depth into single buffer
        rgb_bytes = rgb.tobytes()
        depth_bytes = depth.tobytes()
        combined = np.frombuffer(rgb_bytes + depth_bytes, dtype=np.uint8)

        return {
            "cmd": "get_rgbd_ok",
            "ts": rgb_ts,
            "args": {
                "rgb_shape": list(rgb.shape),
                "rgb_dtype": str(rgb.dtype),
                "depth_shape": list(depth.shape),
                "depth_dtype": str(depth.dtype),
            }
        }, combined

    @abstractmethod
    def _process_request(self, recv_hdr: dict, recv_buf: np.ndarray):
        raise NotImplementedError(
            "`_process_request` should be implemented by the child class")
