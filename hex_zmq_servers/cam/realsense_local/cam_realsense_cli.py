#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# RealSense Camera Client
################################################################

import numpy as np

from ..cam_base import HexCamClientBase

NET_CONFIG = {
    "ip": "127.0.0.1",
    "port": 12346,
    "client_timeout_ms": 200,
    "server_timeout_ms": 1_000,
    "server_num_workers": 4,
}


class HexCamRealsenseClient(HexCamClientBase):
    """RealSense camera ZMQ client."""

    def __init__(self, net_config: dict = NET_CONFIG):
        HexCamClientBase.__init__(self, net_config)

    def get_rgb(self):
        """
        Get RGB image from camera.

        Returns:
            tuple: (header, image) where image is numpy array (H, W, 3) uint8,
                   or (None, None) on failure
        """
        return self._process_frame(depth_flag=False)

    def get_depth(self):
        """
        Get depth image from camera.

        Returns:
            tuple: (header, image) where image is numpy array (H, W) uint16 (mm),
                   or (None, None) on failure
        """
        return self._process_frame(depth_flag=True)

    def get_rgbd(self):
        """Fetch a synchronized (rgb, depth) pair produced from the same
        sensor tick. Returns (hdr, rgb, depth) or (None, None, None) on failure.

        Server packs rgb bytes followed by depth bytes into one buffer and
        puts the unpack metadata under hdr["args"]. The `ts` field in hdr is
        the RGB frame's sensor/push timestamp in hex_zmq ts dict format.
        """
        hdr, buf = self.request({"cmd": "get_rgbd"})
        try:
            if hdr is None:
                return None, None, None
            cmd = hdr.get("cmd", "")
            if cmd != "get_rgbd_ok":
                return None, None, None
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

    def get_intri(self):
        """
        Get camera intrinsic parameters.

        Returns:
            tuple: (header, intrinsics) where intrinsics is numpy array [fx, fy, cx, cy],
                   or (None, None) on failure
        """
        hdr, intri = self.request({"cmd": "get_intri"})

        try:
            cmd = hdr["cmd"]
            if cmd == "get_intri_ok":
                return hdr, intri
            else:
                print(f"\033[91mget_intri failed: {cmd}\033[0m")
                return None, None
        except KeyError:
            print(f"\033[91mget_intri response missing 'cmd'\033[0m")
            return None, None
        except Exception as e:
            print(f"\033[91mget_intri error: {e}\033[0m")
            return None, None
