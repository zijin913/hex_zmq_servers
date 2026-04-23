#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-12
################################################################

import numpy as np

from ..cam_base import HexCamClientBase

NET_CONFIG = {
    "ip": "127.0.0.1",
    "port": 12345,
    "realtime_mode": False,
    "deque_maxlen": 10,
    "client_timeout_ms": 200,
    "server_timeout_ms": 1_000,
    "server_num_workers": 4,
}


class HexCamRealsenseClient(HexCamClientBase):

    def __init__(
        self,
        net_config: dict = NET_CONFIG,
    ):
        HexCamClientBase.__init__(self, net_config)
        self._wait_for_working()

    def get_intri(self):
        intri_hdr, intri = self.request({"cmd": "get_intri"})
        return intri_hdr, intri

    def get_imu(self):
        """Fetch and drain buffered IMU samples from the server.

        Returns:
            (hdr, gyro (N,4), accel (M,4)) where rows are [t_ns, x, y, z].
            Returns (None, None, None) on failure / IMU not enabled.
        """
        hdr, buf = self.request({"cmd": "get_imu"})
        if hdr is None or hdr.get("cmd") != "get_imu_ok":
            return None, None, None
        args = hdr["args"]
        gyro_shape = tuple(args["gyro_shape"])
        gyro_dtype = np.dtype(args["gyro_dtype"])
        accel_shape = tuple(args["accel_shape"])
        accel_dtype = np.dtype(args["accel_dtype"])
        gyro_size = int(np.prod(gyro_shape) * gyro_dtype.itemsize) \
            if gyro_shape[0] > 0 else 0
        gyro = np.frombuffer(buf[:gyro_size], dtype=gyro_dtype).reshape(
            gyro_shape).copy() if gyro_size > 0 else np.zeros((0, 4))
        accel = np.frombuffer(buf[gyro_size:], dtype=accel_dtype).reshape(
            accel_shape).copy() if np.prod(accel_shape) > 0 else np.zeros((0, 4))
        return hdr, gyro, accel

    def get_rgbd_imu(self):
        """Fetch synchronized RGB + Depth and drain buffered IMU.

        Returns (hdr, rgb, depth, gyro, accel) on success, or Nones on failure.
        """
        hdr, buf = self.request({"cmd": "get_rgbd_imu"})
        if hdr is None or hdr.get("cmd") != "get_rgbd_imu_ok":
            return None, None, None, None, None
        args = hdr["args"]
        rgb_shape = tuple(args["rgb_shape"])
        rgb_dtype = np.dtype(args["rgb_dtype"])
        depth_shape = tuple(args["depth_shape"])
        depth_dtype = np.dtype(args["depth_dtype"])
        gyro_shape = tuple(args["gyro_shape"])
        gyro_dtype = np.dtype(args["gyro_dtype"])
        accel_shape = tuple(args["accel_shape"])
        accel_dtype = np.dtype(args["accel_dtype"])

        rgb_size = int(np.prod(rgb_shape) * rgb_dtype.itemsize)
        depth_size = int(np.prod(depth_shape) * depth_dtype.itemsize)
        gyro_size = int(np.prod(gyro_shape) * gyro_dtype.itemsize) \
            if gyro_shape[0] > 0 else 0

        off = 0
        rgb = np.frombuffer(buf[off:off + rgb_size], dtype=rgb_dtype).reshape(
            rgb_shape).copy()
        off += rgb_size
        depth = np.frombuffer(buf[off:off + depth_size],
                              dtype=depth_dtype).reshape(depth_shape).copy()
        off += depth_size
        if gyro_size > 0:
            gyro = np.frombuffer(buf[off:off + gyro_size],
                                 dtype=gyro_dtype).reshape(gyro_shape).copy()
        else:
            gyro = np.zeros((0, 4))
        off += gyro_size
        if np.prod(accel_shape) > 0:
            accel = np.frombuffer(buf[off:], dtype=accel_dtype).reshape(
                accel_shape).copy()
        else:
            accel = np.zeros((0, 4))
        return hdr, rgb, depth, gyro, accel
