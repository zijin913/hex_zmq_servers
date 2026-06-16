#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-12
################################################################

import numpy as np

try:
    from ..cam_base import HexCamServerBase
    from .cam_realsense import HexCamRealsense
except (ImportError, ValueError):
    import sys
    from pathlib import Path
    this_file = Path(__file__).resolve()
    project_root = this_file.parents[3]
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))
    from hex_zmq_servers.cam.cam_base import HexCamServerBase
    from hex_zmq_servers.cam.realsense.cam_realsense import HexCamRealsense

NET_CONFIG = {
    "ip": "127.0.0.1",
    "port": 12345,
    "realtime_mode": False,
    "deque_maxlen": 10,
    "client_timeout_ms": 200,
    "server_timeout_ms": 1_000,
    "server_num_workers": 4,
}

CAMERA_CONFIG = {
    "serial_number": '243422073194',
    "resolution": [640, 480],
    "frame_rate": 30,
    "sens_ts": True,
    "enable_imu": False,
}


class HexCamRealsenseServer(HexCamServerBase):

    def __init__(
        self,
        net_config: dict = NET_CONFIG,
        params_config: dict = CAMERA_CONFIG,
    ):
        HexCamServerBase.__init__(self, net_config)

        # camera
        self._device = HexCamRealsense(params_config,
                                       net_config.get("realtime_mode", False))

    def _process_request(self, recv_hdr: dict, recv_buf: np.ndarray):
        if recv_hdr["cmd"] == "is_working":
            return self.no_ts_hdr(recv_hdr, self._device.is_working()), None
        elif recv_hdr["cmd"] == "get_intri":
            intri = self._device.get_intri()
            return self.no_ts_hdr(recv_hdr, intri is not None), intri
        elif recv_hdr["cmd"] == "get_rgb":
            return self._get_frame(recv_hdr, False)
        elif recv_hdr["cmd"] == "get_depth":
            return self._get_frame(recv_hdr, True)
        elif recv_hdr["cmd"] == "get_rgbd":
            return self._get_rgbd(recv_hdr)
        elif recv_hdr["cmd"] == "get_imu":
            return self._get_imu(recv_hdr)
        elif recv_hdr["cmd"] == "get_rgbd_imu":
            return self._get_rgbd_imu(recv_hdr)
        else:
            raise ValueError(f"unknown command: {recv_hdr['cmd']}")

    def _get_imu(self, recv_hdr: dict):
        if not getattr(self._device, "imu_enabled", lambda: False)():
            return {"cmd": "get_imu_failed",
                    "args": "imu not enabled"}, None
        gyro, accel = self._device.drain_imu()
        gyro_bytes = gyro.astype(np.float64).tobytes()
        accel_bytes = accel.astype(np.float64).tobytes()
        combined = np.frombuffer(gyro_bytes + accel_bytes, dtype=np.uint8)
        return {
            "cmd": "get_imu_ok",
            "args": {
                "gyro_shape": list(gyro.shape),
                "gyro_dtype": str(gyro.dtype),
                "accel_shape": list(accel.shape),
                "accel_dtype": str(accel.dtype),
            }
        }, combined

    def _get_rgbd_imu(self, recv_hdr: dict):
        """Return the latest RGB+Depth plus any buffered IMU samples since last call."""
        try:
            if self._realtime_mode:
                rgb_ts, rgb_count, rgb = self._rgb_queue[-1]
                depth_ts, depth_count, depth = self._depth_queue[-1]
            else:
                rgb_ts, rgb_count, rgb = self._rgb_queue.popleft()
                depth_ts, depth_count, depth = self._depth_queue.popleft()
        except IndexError:
            return {"cmd": "get_rgbd_imu_failed"}, None
        except Exception as e:
            print(f"\033[91mget_rgbd_imu failed: {e}\033[0m")
            return {"cmd": "get_rgbd_imu_failed"}, None

        if getattr(self._device, "imu_enabled", lambda: False)():
            gyro, accel = self._device.drain_imu()
        else:
            gyro = np.zeros((0, 4), dtype=np.float64)
            accel = np.zeros((0, 4), dtype=np.float64)

        rgb_bytes = rgb.tobytes()
        depth_bytes = depth.tobytes()
        gyro_bytes = gyro.astype(np.float64).tobytes()
        accel_bytes = accel.astype(np.float64).tobytes()
        combined = np.frombuffer(
            rgb_bytes + depth_bytes + gyro_bytes + accel_bytes, dtype=np.uint8)

        return {
            "cmd": "get_rgbd_imu_ok",
            "ts": rgb_ts,
            "args": {
                "rgb_shape": list(rgb.shape),
                "rgb_dtype": str(rgb.dtype),
                "depth_shape": list(depth.shape),
                "depth_dtype": str(depth.dtype),
                "gyro_shape": list(gyro.shape),
                "gyro_dtype": str(gyro.dtype),
                "accel_shape": list(accel.shape),
                "accel_dtype": str(accel.dtype),
                "count": int(rgb_count),
            }
        }, combined


if __name__ == "__main__":
    import argparse, json
    from hex_zmq_servers.zmq_base import hex_server_helper

    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    hex_server_helper(cfg, HexCamRealsenseServer)
