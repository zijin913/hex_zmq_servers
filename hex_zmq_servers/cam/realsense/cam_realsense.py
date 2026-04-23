#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-14
################################################################

import threading
import numpy as np
from collections import deque

from ..cam_base import HexCamBase
from ...zmq_base import (
    hex_ns_now,
    hex_zmq_ts_now,
    hex_zmq_ts_delta_ms,
)
from ...hex_launch import hex_log, HEX_LOG_LEVEL
import pyrealsense2 as rs

CAMERA_CONFIG = {
    "serial_number": None,  # None = use first available device
    "resolution": [640, 480],
    "depth_resolution": None,  # None = same as resolution
    "frame_rate": 30,
    "sens_ts": True,
    "enable_imu": False,   # D435i: gyro + accel multiplexed onto same pipeline
    "gyro_rate": 200,
    "accel_rate": 250,
    "imu_buffer_size": 2000,
}


class HexCamRealsense(HexCamBase):

    def __init__(
        self,
        camera_config: dict = CAMERA_CONFIG,
        realtime_mode: bool = False,
    ):
        HexCamBase.__init__(self, realtime_mode)

        try:
            self.__serial_number = camera_config.get("serial_number", None)
            self.__resolution = camera_config["resolution"]
            self.__depth_resolution = camera_config.get("depth_resolution", None) or self.__resolution
            self.__frame_rate = camera_config["frame_rate"]
            self.__sens_ts = camera_config.get("sens_ts", True)
            self.__enable_imu = bool(camera_config.get("enable_imu", False))
            self.__gyro_rate = int(camera_config.get("gyro_rate", 200))
            self.__accel_rate = int(camera_config.get("accel_rate", 250))
            self.__imu_buffer_size = int(camera_config.get("imu_buffer_size", 2000))
        except KeyError as ke:
            missing_key = ke.args[0]
            raise ValueError(
                f"camera_config is not valid, missing key: {missing_key}")

        # variables
        # realsense variables
        ctx = rs.context()
        available_devices = []
        for dev in ctx.query_devices():
            serial = dev.get_info(rs.camera_info.serial_number)
            name = dev.get_info(rs.camera_info.name)
            print(f"  - Device: {name}, Serial: {serial}")
            available_devices.append(serial)

        # If no serial specified, use first available device
        if not self.__serial_number and available_devices:
            self.__serial_number = available_devices[0]
            print(f"  Using first available device: {self.__serial_number}")
        elif self.__serial_number not in available_devices:
            print(
                f"can not find device with serial number: {self.__serial_number}"
            )
            return

        # camera variables
        self.__intri = np.zeros(4)

        # open device
        self.__pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(self.__serial_number)
        config.enable_stream(
            rs.stream.color,
            self.__resolution[0],
            self.__resolution[1],
            rs.format.bgr8,
            self.__frame_rate,
        )
        config.enable_stream(
            rs.stream.depth,
            self.__depth_resolution[0],
            self.__depth_resolution[1],
            rs.format.z16,
            self.__frame_rate,
        )

        # IMU activation flag (only relevant when enable_imu=True).
        self.__imu_active = False
        # Stash config + align + callback bookkeeping (used by IMU path only).
        self.__config = config
        self.__align = rs.align(rs.stream.color)
        self.__bias_ns = None
        self.__cb_lock = threading.Lock()
        self.__cb_rgb_count = 0
        self.__cb_depth_count = 0
        self.__cb_rgb_queue = None
        self.__cb_depth_queue = None
        self.__last_color = None
        self.__last_color_ts_us = 0.0
        self.__last_depth = None
        self.__last_depth_ts_us = 0.0
        self.__sync_window_ms = 50.0

        if self.__enable_imu:
            # IMU path: defer pipeline.start to work_loop (with callback).
            # Probe intrinsics by briefly starting a video-only pipeline.
            try:
                config.enable_stream(
                    rs.stream.gyro, rs.format.motion_xyz32f, self.__gyro_rate)
                config.enable_stream(
                    rs.stream.accel, rs.format.motion_xyz32f, self.__accel_rate)
                self.__imu_active = True
            except Exception as exc:
                hex_log(HEX_LOG_LEVEL["warn"],
                        f"HexCamRealsense IMU enable failed: {exc}")
            probe = rs.pipeline()
            probe_cfg = rs.config()
            probe_cfg.enable_device(self.__serial_number)
            probe_cfg.enable_stream(
                rs.stream.color, self.__resolution[0], self.__resolution[1],
                rs.format.bgr8, self.__frame_rate)
            probe_profile = probe.start(probe_cfg)
            color_profile = probe_profile.get_stream(rs.stream.color)
            color_intrinsics = color_profile.as_video_stream_profile(
            ).get_intrinsics()
            self.__intri[0] = color_intrinsics.fx
            self.__intri[1] = color_intrinsics.fy
            self.__intri[2] = color_intrinsics.ppx
            self.__intri[3] = color_intrinsics.ppy
            probe.stop()
        else:
            # Original (no-IMU) path: start the real pipeline here so the
            # device stays claimed and intrinsics come straight from it.
            # Behaviour is byte-equivalent to the pre-IMU version.
            profile = self.__pipeline.start(config)
            color_profile = profile.get_stream(rs.stream.color)
            color_intrinsics = color_profile.as_video_stream_profile(
            ).get_intrinsics()
            self.__intri[0] = color_intrinsics.fx
            self.__intri[1] = color_intrinsics.fy
            self.__intri[2] = color_intrinsics.ppx
            self.__intri[3] = color_intrinsics.ppy

        # IMU buffers
        self.__imu_lock = threading.Lock()
        self.__gyro_buffer = deque(maxlen=self.__imu_buffer_size)
        self.__accel_buffer = deque(maxlen=self.__imu_buffer_size)
        self.__imu_bias_ns = None

        if self.__imu_active:
            hex_log(HEX_LOG_LEVEL["info"],
                    f"HexCamRealsense IMU enabled in pipeline "
                    f"(gyro {self.__gyro_rate}Hz, accel {self.__accel_rate}Hz)")

        # start work loop
        self._working.set()

    def get_intri(self) -> np.ndarray:
        self._wait_for_working()
        return self.__intri

    def get_serial_number(self) -> np.ndarray:
        self._wait_for_working()
        return self.__serial_number

    def imu_enabled(self) -> bool:
        return self.__imu_active

    def drain_imu(self):
        """Pop and return all buffered IMU samples.

        Returns:
            gyro: (N, 4) ndarray of [t_ns, x, y, z] or empty.
            accel: (M, 4) ndarray of [t_ns, x, y, z] or empty.
        """
        with self.__imu_lock:
            gyro = np.asarray(list(self.__gyro_buffer), dtype=np.float64) \
                if self.__gyro_buffer else np.zeros((0, 4), dtype=np.float64)
            accel = np.asarray(list(self.__accel_buffer), dtype=np.float64) \
                if self.__accel_buffer else np.zeros((0, 4), dtype=np.float64)
            self.__gyro_buffer.clear()
            self.__accel_buffer.clear()
        return gyro, accel

    def peek_imu(self):
        """Return a snapshot of buffered IMU samples without clearing."""
        with self.__imu_lock:
            gyro = np.asarray(list(self.__gyro_buffer), dtype=np.float64) \
                if self.__gyro_buffer else np.zeros((0, 4), dtype=np.float64)
            accel = np.asarray(list(self.__accel_buffer), dtype=np.float64) \
                if self.__accel_buffer else np.zeros((0, 4), dtype=np.float64)
        return gyro, accel

    def __ingest_motion(self, motion_frame):
        """Pull a single motion frame into the gyro/accel buffer."""
        try:
            stream_type = motion_frame.get_profile().stream_type()
            sens_ts_ms = motion_frame.get_timestamp()
            if self.__imu_bias_ns is None:
                self.__imu_bias_ns = np.int64(hex_ns_now()) - np.int64(
                    sens_ts_ms * 1_000_000)
            ts_ns = self.__imu_bias_ns + np.int64(sens_ts_ms * 1_000_000)
            data = motion_frame.get_motion_data()
            sample = (float(ts_ns), float(data.x),
                      float(data.y), float(data.z))
            with self.__imu_lock:
                if stream_type == rs.stream.gyro:
                    self.__gyro_buffer.append(sample)
                elif stream_type == rs.stream.accel:
                    self.__accel_buffer.append(sample)
        except Exception:
            pass

    def __pipeline_callback(self, frame):
        """Pipeline-level callback. Each invocation receives ONE frame."""
        try:
            # Motion frame -> IMU buffer
            if frame.is_motion_frame():
                self.__ingest_motion(frame.as_motion_frame())
                return

            stream_type = frame.get_profile().stream_type()
            ts_us = frame.get_timestamp()  # ms (float)

            with self.__cb_lock:
                if stream_type == rs.stream.color:
                    self.__last_color = frame
                    self.__last_color_ts_us = ts_us
                elif stream_type == rs.stream.depth:
                    self.__last_depth = frame
                    self.__last_depth_ts_us = ts_us
                else:
                    return

                # Try to emit a paired (color, depth) snapshot
                if self.__last_color is None or self.__last_depth is None:
                    return
                dt_ms = abs(self.__last_color_ts_us - self.__last_depth_ts_us)
                if dt_ms > self.__sync_window_ms:
                    return

                color = self.__last_color
                depth = self.__last_depth
                pair_ts_us = self.__last_color_ts_us
                rgb_q = self.__cb_rgb_queue
                depth_q = self.__cb_depth_queue
                rgb_count = self.__cb_rgb_count
                depth_count = self.__cb_depth_count
                self.__cb_rgb_count = (self.__cb_rgb_count + 1) % self._max_seq_num
                self.__cb_depth_count = (self.__cb_depth_count + 1) % self._max_seq_num
                # Reset to avoid re-emitting the same pair
                self.__last_color = None
                self.__last_depth = None

            # Bias-correct the timestamp
            cur_ns = hex_zmq_ts_now()
            try:
                if self.__bias_ns is None:
                    self.__bias_ns = np.int64(hex_ns_now()) - np.int64(
                        pair_ts_us * 1_000_000)
                sen_ts_ns = self.__bias_ns + np.int64(pair_ts_us * 1_000_000)
                sen_ts = {
                    "s": int(sen_ts_ns // 1_000_000_000),
                    "ns": int(sen_ts_ns % 1_000_000_000),
                }
                if hex_zmq_ts_delta_ms(cur_ns, sen_ts) < 0:
                    sen_ts = cur_ns
            except Exception:
                sen_ts = cur_ns
            ts = sen_ts if self.__sens_ts else cur_ns

            color_arr = np.asanyarray(color.get_data()).copy()
            depth_arr = np.asanyarray(depth.get_data()).copy()
            if rgb_q is not None:
                rgb_q.append((ts, rgb_count, color_arr))
            if depth_q is not None:
                depth_q.append((ts, depth_count, depth_arr))
        except Exception as exc:
            hex_log(HEX_LOG_LEVEL["warn"],
                    f"HexCamRealsense callback error: {exc}")

    def work_loop(self, hex_queues: list[deque | threading.Event]):
        rgb_queue = hex_queues[0]
        depth_queue = hex_queues[1]
        stop_event = hex_queues[2]

        with self.__cb_lock:
            self.__cb_rgb_queue = rgb_queue
            self.__cb_depth_queue = depth_queue

        if self.__imu_active:
            # Callback-driven path (single pipeline with mixed video + IMU).
            # NOTE: This path is currently unreliable on D435i + librealsense
            # 2.x — video frames may not get delivered. Prefer enable_imu=false
            # and capture IMU separately if needed.
            try:
                self.__pipeline.start(self.__config, self.__pipeline_callback)
            except RuntimeError as exc:
                hex_log(HEX_LOG_LEVEL["warn"],
                        f"Pipeline start with IMU failed ({exc}); "
                        "retrying without IMU")
                self.__imu_active = False
                self.__config = self.__build_video_only_config()
                self.__pipeline.start(self.__config)

            while self._working.is_set() and not stop_event.is_set():
                stop_event.wait(0.1)
        else:
            # Polling path: classic wait_for_frames + align.
            # Pipeline was already started in __init__ (no-IMU branch),
            # so do NOT start it again here — that would throw.
            self.__run_video_loop(rgb_queue, depth_queue, stop_event)

        self.close()

    def __build_video_only_config(self):
        cfg = rs.config()
        cfg.enable_device(self.__serial_number)
        cfg.enable_stream(
            rs.stream.color, self.__resolution[0], self.__resolution[1],
            rs.format.bgr8, self.__frame_rate)
        cfg.enable_stream(
            rs.stream.depth, self.__depth_resolution[0],
            self.__depth_resolution[1], rs.format.z16, self.__frame_rate)
        return cfg

    def __run_video_loop(self, rgb_queue, depth_queue, stop_event):
        """Polling-mode loop: wait_for_frames + align, push to queues."""
        first = self.__pipeline.wait_for_frames()
        try:
            sens_us = first.get_frame_metadata(
                rs.frame_metadata_value.sensor_timestamp)
            self.__bias_ns = np.int64(hex_ns_now()) - np.int64(sens_us * 1_000)
        except Exception:
            self.__bias_ns = 0

        rgb_count = 0
        depth_count = 0
        while self._working.is_set() and not stop_event.is_set():
            try:
                aligned = self.__align.process(
                    self.__pipeline.wait_for_frames())
            except Exception as exc:
                hex_log(HEX_LOG_LEVEL["warn"],
                        f"wait_for_frames failed: {exc}")
                continue

            cur_ns = hex_zmq_ts_now()
            try:
                sens_us = aligned.get_frame_metadata(
                    rs.frame_metadata_value.sensor_timestamp)
                sen_ts_ns = self.__bias_ns + np.int64(sens_us * 1_000)
                sen_ts = {
                    "s": int(sen_ts_ns // 1_000_000_000),
                    "ns": int(sen_ts_ns % 1_000_000_000),
                }
                if hex_zmq_ts_delta_ms(cur_ns, sen_ts) < 0:
                    sen_ts = cur_ns
            except Exception:
                sen_ts = cur_ns
            ts = sen_ts if self.__sens_ts else cur_ns

            color = aligned.get_color_frame()
            if color:
                rgb_queue.append((ts, rgb_count,
                                   np.asanyarray(color.get_data()).copy()))
                rgb_count = (rgb_count + 1) % self._max_seq_num
            depth = aligned.get_depth_frame()
            if depth:
                depth_queue.append((ts, depth_count,
                                     np.asanyarray(depth.get_data()).copy()))
                depth_count = (depth_count + 1) % self._max_seq_num

    def close(self):
        if not self._working.is_set():
            return
        self._working.clear()
        try:
            self.__pipeline.stop()
        except Exception:
            pass
        hex_log(HEX_LOG_LEVEL["info"], "HexCamRealsense closed")
