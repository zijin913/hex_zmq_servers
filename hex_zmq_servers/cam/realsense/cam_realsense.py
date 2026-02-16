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
from ...hex_launch import hex_log, HEX_LOG_LEVEL
import pyrealsense2 as rs

from hex_robo_utils import (
    hex_ns_now,
    hex_zmq_ts_delta_ms,
    hex_zmq_ts_now,
)

CAMERA_CONFIG = {
    "serial_number": '243422073194',
    "resolution": [640, 480],
    "frame_rate": 30,
    "sens_ts": True,
}


class HexCamRealsense(HexCamBase):

    def __init__(
        self,
        camera_config: dict = CAMERA_CONFIG,
        realtime_mode: bool = False,
    ):
        HexCamBase.__init__(self, realtime_mode)

        try:
            self.__serial_number = camera_config["serial_number"]
            self.__resolution = camera_config["resolution"]
            self.__frame_rate = camera_config["frame_rate"]
            self.__sens_ts = camera_config["sens_ts"]
        except KeyError as ke:
            missing_key = ke.args[0]
            raise ValueError(
                f"camera_config is not valid, missing key: {missing_key}")

        # variables
        # realsense variables
        ctx = rs.context()
        serial_numbers = []
        for dev in ctx.query_devices():
            serial = dev.get_info(rs.camera_info.serial_number)
            name = dev.get_info(rs.camera_info.name)
            print(f"  - Device: {name}, Serial: {serial}")
            serial_numbers.append(serial)
        if self.__serial_number not in serial_numbers:
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
            self.__resolution[0],
            self.__resolution[1],
            rs.format.z16,
            self.__frame_rate,
        )
        profile = self.__pipeline.start(config)
        color_profile = profile.get_stream(rs.stream.color)
        color_intrinsics = color_profile.as_video_stream_profile(
        ).get_intrinsics()
        self.__intri[0] = color_intrinsics.fx
        self.__intri[1] = color_intrinsics.fy
        self.__intri[2] = color_intrinsics.ppx
        self.__intri[3] = color_intrinsics.ppy
        self.__align = rs.align(rs.stream.color)
        self.__bias_ns = None

        # start work loop
        self._working.set()

    def get_intri(self) -> np.ndarray:
        self._wait_for_working()
        return self.__intri

    def get_serial_number(self) -> np.ndarray:
        self._wait_for_working()
        return self.__serial_number

    def work_loop(self, hex_queues: list[deque | threading.Event]):
        rgb_queue = hex_queues[0]
        depth_queue = hex_queues[1]
        stop_event = hex_queues[2]

        frames = self.__pipeline.wait_for_frames()
        bias_ns = np.int64(hex_ns_now()) - np.int64(
            frames.get_frame_metadata(rs.frame_metadata_value.sensor_timestamp)
            * 1_000)

        rgb_count = 0
        depth_count = 0
        while self._working.is_set() and not stop_event.is_set():
            # read frame
            aligned_frames = self.__align.process(
                self.__pipeline.wait_for_frames())
            cur_ns = hex_zmq_ts_now()
            sen_ts_ns = bias_ns + np.int64(
                aligned_frames.get_frame_metadata(
                    rs.frame_metadata_value.sensor_timestamp) * 1_000)
            sen_ts = {
                "s": sen_ts_ns // 1_000_000_000,
                "ns": sen_ts_ns % 1_000_000_000,
            }
            if hex_zmq_ts_delta_ms(cur_ns, sen_ts) < 0:
                sen_ts = cur_ns

            # collect rgb frame
            color_frame = aligned_frames.get_color_frame()
            if color_frame:

                rgb_queue.append(
                    (sen_ts if self.__sens_ts else cur_ns, rgb_count,
                     np.asanyarray(color_frame.get_data())))
                rgb_count = (rgb_count + 1) % self._max_seq_num

            # collect depth frame
            depth_frame = aligned_frames.get_depth_frame()
            if depth_frame:
                depth_queue.append(
                    (sen_ts if self.__sens_ts else cur_ns, depth_count,
                     np.asanyarray(depth_frame.get_data())))
                depth_count = (depth_count + 1) % self._max_seq_num

        # close
        self.close()

    def close(self):
        if not self._working.is_set():
            return
        self._working.clear()
        self.__pipeline.stop()
        hex_log(HEX_LOG_LEVEL["info"], "HexCamRealsense closed")
