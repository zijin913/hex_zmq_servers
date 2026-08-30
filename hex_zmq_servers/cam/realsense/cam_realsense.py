#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-14
################################################################

import fcntl
import os
import threading
import time
import numpy as np
from collections import deque
from contextlib import contextmanager

from ..cam_base import HexCamBase, CamFrame
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
    # Polling-path recovery.  A disconnected librealsense pipeline can turn
    # wait_for_frames() into an immediate exception; every retry below is
    # therefore paced and the whole in-process recovery has a hard deadline.
    "frame_timeout_ms": 5000,
    "timeout_failures_before_recovery": 3,
    "recovery_timeout_s": 180.0,
    "recovery_backoff_s": [1.0, 2.0, 5.0, 10.0, 30.0],
    "hardware_reset_after_failures": 2,
    "hardware_reset_cooldown_s": 60.0,
    "hardware_reset_settle_s": 4.0,
}


_DISCONNECTED_MARKERS = (
    "device disconnected",
    "no device connected",
    "cannot be called before start",
)
_FRAME_TIMEOUT_MARKERS = (
    "frame didn't arrive",
    "frame did not arrive",
)
_PERMANENT_CONFIG_MARKERS = (
    "couldn't resolve requests",
    "could not resolve requests",
    "invalid configuration",
    "invalid value",
)


class _RealSenseDeviceUnavailable(RuntimeError):
    """The configured serial is not present in a fresh rs.context()."""


def _classify_wait_for_frames_error(exc: Exception) -> str:
    """Return ``disconnected``, ``timeout``, or ``other`` for capture errors."""
    message = str(exc).lower()
    if any(marker in message for marker in _DISCONNECTED_MARKERS):
        return "disconnected"
    if any(marker in message for marker in _FRAME_TIMEOUT_MARKERS):
        return "timeout"
    return "other"


def _is_permanent_config_error(exc: Exception) -> bool:
    message = str(exc).lower()
    return any(marker in message for marker in _PERMANENT_CONFIG_MARKERS)


def _realsense_start_lock_path() -> str:
    """Choose a lock on tmpfs when possible, with a /tmp compatibility fallback."""
    candidates = [
        os.environ.get("XDG_RUNTIME_DIR"),
        f"/run/user/{os.getuid()}",
        "/tmp",
    ]
    for directory in candidates:
        if directory and os.path.isdir(directory) and os.access(directory, os.W_OK):
            return os.path.join(directory, "soda-realsense-start.lock")
    return "/tmp/soda-realsense-start.lock"


class HexCamRealsense(HexCamBase):

    def __init__(
        self,
        camera_config: dict = CAMERA_CONFIG,
        realtime_mode: bool = False,
    ):
        HexCamBase.__init__(self, realtime_mode)

        # Establish cleanup-safe lifecycle state before any SDK call can fail.
        self.__cb_lock = threading.Lock()
        self.__imu_lock = threading.Lock()
        self.__streaming = threading.Event()
        self.__pipeline = None
        self.__pipeline_started = False
        self.__closed_logged = False
        self.__state_pub = None
        # Injectable in tests so recovery timing assertions never sleep on a
        # wall clock.  Frame receipt stamps intentionally keep time.monotonic.
        self.__recovery_clock = time.monotonic

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
            self.__frame_timeout_ms = int(camera_config.get(
                "frame_timeout_ms", CAMERA_CONFIG["frame_timeout_ms"]))
            self.__timeout_failures_before_recovery = max(1, int(
                camera_config.get(
                    "timeout_failures_before_recovery",
                    CAMERA_CONFIG["timeout_failures_before_recovery"])))
            self.__recovery_timeout_s = float(camera_config.get(
                "recovery_timeout_s", CAMERA_CONFIG["recovery_timeout_s"]))
            self.__recovery_backoff_s = tuple(float(value) for value in
                camera_config.get(
                    "recovery_backoff_s",
                    CAMERA_CONFIG["recovery_backoff_s"]))
            self.__hardware_reset_after_failures = max(1, int(
                camera_config.get(
                    "hardware_reset_after_failures",
                    CAMERA_CONFIG["hardware_reset_after_failures"])))
            self.__hardware_reset_cooldown_s = float(camera_config.get(
                "hardware_reset_cooldown_s",
                CAMERA_CONFIG["hardware_reset_cooldown_s"]))
            self.__hardware_reset_settle_s = float(camera_config.get(
                "hardware_reset_settle_s",
                CAMERA_CONFIG["hardware_reset_settle_s"]))
        except KeyError as ke:
            missing_key = ke.args[0]
            raise ValueError(
                f"camera_config is not valid, missing key: {missing_key}")
        if not self.__recovery_backoff_s or any(
                value < 1.0 for value in self.__recovery_backoff_s):
            raise ValueError(
                "recovery_backoff_s must contain delays >= 1 second")
        if self.__recovery_timeout_s <= 0:
            raise ValueError("recovery_timeout_s must be > 0")
        if self.__frame_timeout_ms <= 0:
            raise ValueError("frame_timeout_ms must be > 0")
        if self.__hardware_reset_cooldown_s < 0:
            raise ValueError("hardware_reset_cooldown_s must be >= 0")
        if self.__hardware_reset_settle_s < 1.0:
            raise ValueError("hardware_reset_settle_s must be >= 1 second")

        # Optional high-rate image broadcast (zmq.PUB): cam/<topic_name>/image/compressed
        # (JPEG bgr8 + 16-byte [device_ts, host_ts] header) + throttled
        # cam/<topic_name>/camera_info (≈ sensor_msgs/CompressedImage + CameraInfo).
        # Enabled per camera via site.yaml cameras.<name>.pub_port. NON-BLOCKING:
        # a slow/absent subscriber drops frames, never stalls the capture path.
        self.__pub_topic = str(camera_config.get("topic_name", "cam"))
        self.__cam_info_tick = 0
        _pub_port = camera_config.get("pub_port")
        if _pub_port:
            try:
                from ...robot.state_pub import StatePublisher
                self.__state_pub = StatePublisher(int(_pub_port))
                print(f"[realsense] image PUB on :{int(_pub_port)} "
                      f"(topic cam/{self.__pub_topic}/image/compressed)")
            except Exception as e:
                print(f"[realsense] image PUB disabled: {e}")

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
            # 早退 raise(productization fix):原代码 return 但留下半初始化对象,
            # work_loop 用 self.__imu_active / __cb_lock 等未设 attr 崩 AttributeError。
            raise RuntimeError(
                f"RealSense not found: serial={self.__serial_number}, available={available_devices}"
            )

        # camera variables
        # [fx, fy, ppx, ppy, k1, k2, p1, p2, k3]. Coeffs (idx 4:9) carry the
        # RealSense color distortion (inverse_brown_conrady). Length grew from
        # 4 -> 9; all consumers slice [0:4], so this is backward compatible.
        self.__intri = np.zeros(9)

        # IMU activation flag (only relevant when enable_imu=True).
        self.__imu_active = False
        # Stash config + align + callback bookkeeping (used by IMU path only).
        config = self.__build_video_only_config()
        self.__config = config
        self.__align = rs.align(rs.stream.color)
        self.__bias_ns = None
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
            self.__pipeline = rs.pipeline(ctx)
            probe = rs.pipeline(ctx)
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
            self.__intri[4:9] = np.asarray(color_intrinsics.coeffs, dtype=float)[:5]
            probe.stop()
        else:
            # Initial start uses the same fresh-context, exact-serial and
            # first-frame acceptance path as runtime recovery.
            with self.__start_lock():
                self.__start_video_pipeline(ctx)

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

    @contextmanager
    def __start_lock(self, stop_event=None):
        """Serialize pipeline cold-starts across all RealSense server processes.

        Backoff and hardware-reset settle waits deliberately happen outside
        this lock.  Only SDK teardown/startup and the first-frame gate are
        serialized, matching the sequential cold-start contract in the host
        launcher without letting one absent camera block the others.
        """
        fd = os.open(_realsense_start_lock_path(),
                     os.O_CREAT | os.O_RDWR, 0o600)
        acquired = False
        try:
            while not acquired:
                if stop_event is not None and stop_event.is_set():
                    yield False
                    return
                try:
                    fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
                    acquired = True
                except BlockingIOError:
                    if stop_event is not None and stop_event.wait(0.1):
                        yield False
                        return
                    if stop_event is None:
                        time.sleep(0.1)
            yield True
        finally:
            if acquired:
                fcntl.flock(fd, fcntl.LOCK_UN)
            os.close(fd)

    def __find_configured_device(self, ctx):
        available = []
        selected = None
        for dev in ctx.query_devices():
            serial = dev.get_info(rs.camera_info.serial_number)
            available.append(serial)
            if serial == self.__serial_number:
                selected = dev
        if selected is None:
            raise _RealSenseDeviceUnavailable(
                f"RealSense not found: serial={self.__serial_number}, "
                f"available={available}")
        return selected

    def __set_frame_bias(self, frames):
        try:
            sens_us = frames.get_frame_metadata(
                rs.frame_metadata_value.sensor_timestamp)
            self.__bias_ns = np.int64(hex_ns_now()) - np.int64(sens_us * 1_000)
        except Exception:
            self.__bias_ns = 0

    def __update_intrinsics(self, profile):
        color_profile = profile.get_stream(rs.stream.color)
        color_intrinsics = color_profile.as_video_stream_profile(
        ).get_intrinsics()
        self.__intri[0] = color_intrinsics.fx
        self.__intri[1] = color_intrinsics.fy
        self.__intri[2] = color_intrinsics.ppx
        self.__intri[3] = color_intrinsics.ppy
        self.__intri[4:9] = np.asarray(
            color_intrinsics.coeffs, dtype=float)[:5]
        hex_log(HEX_LOG_LEVEL["info"],
                f"HexCamRealsense color intrinsics serial={self.__serial_number} "
                f"model={color_intrinsics.model} "
                f"fx={color_intrinsics.fx:.1f} fy={color_intrinsics.fy:.1f} "
                f"ppx={color_intrinsics.ppx:.1f} ppy={color_intrinsics.ppy:.1f} "
                f"coeffs={[round(c, 5) for c in color_intrinsics.coeffs]}")

    def __start_video_pipeline(self, ctx):
        """Build a fresh exact-serial pipeline and accept it only after a frame."""
        pipeline = rs.pipeline(ctx)
        config = self.__build_video_only_config()
        align = rs.align(rs.stream.color)
        started = False
        try:
            profile = pipeline.start(config)
            started = True
            self.__force_depth_units(pipeline)
            first = pipeline.wait_for_frames(
                timeout_ms=self.__frame_timeout_ms)
            aligned_first = align.process(first)
            self.__update_intrinsics(profile)
        except BaseException:
            if started:
                try:
                    pipeline.stop()
                except Exception:
                    pass
            raise

        self.__pipeline = pipeline
        self.__config = config
        self.__align = align
        self.__pipeline_started = True
        self.__set_frame_bias(aligned_first)
        self.__streaming.set()

    def __stop_pipeline(self):
        pipeline = self.__pipeline
        started = self.__pipeline_started
        self.__pipeline_started = False
        self.__pipeline = None
        if pipeline is not None and started:
            try:
                pipeline.stop()
            except Exception:
                pass

    def __force_depth_units(self, pipeline=None):
        """Force the depth sensor to 1mm units (0.001 m/unit).

        Downstream (soda camera_service) hardcodes raw-uint16 -> meters as
        ``/1000``, which assumes 1mm units. D435/D435i default to that, but
        D405 defaults to 0.1mm (0.0001 m/unit), so without this the cloud
        comes out 10x too far and gets pushed out of view / clipped.
        Setting it to 0.001 is a no-op on cameras already at 1mm.
        """
        try:
            active_pipeline = pipeline if pipeline is not None else self.__pipeline
            dev = active_pipeline.get_active_profile().get_device()
            ds = dev.first_depth_sensor()
            before = ds.get_option(rs.option.depth_units) \
                if ds.supports(rs.option.depth_units) else None
            scale_before = ds.get_depth_scale()
            if ds.supports(rs.option.depth_units):
                ds.set_option(rs.option.depth_units, 0.001)
            after = ds.get_option(rs.option.depth_units) \
                if ds.supports(rs.option.depth_units) else None
            scale_after = ds.get_depth_scale()
            hex_log(HEX_LOG_LEVEL["info"],
                    f"HexCamRealsense depth_units: option {before}->{after}, "
                    f"depth_scale {scale_before}->{scale_after} "
                    f"(target 0.001; downstream assumes 1mm)")
            if after is None or abs(after - 0.001) > 1e-6:
                hex_log(HEX_LOG_LEVEL["warn"],
                        "HexCamRealsense depth_units did NOT stick at 0.001 — "
                        "point cloud will be mis-scaled")
        except Exception as exc:
            hex_log(HEX_LOG_LEVEL["warn"],
                    f"HexCamRealsense set depth_units failed: {exc}")

    def get_intri(self) -> np.ndarray:
        self._wait_for_working()
        return self.__intri

    def get_serial_number(self) -> np.ndarray:
        self._wait_for_working()
        return self.__serial_number

    def is_working(self) -> bool:
        """Report capture health, not merely that the process is still alive."""
        return self._working.is_set() and self.__streaming.is_set()

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

    def __pub_jpg(self, ts, color_arr, host_ts=None):
        """Publish one frame on cam/<name>/image/compressed (+ throttled camera_info). Failures drop.

        host_ts is the frame-arrival RECEIPT stamp (time.monotonic, taken when the
        frame reached this process, before the JPEG encode). Published verbatim so
        the header's host_ts reflects arrival, not send -- same clock and same
        semantics as the arm's host_ts, so a subscriber can align camera against
        arm on host_ts directly. None falls back to send-time inside StatePublisher.
        """
        if self.__state_pub is None or color_arr is None:
            return
        if not self.__state_pub.jpeg_wanted(self.__pub_topic):
            return   # nobody subscribed to the image topic -> skip the expensive encode
        try:
            import cv2
            ok, buf = cv2.imencode(".jpg", color_arr)   # stream is bgr8 (rs.format.bgr8)
            if not ok:
                return
            dts = (float(ts.get("s", 0)) + float(ts.get("ns", 0)) * 1e-9
                   if isinstance(ts, dict) else float(ts))
            self.__state_pub.publish_jpeg(self.__pub_topic, dts, buf.tobytes(),
                                          host_ts=host_ts)
            n = self.__cam_info_tick
            self.__cam_info_tick = n + 1
            if n % 30 == 0:
                h, w = color_arr.shape[:2]
                self.__state_pub.publish_json(
                    f"cam/{self.__pub_topic}/camera_info", dts,
                    {"width": int(w), "height": int(h),
                     "fps": int(self.__frame_rate),
                     "format": "jpeg/bgr8", "frame": self.__pub_topic},
                    host_ts=host_ts)
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

            # RECEIPT stamp: this pair is now in-process. time.monotonic() to match
            # the arm's host_ts clock; the PUB path publishes it verbatim (no queue
            # on the camera PUB path, so this is essentially capture-to-publish).
            _recv_host_ts = time.monotonic()

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
            self.__streaming.set()
            # The SAME receipt stamp goes to both transports. The PUB path
            # already carried it; the REQ/REP queue used to drop it, which left
            # every soda_os-side consumer (teleop, policy, recording) unable to
            # say when a frame was captured.
            if rgb_q is not None:
                rgb_q.append(CamFrame(ts, rgb_count, color_arr, _recv_host_ts))
            self.__pub_jpg(ts, color_arr, host_ts=_recv_host_ts)
            if depth_q is not None:
                depth_q.append(CamFrame(ts, depth_count, depth_arr, _recv_host_ts))
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
                self.__pipeline_started = True
            except RuntimeError as exc:
                hex_log(HEX_LOG_LEVEL["warn"],
                        f"Pipeline start with IMU failed ({exc}); "
                        "retrying without IMU")
                self.__imu_active = False
                self.__pipeline = None
                with self.__start_lock(stop_event) as acquired:
                    if acquired:
                        ctx = rs.context()
                        self.__find_configured_device(ctx)
                        self.__start_video_pipeline(ctx)
                if acquired:
                    self.__run_video_loop(rgb_queue, depth_queue, stop_event)
            else:
                self.__force_depth_units()
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

    def __recover_video_pipeline(
        self,
        rgb_queue,
        depth_queue,
        stop_event,
        cause: Exception,
    ) -> bool:
        """Recover the polling pipeline, or exit non-zero for systemd fallback."""
        self.__streaming.clear()
        rgb_queue.clear()
        depth_queue.clear()
        recovery_started = self.__recovery_clock()
        last_hardware_reset = None
        present_device_failures = 0
        backoff_index = 0
        next_delay = self.__recovery_backoff_s[backoff_index]
        attempt = 0
        hex_log(HEX_LOG_LEVEL["warn"],
                f"RealSense stream lost serial={self.__serial_number}; "
                f"entering recovery: {cause}")

        while self._working.is_set() and not stop_event.is_set():
            elapsed = self.__recovery_clock() - recovery_started
            remaining = self.__recovery_timeout_s - elapsed
            if remaining <= 0:
                hex_log(HEX_LOG_LEVEL["err"],
                        f"RealSense recovery exhausted after {elapsed:.1f}s "
                        f"serial={self.__serial_number}; exiting for supervisor restart")
                self.close()
                raise SystemExit(3)

            # All waits happen outside the cross-process start lock.  A one
            # second minimum makes even an immediate SDK exception bounded.
            if stop_event.wait(min(next_delay, remaining)):
                return False
            if self.__recovery_clock() - recovery_started >= \
                    self.__recovery_timeout_s:
                continue
            attempt += 1
            reset_requested = False
            try:
                with self.__start_lock(stop_event) as acquired:
                    if not acquired:
                        return False
                    self.__stop_pipeline()
                    ctx = rs.context()
                    device = self.__find_configured_device(ctx)
                    try:
                        self.__start_video_pipeline(ctx)
                    except Exception as exc:
                        if _is_permanent_config_error(exc):
                            hex_log(HEX_LOG_LEVEL["err"],
                                    f"RealSense permanent configuration error "
                                    f"serial={self.__serial_number}: {exc}")
                            self.close()
                            raise SystemExit(3)
                        present_device_failures += 1
                        now = self.__recovery_clock()
                        reset_due = (
                            present_device_failures >=
                            self.__hardware_reset_after_failures
                            and (last_hardware_reset is None or
                                 now - last_hardware_reset >=
                                 self.__hardware_reset_cooldown_s)
                        )
                        if reset_due:
                            # Cool down reset attempts even if the SDK call
                            # itself fails; a broken USB stack must not create
                            # a second reset hot loop.
                            last_hardware_reset = now
                            present_device_failures = 0
                            try:
                                device.hardware_reset()
                                reset_requested = True
                                hex_log(HEX_LOG_LEVEL["warn"],
                                        f"RealSense hardware_reset issued "
                                        f"serial={self.__serial_number}")
                            except Exception as reset_exc:
                                hex_log(HEX_LOG_LEVEL["warn"],
                                        f"RealSense hardware_reset failed "
                                        f"serial={self.__serial_number}: {reset_exc}")
                        raise
            except _RealSenseDeviceUnavailable as exc:
                present_device_failures = 0
                hex_log(HEX_LOG_LEVEL["warn"],
                        f"RealSense recovery attempt={attempt} waiting for device: {exc}")
            except SystemExit:
                raise
            except Exception as exc:
                hex_log(HEX_LOG_LEVEL["warn"],
                        f"RealSense recovery attempt={attempt} failed "
                        f"serial={self.__serial_number}: {exc}")
            else:
                elapsed = self.__recovery_clock() - recovery_started
                hex_log(HEX_LOG_LEVEL["info"],
                        f"RealSense recovered serial={self.__serial_number} "
                        f"after {elapsed:.1f}s attempts={attempt}")
                return True

            if reset_requested:
                # The device handle becomes invalid immediately after reset;
                # release the lock first, then let USB re-enumerate.
                next_delay = self.__hardware_reset_settle_s
            else:
                backoff_index = min(
                    backoff_index + 1,
                    len(self.__recovery_backoff_s) - 1)
                next_delay = self.__recovery_backoff_s[backoff_index]

        return False

    def __run_video_loop(self, rgb_queue, depth_queue, stop_event):
        """Polling-mode loop: wait_for_frames + align, push to queues."""
        rgb_count = 0
        depth_count = 0
        timeout_failures = 0
        while self._working.is_set() and not stop_event.is_set():
            try:
                aligned = self.__align.process(
                    self.__pipeline.wait_for_frames(
                        timeout_ms=self.__frame_timeout_ms))
            except Exception as exc:
                failure_kind = _classify_wait_for_frames_error(exc)
                if failure_kind == "timeout":
                    timeout_failures += 1
                    if timeout_failures < self.__timeout_failures_before_recovery:
                        if timeout_failures == 1:
                            hex_log(HEX_LOG_LEVEL["warn"],
                                    f"RealSense frame timeout "
                                    f"serial={self.__serial_number}; "
                                    f"waiting for {self.__timeout_failures_before_recovery} "
                                    "consecutive failures before recovery")
                        continue
                timeout_failures = 0
                if not self.__recover_video_pipeline(
                        rgb_queue, depth_queue, stop_event, exc):
                    break
                continue

            timeout_failures = 0
            self.__streaming.set()

            # RECEIPT stamp: wait_for_frames just returned this frame. time.monotonic()
            # to match the arm's host_ts clock; published verbatim by the PUB path.
            _recv_host_ts = time.monotonic()
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
                color_arr = np.asanyarray(color.get_data()).copy()
                rgb_queue.append(CamFrame(ts, rgb_count, color_arr, _recv_host_ts))
                rgb_count = (rgb_count + 1) % self._max_seq_num
                self.__pub_jpg(ts, color_arr, host_ts=_recv_host_ts)
            depth = aligned.get_depth_frame()
            if depth:
                depth_queue.append(CamFrame(
                    ts, depth_count,
                    np.asanyarray(depth.get_data()).copy(), _recv_host_ts))
                depth_count = (depth_count + 1) % self._max_seq_num

    def close(self):
        self._working.clear()
        self.__streaming.clear()
        self.__stop_pipeline()
        state_pub = self.__state_pub
        self.__state_pub = None
        if state_pub is not None:
            try:
                state_pub.close()
            except Exception:
                pass
        if not self.__closed_logged:
            self.__closed_logged = True
            hex_log(HEX_LOG_LEVEL["info"], "HexCamRealsense closed")
