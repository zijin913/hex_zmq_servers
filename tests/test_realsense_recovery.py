"""Hardware-free contracts for the RealSense polling recovery state machine."""

from __future__ import annotations

from collections import deque
from contextlib import nullcontext
import threading

from hex_zmq_servers.cam.realsense import cam_realsense as camera_module
from hex_zmq_servers.cam.realsense.cam_realsense import (
    HexCamRealsense,
    _RealSenseDeviceUnavailable,
    _classify_wait_for_frames_error,
)


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def monotonic(self):
        return self.now


class FakeStopEvent:
    def __init__(self, clock: FakeClock):
        self.clock = clock
        self.waits = []
        self.stopped = False

    def is_set(self):
        return self.stopped

    def wait(self, delay):
        self.waits.append(delay)
        self.clock.now += delay
        return self.stopped


class FakeDevice:
    def __init__(self):
        self.hardware_resets = 0

    def hardware_reset(self):
        self.hardware_resets += 1


def bare_camera(clock: FakeClock) -> HexCamRealsense:
    """Construct only the state used by recovery, bypassing all SDK startup."""
    cam = object.__new__(HexCamRealsense)
    cam._working = threading.Event()
    cam._working.set()
    cam._HexCamRealsense__streaming = threading.Event()
    cam._HexCamRealsense__streaming.set()
    cam._HexCamRealsense__serial_number = "D405-TEST"
    cam._HexCamRealsense__recovery_clock = clock.monotonic
    cam._HexCamRealsense__recovery_timeout_s = 180.0
    cam._HexCamRealsense__recovery_backoff_s = (1.0, 2.0, 5.0, 10.0, 30.0)
    cam._HexCamRealsense__hardware_reset_after_failures = 2
    cam._HexCamRealsense__hardware_reset_cooldown_s = 60.0
    cam._HexCamRealsense__hardware_reset_settle_s = 4.0
    cam._HexCamRealsense__pipeline = None
    cam._HexCamRealsense__pipeline_started = False
    cam._HexCamRealsense__state_pub = None
    cam._HexCamRealsense__closed_logged = False
    cam._HexCamRealsense__start_lock = lambda stop_event=None: nullcontext(True)
    cam._HexCamRealsense__stop_pipeline = lambda: None
    return cam


def test_wait_error_classification_distinguishes_disconnect_and_timeout():
    assert _classify_wait_for_frames_error(RuntimeError(
        "Device disconnected. Failed to reconnect: No device connected")) \
        == "disconnected"
    assert _classify_wait_for_frames_error(RuntimeError(
        "wait_for_frames cannot be called before start()")) == "disconnected"
    assert _classify_wait_for_frames_error(RuntimeError(
        "Frame didn't arrive within 5000")) == "timeout"


def test_disconnect_recovers_immediately_but_timeout_waits_for_third_failure():
    def run(errors):
        clock = FakeClock()
        cam = bare_camera(clock)
        cam._HexCamRealsense__frame_timeout_ms = 5000
        cam._HexCamRealsense__timeout_failures_before_recovery = 3
        calls = []

        class Pipeline:
            def wait_for_frames(self, timeout_ms):
                raise errors.pop(0)

        cam._HexCamRealsense__pipeline = Pipeline()
        cam._HexCamRealsense__align = object()
        cam._HexCamRealsense__recover_video_pipeline = (
            lambda rgb, depth, stop, exc: calls.append(str(exc)) or False)
        cam._HexCamRealsense__run_video_loop(
            deque(), deque(), FakeStopEvent(clock))
        return calls

    disconnect_calls = run([RuntimeError("Device disconnected")])
    timeout_calls = run([
        RuntimeError("Frame didn't arrive within 5000"),
        RuntimeError("Frame didn't arrive within 5000"),
        RuntimeError("Frame didn't arrive within 5000"),
    ])
    assert len(disconnect_calls) == 1
    assert len(timeout_calls) == 1


def test_recovery_is_paced_resets_after_two_present_failures_and_recovers():
    clock = FakeClock()
    stop_event = FakeStopEvent(clock)
    cam = bare_camera(clock)
    device = FakeDevice()
    attempts = []

    cam._HexCamRealsense__find_configured_device = lambda ctx: device

    def start_pipeline(ctx):
        attempts.append(clock.now)
        assert cam.is_working() is False
        if len(attempts) <= 2:
            raise RuntimeError("pipeline start failed")
        cam._HexCamRealsense__streaming.set()

    cam._HexCamRealsense__start_video_pipeline = start_pipeline
    original_context = camera_module.rs.context
    camera_module.rs.context = lambda: object()
    rgb = deque(["stale-rgb"])
    depth = deque(["stale-depth"])
    try:
        recovered = cam._HexCamRealsense__recover_video_pipeline(
            rgb, depth, stop_event, RuntimeError("Device disconnected"))
    finally:
        camera_module.rs.context = original_context

    assert recovered is True
    assert not rgb and not depth
    assert device.hardware_resets == 1
    assert len(attempts) == 3
    assert all(later - earlier >= 1.0
               for earlier, later in zip(attempts, attempts[1:]))
    assert stop_event.waits == [1.0, 2.0, 4.0]
    assert cam.is_working() is True


def test_recovery_deadline_exits_three_after_idempotent_cleanup():
    clock = FakeClock()
    stop_event = FakeStopEvent(clock)
    cam = bare_camera(clock)
    cam._HexCamRealsense__recovery_timeout_s = 3.0
    cam._HexCamRealsense__recovery_backoff_s = (1.0, 2.0)
    close_calls = []
    cam.close = lambda: close_calls.append(clock.now)
    cam._HexCamRealsense__find_configured_device = lambda ctx: (_ for _ in ()).throw(
        _RealSenseDeviceUnavailable("not present"))
    original_context = camera_module.rs.context
    camera_module.rs.context = lambda: object()
    try:
        try:
            cam._HexCamRealsense__recover_video_pipeline(
                deque(), deque(), stop_event, RuntimeError("Device disconnected"))
        except SystemExit as exc:
            assert exc.code == 3
        else:
            raise AssertionError("recovery deadline did not exit")
    finally:
        camera_module.rs.context = original_context

    assert close_calls == [3.0]
    assert stop_event.waits == [1.0, 2.0]


def test_failed_hardware_reset_is_still_rate_limited():
    clock = FakeClock()
    stop_event = FakeStopEvent(clock)
    cam = bare_camera(clock)
    cam._HexCamRealsense__recovery_timeout_s = 20.0
    reset_attempts = []

    class BrokenResetDevice:
        def hardware_reset(self):
            reset_attempts.append(clock.now)
            raise RuntimeError("USB reset failed")

    cam._HexCamRealsense__find_configured_device = (
        lambda ctx: BrokenResetDevice())
    cam._HexCamRealsense__start_video_pipeline = (
        lambda ctx: (_ for _ in ()).throw(RuntimeError("start failed")))
    cam.close = lambda: None
    original_context = camera_module.rs.context
    camera_module.rs.context = lambda: object()
    try:
        try:
            cam._HexCamRealsense__recover_video_pipeline(
                deque(), deque(), stop_event, RuntimeError("Device disconnected"))
        except SystemExit as exc:
            assert exc.code == 3
        else:
            raise AssertionError("recovery deadline did not exit")
    finally:
        camera_module.rs.context = original_context

    assert reset_attempts == [3.0]


def test_close_stops_pipeline_and_publisher_only_once():
    clock = FakeClock()
    cam = bare_camera(clock)
    del cam.__dict__["_HexCamRealsense__stop_pipeline"]

    class Resource:
        def __init__(self):
            self.closes = 0

        def close(self):
            self.closes += 1

    class Pipeline:
        def __init__(self):
            self.stops = 0

        def stop(self):
            self.stops += 1

    pipeline = Pipeline()
    publisher = Resource()
    cam._HexCamRealsense__pipeline = pipeline
    cam._HexCamRealsense__pipeline_started = True
    cam._HexCamRealsense__state_pub = publisher

    cam.close()
    cam.close()

    assert pipeline.stops == 1
    assert publisher.closes == 1
    assert cam.is_working() is False
