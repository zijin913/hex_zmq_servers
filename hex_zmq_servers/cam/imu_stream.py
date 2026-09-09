"""Non-blocking, versioned IMU fan-out for camera-owned RealSense devices.

The RealSense process remains the sole owner of the USB device.  Its SDK
callback only copies each motion sample into a bounded queue; this module's
worker owns the ZMQ socket and performs serialization.  A stalled or absent
subscriber therefore cannot delay video capture or the librealsense callback.

Wire format (three ZMQ frames)::

    b"imu/<source>/<kind>"
    UTF-8 JSON header
    little-endian float32[3]

``device_ts_ns`` is the unmodified RealSense device clock.  ``host_ts_ns`` is
``CLOCK_MONOTONIC`` at callback receipt.  Consumers must use receive time for
freshness and the device clock for integration; neither timestamp is a wall
clock.
"""

from __future__ import annotations

from collections import deque
import json
import math
import threading
import time
from typing import Any

import numpy as np

try:
    import zmq
except Exception:  # pragma: no cover - production image always has pyzmq
    zmq = None


SCHEMA_VERSION = 1
KINDS = ("gyro", "accel")
UNITS = {"gyro": "rad_s", "accel": "m_s2"}
DEFAULT_HWM = 256
DEFAULT_QUEUE_SIZE = 1024


def encode_imu_message(
    source: str,
    kind: str,
    seq: int,
    device_ts_ns: int,
    host_ts_ns: int,
    stream_epoch: int,
    values: tuple[float, float, float],
    *,
    frame: str = "camera_imu",
) -> tuple[bytes, bytes, bytes]:
    """Pure wire encoder shared by tests and the publisher worker."""

    source = str(source).strip()
    if not source or "/" in source:
        raise ValueError("IMU source must be a non-empty topic component")
    if kind not in KINDS:
        raise ValueError(f"unsupported IMU kind: {kind!r}")
    if any(isinstance(value, bool) or not isinstance(value, int) or value < 0
           for value in (seq, device_ts_ns, host_ts_ns, stream_epoch)):
        raise ValueError("IMU sequence and timestamps must be non-negative integers")
    vector = np.asarray(values, dtype="<f4")
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError("IMU payload must contain three finite values")
    header = {
        "schema": SCHEMA_VERSION,
        "source": source,
        "kind": kind,
        "seq": seq,
        "device_ts_ns": device_ts_ns,
        "host_ts_ns": host_ts_ns,
        "stream_epoch": stream_epoch,
        "frame": str(frame),
        "units": UNITS[kind],
    }
    return (
        f"imu/{source}/{kind}".encode("utf-8"),
        json.dumps(header, separators=(",", ":"), sort_keys=True).encode("utf-8"),
        vector.tobytes(),
    )


class ImuPublisher:
    """Bounded callback-to-PUB bridge whose worker exclusively owns ZMQ."""

    def __init__(
        self,
        port: int,
        *,
        source: str,
        frame: str = "camera_imu",
        ip: str = "127.0.0.1",
        hwm: int = DEFAULT_HWM,
        queue_size: int = DEFAULT_QUEUE_SIZE,
        endpoint: str | None = None,
    ) -> None:
        if zmq is None:
            raise RuntimeError("pyzmq not available")
        if not 1024 <= int(port) <= 65535:
            raise ValueError("IMU PUB port must be in [1024, 65535]")
        if int(hwm) <= 0 or int(queue_size) <= 0:
            raise ValueError("IMU HWM and queue size must be positive")
        source = str(source).strip()
        if not source or "/" in source:
            raise ValueError("IMU source must be a non-empty topic component")

        self._endpoint = (
            str(endpoint) if endpoint is not None
            else f"tcp://{ip}:{int(port)}"
        )
        if "://" not in self._endpoint:
            raise ValueError("IMU PUB endpoint must include a ZMQ transport")
        self._source = source
        self._frame = str(frame)
        self._hwm = int(hwm)
        self._queue_size = int(queue_size)
        self._condition = threading.Condition()
        self._queue: deque[tuple[str, int, int, tuple[float, float, float]]] = deque()
        self._stop = False
        self._ready = threading.Event()
        self._worker_error: str | None = None
        self._stream_epoch = time.monotonic_ns()
        self._seq = {kind: 0 for kind in KINDS}
        self._enqueued = {kind: 0 for kind in KINDS}
        self._published = {kind: 0 for kind in KINDS}
        self._queue_drops = {kind: 0 for kind in KINDS}
        self._zmq_drops = {kind: 0 for kind in KINDS}
        self._latest: dict[str, dict[str, int]] = {}
        self._thread = threading.Thread(
            target=self._run,
            name=f"realsense-imu-pub-{source}",
            daemon=True,
        )
        self._thread.start()
        if not self._ready.wait(1.0):
            self.close()
            raise TimeoutError(f"IMU publisher did not bind {self._endpoint}")
        if self._worker_error is not None:
            error = self._worker_error
            self.close()
            raise RuntimeError(f"IMU publisher failed to bind {self._endpoint}: {error}")

    def publish(
        self,
        kind: str,
        device_ts_ns: int,
        host_ts_ns: int,
        values: tuple[float, float, float],
    ) -> bool:
        """Queue one sample without waiting; oldest telemetry is dropped first."""

        if kind not in KINDS:
            return False
        try:
            vector = tuple(float(value) for value in values)
            valid = (
                len(vector) == 3
                and all(math.isfinite(value) for value in vector)
                and int(device_ts_ns) >= 0
                and int(host_ts_ns) >= 0
            )
        except (TypeError, ValueError):
            return False
        if not valid:
            return False
        item = (kind, int(device_ts_ns), int(host_ts_ns), vector)
        with self._condition:
            if self._stop:
                return False
            if len(self._queue) >= self._queue_size:
                dropped_kind = self._queue.popleft()[0]
                self._queue_drops[dropped_kind] += 1
            self._queue.append(item)
            self._enqueued[kind] += 1
            self._latest[kind] = {
                "device_ts_ns": int(device_ts_ns),
                "host_ts_ns": int(host_ts_ns),
            }
            self._condition.notify()
        return True

    def health(self) -> dict[str, Any]:
        with self._condition:
            return {
                "enabled": True,
                "endpoint": self._endpoint,
                "source": self._source,
                "schema": SCHEMA_VERSION,
                "stream_epoch": self._stream_epoch,
                "worker_alive": self._thread.is_alive(),
                "worker_error": self._worker_error,
                "queue_depth": len(self._queue),
                "queue_capacity": self._queue_size,
                "enqueued": dict(self._enqueued),
                "published": dict(self._published),
                "queue_drops": dict(self._queue_drops),
                "zmq_drops": dict(self._zmq_drops),
                "latest": {kind: dict(sample) for kind, sample in self._latest.items()},
            }

    def _run(self) -> None:
        socket = None
        try:
            context = zmq.Context.instance()
            socket = context.socket(zmq.PUB)
            socket.setsockopt(zmq.SNDHWM, self._hwm)
            socket.setsockopt(zmq.LINGER, 0)
            socket.bind(self._endpoint)
        except Exception as exc:
            with self._condition:
                self._worker_error = f"{type(exc).__name__}:{exc}"
            self._ready.set()
            if socket is not None:
                socket.close(0)
            return
        self._ready.set()

        try:
            while True:
                with self._condition:
                    while not self._queue and not self._stop:
                        self._condition.wait(timeout=0.1)
                    if self._stop:
                        return
                    kind, device_ts_ns, host_ts_ns, values = self._queue.popleft()
                    seq = self._seq[kind]
                    self._seq[kind] += 1
                parts = encode_imu_message(
                    self._source,
                    kind,
                    seq,
                    device_ts_ns,
                    host_ts_ns,
                    self._stream_epoch,
                    values,
                    frame=self._frame,
                )
                try:
                    socket.send_multipart(parts, flags=zmq.NOBLOCK)
                except zmq.Again:
                    with self._condition:
                        self._zmq_drops[kind] += 1
                else:
                    with self._condition:
                        self._published[kind] += 1
        except Exception as exc:
            with self._condition:
                self._worker_error = f"{type(exc).__name__}:{exc}"
        finally:
            socket.close(0)

    def close(self) -> None:
        with self._condition:
            if self._stop:
                return
            self._stop = True
            self._queue.clear()
            self._condition.notify_all()
        self._thread.join(timeout=1.0)


__all__ = [
    "DEFAULT_HWM",
    "DEFAULT_QUEUE_SIZE",
    "ImuPublisher",
    "KINDS",
    "SCHEMA_VERSION",
    "UNITS",
    "encode_imu_message",
]
