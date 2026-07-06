#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""High-rate state broadcaster: zmq.PUB with per-parameter topics + selective SUB.

Sits ALONGSIDE the REQ/REP get_states path (which stays as-is). This is a PUSH
stream for clients (IF-X, Plane 2) that want:
  * genuine high-rate feedback at the device work_loop rate (no polling), and
  * ROS-topic-style SELECTIVE subscription — a zmq.SUB filters by topic PREFIX at
    the socket level, so parameters you don't subscribe to are never delivered.

Topics are ``"<side>.<param>"`` (e.g. ``left.eff``). Params: pos / vel / eff, a combined
``state``, and (when the device has a gravity model) ``tau_ext`` — a model-based
external-torque ESTIMATE (measured effort minus modeled gravity at the measured pose;
NOT a certified F/T sensor). Payload is a float64 buffer ``[device_ts, host_ts, *values]``:
  * device_ts — the per-arm device sample clock (use for sensor-to-host latency).
  * host_ts   — ``time.monotonic()`` on the publishing host at send. Both the left and
                right arm servers run on the SAME host, and CLOCK_MONOTONIC is system-
                wide, so host_ts is a SINGLE clock comparable across the two arms —
                align left/right on host_ts (no PTP needed for a one-box cell).

SAFETY: the publisher is NON-BLOCKING (bounded SNDHWM + zmq.NOBLOCK). A slow or
absent subscriber can NEVER stall the device control loop — frames are dropped, not
queued unboundedly. This is why it is safe to call from robot_hexarm.work_loop.
"""
import time

import numpy as np

try:
    import zmq
except Exception:  # pragma: no cover - zmq always present in the device env
    zmq = None

PARAMS = ("pos", "vel", "eff")
# Send high-water mark: bounded (drop, never block) but must exceed one tick's
# burst of topics — the sim device publishes both arms + cameras on ONE socket
# (~17 msgs/tick); HWM 8 dropped the tail topics of every burst (seen as some
# topics at ~12 Hz while others ran ~250 Hz on the soda-smi panel).
DEFAULT_HWM = 64


def topic(side: str, param: str) -> bytes:
    return f"{side}.{param}".encode()


def _ts_to_float(ts):
    """Coerce a timestamp to float seconds. Accepts a plain number OR a hex
    timestamp dict (``{'s':.., 'ns':..}`` / ``{'sec':.., 'nsec':..}``); anything
    else -> 0.0. The REAL device work_loop hands in a hex-ts dict while the sim
    hands in a float — coercing here keeps the PUB robust so a call-site slip can
    never crash the device control loop (this is optional telemetry)."""
    if isinstance(ts, dict):
        return (float(ts.get("s", ts.get("sec", 0)))
                + float(ts.get("ns", ts.get("nsec", 0))) * 1e-9)
    try:
        return float(ts)
    except (TypeError, ValueError):
        return 0.0


def encode_frames(side, device_ts, host_ts, pos, vel, eff, tau_ext=None, ee=None):
    """Pure: return ``[(topic_bytes, payload_bytes), ...]`` for pos/vel/eff + state
    (+ optional ``tau_ext``).

    payload = float64 ``[device_ts, host_ts, *values]``. No sockets — unit-testable.

    ``tau_ext`` is a model-based EXTERNAL-TORQUE ESTIMATE: measured motor effort minus
    the modeled gravity/bias torque at the measured pose. It is an estimate from motor
    currents + the rigid-body model — NOT a certified 6-axis F/T sensor; unmodeled
    friction / payload / inertia show up as bias. The gripper entry is raw effort
    (no gravity model for the claw)."""
    head = [np.float64(_ts_to_float(device_ts)), np.float64(_ts_to_float(host_ts))]
    frames = []
    for name, arr in (("pos", pos), ("vel", vel), ("eff", eff)):
        vals = np.asarray(arr, dtype=np.float64).ravel()
        frames.append((topic(side, name), np.concatenate((head, vals)).tobytes()))
    combined = np.concatenate((head,
                               np.asarray(pos, np.float64).ravel(),
                               np.asarray(vel, np.float64).ravel(),
                               np.asarray(eff, np.float64).ravel()))
    frames.append((topic(side, "state"), combined.tobytes()))
    if tau_ext is not None:
        vals = np.asarray(tau_ext, dtype=np.float64).ravel()
        frames.append((topic(side, "tau_ext"), np.concatenate((head, vals)).tobytes()))
    if ee is not None:
        # [x,y,z, qx,qy,qz,qw] — link_6 pose in THIS arm's base frame, quaternion
        # xyzw (ROS convention). Computed by robot/ee_fk.py from the same gr100.urdf
        # on sim and real, so conventions are identical.
        vals = np.asarray(ee, dtype=np.float64).ravel()
        frames.append((topic(side, "ee"), np.concatenate((head, vals)).tobytes()))
    return frames


def decode_payload(payload):
    """Pure: ``(device_ts, host_ts, values)`` from a payload (values = ndarray)."""
    a = np.frombuffer(payload, dtype=np.float64)
    return float(a[0]), float(a[1]), a[2:].copy()


class StatePublisher:
    """Non-blocking zmq.PUB broadcaster bound to ``port``."""

    def __init__(self, port: int, hwm: int = DEFAULT_HWM, ip: str = "127.0.0.1"):
        if zmq is None:
            raise RuntimeError("pyzmq not available")
        self._ctx = zmq.Context.instance()
        # XPUB (a superset of PUB for sending) so we can see (un)subscribe notices and
        # skip the expensive camera JPEG encode when nobody is watching — see
        # jpeg_wanted(). Cheap topics (state/ee/tau_ext) still publish unconditionally.
        self._sock = self._ctx.socket(zmq.XPUB)
        self._sock.setsockopt(zmq.SNDHWM, int(hwm))  # bounded -> drop, never block
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.bind(f"tcp://{ip}:{int(port)}")
        self._subs = set()   # subscribed topic prefixes with >=1 live subscriber

    def publish(self, side, device_ts, pos, vel, eff, tau_ext=None, ee=None):
        # host_ts on the shared system-wide monotonic clock -> cross-arm alignment.
        # This is OPTIONAL telemetry: it must NEVER crash or stall the device control
        # loop (see module docstring). Swallow ALL errors, not just zmq.Again, so a
        # malformed arg (e.g. a bad ts/ee shape) can't take down the arm server.
        try:
            frames = encode_frames(side, device_ts, time.monotonic(),
                                   pos, vel, eff, tau_ext=tau_ext, ee=ee)
        except Exception:
            return
        for tp, pl in frames:
            try:
                self._sock.send_multipart([tp, pl], flags=zmq.NOBLOCK)
            except zmq.Again:
                pass  # subscriber slow/absent: drop this frame, keep the loop free
            except Exception:
                return  # never let a telemetry send error kill the control loop

    def _send_raw(self, tp: bytes, device_ts: float, payload: bytes):
        head = np.array([device_ts, time.monotonic()], dtype=np.float64).tobytes()
        try:
            self._sock.send_multipart([tp, head + payload], flags=zmq.NOBLOCK)
        except zmq.Again:
            pass

    def publish_jpeg(self, name: str, device_ts: float, jpg_bytes: bytes):
        """Camera frame topic ``cam.<name>.jpg``: 16-byte [device_ts, host_ts]
        header + JPEG bytes (bgr8-encoded, ROS image_transport/compressed idiom)."""
        self._send_raw(f"cam.{name}.jpg".encode(), device_ts, bytes(jpg_bytes))

    def _drain_subs(self):
        """Absorb any pending XPUB (un)subscribe notices into self._subs.
        Message = 1 byte (1=subscribe, 0=unsubscribe) + the subscribed topic prefix."""
        while True:
            try:
                msg = self._sock.recv(flags=zmq.NOBLOCK)
            except zmq.Again:
                return
            except Exception:
                return
            if not msg:
                continue
            if msg[0] == 1:
                self._subs.add(msg[1:])
            elif msg[0] == 0:
                self._subs.discard(msg[1:])

    def has_subscriber(self, topic: bytes) -> bool:
        """True if any current SUB prefix matches ``topic`` (drains pending notices
        first). Lets a producer skip expensive work for a topic nobody wants."""
        self._drain_subs()
        return any(topic.startswith(p) for p in self._subs) if self._subs else False

    def jpeg_wanted(self, name: str) -> bool:
        """Should we bother encoding + publishing ``cam.<name>.jpg``? False when no
        one is subscribed — so the caller skips the expensive cv2.imencode entirely.
        A newly-connected subscriber sees at most a ~1-frame startup delay."""
        return self.has_subscriber(f"cam.{name}.jpg".encode())

    def publish_json(self, tp: str, device_ts: float, obj: dict):
        """Low-rate metadata topic (e.g. ``cam.<name>.info`` ≈ ROS camera_info):
        16-byte ts header + UTF-8 JSON."""
        import json
        self._send_raw(tp.encode(), device_ts, json.dumps(obj).encode())

    def close(self):
        try:
            self._sock.close(0)
        except Exception:
            pass


class StateSubscriber:
    """zmq.SUB that connects to one or more publisher endpoints and filters by
    topic prefix — unsubscribed topics are never delivered."""

    def __init__(self, endpoints, topics, ip: str = "127.0.0.1", timeout_ms: int = 1000):
        if zmq is None:
            raise RuntimeError("pyzmq not available")
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.RCVTIMEO, int(timeout_ms))
        eps = endpoints if isinstance(endpoints, (list, tuple)) else [endpoints]
        for ep in eps:
            self._sock.connect(str(ep) if "://" in str(ep) else f"tcp://{ip}:{int(ep)}")
        for t in (topics or [b""]):
            self._sock.setsockopt(zmq.SUBSCRIBE, t if isinstance(t, bytes) else str(t).encode())

    def recv(self):
        """Blocking (up to timeout_ms) -> (topic, device_ts, host_ts, data).
        Raises zmq.Again on timeout. Align left/right on host_ts (shared host clock).

        ``data`` depends on the topic suffix:
          * ``*.jpg``  -> raw JPEG bytes (cv2.imdecode-able)
          * ``*.info`` -> dict (parsed JSON metadata)
          * otherwise  -> float64 ndarray (state topics)"""
        tp, pl = self._sock.recv_multipart()
        name = tp.decode()
        if name.endswith(".jpg") or name.endswith(".info"):
            head = np.frombuffer(pl[:16], dtype=np.float64)
            body = pl[16:]
            if name.endswith(".info"):
                import json
                body = json.loads(body.decode())
            return name, float(head[0]), float(head[1]), body
        device_ts, host_ts, vals = decode_payload(pl)
        return name, device_ts, host_ts, vals

    def close(self):
        try:
            self._sock.close(0)
        except Exception:
            pass
