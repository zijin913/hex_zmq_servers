#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""High-rate state broadcaster: zmq.PUB with ROS-style typed topics + selective SUB.

Sits ALONGSIDE the REQ/REP get_states path (which stays as-is). This is a PUSH
stream for clients (IF-X, Plane 2) that want:
  * genuine high-rate feedback at the device work_loop rate (no polling), and
  * ROS-topic-style SELECTIVE subscription — a zmq.SUB filters by topic PREFIX at
    the socket level, so parameters you don't subscribe to are never delivered.

Topics are ROS-style ``"<side>/<msg>"`` (e.g. ``left/joint_states``), one standard message
per topic; a zmq.SUB filters by topic PREFIX so you receive only what you subscribe to:
  * ``<side>/joint_states`` — ``sensor_msgs/JointState`` style: ``[pos(7), vel(7), eff(7)]``
    (joint names fixed: joint_1..joint_6, gripper; effort = per-joint MOTOR torque). The three
    are ALSO published individually as ``<side>/pos`` · ``<side>/vel`` · ``<side>/eff`` for a
    client that wants just one signal at high rate.
  * ``<side>/ee_pose``      — ``geometry_msgs/PoseStamped`` style: ``[x,y,z, qx,qy,qz,qw]``,
    link_6 in the arm base frame (quaternion xyzw).
  * ``<side>/wrench``       — ``geometry_msgs/WrenchStamped`` style: ``[fx,fy,fz, mx,my,mz]``,
    the Cartesian EE force estimate (computed only while subscribed).
  * ``<side>/tau_ext``      — per-joint external-torque estimate (measured effort − modeled gravity).
``ee_pose``/``wrench``/``tau_ext`` need the gravity model; ``wrench``/``tau_ext`` are MODEL-BASED
estimates (motor torque + rigid-body model), NOT a certified F/T sensor. There is NO mega-
bundle: every topic from one work-loop tick carries the SAME ``[device_ts, host_ts]`` stamp,
so a client time-syncs across topics like ROS ``message_filters`` (align on host_ts). Payload
is a float64 buffer ``[device_ts, host_ts, *values]``:
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

PARAMS = ("pos", "vel", "eff", "joint_states", "tau_ext", "ee_pose", "wrench")
# Send high-water mark: bounded (drop, never block) but must exceed one tick's
# burst of topics — the sim device publishes both arms + cameras on ONE socket
# (~17 msgs/tick); HWM 8 dropped the tail topics of every burst (seen as some
# topics at ~12 Hz while others ran ~250 Hz on the soda-smi panel).
DEFAULT_HWM = 64


def topic(side: str, param: str) -> bytes:
    return f"{side}/{param}".encode()


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


def encode_frames(side, device_ts, host_ts, pos, vel, eff, tau_ext=None, ee=None,
                  ee_wrench=None):
    """Pure: return ``[(topic_bytes, payload_bytes), ...]`` for the ROS-style topic set.
    No sockets — unit-testable. Each payload is a float64 buffer ``[device_ts, host_ts,
    *values]``; the ``[device_ts, host_ts]`` pair is the message stamp — same tick =>
    same stamp across topics, so a client time-syncs like ROS ``message_filters``.

    Topics: ``<side>/joint_states`` (pos+vel+eff), ``<side>/ee_pose``, ``<side>/wrench``,
    ``<side>/tau_ext``. ``tau_ext``/``wrench`` are model-based ESTIMATES (motor torque +
    rigid-body model) — NOT a certified 6-axis F/T sensor; friction / payload / inertia
    show up as bias. The gripper effort entry is raw effort (no gravity model for the claw)."""
    head = [np.float64(_ts_to_float(device_ts)), np.float64(_ts_to_float(host_ts))]
    frames = []
    p = np.asarray(pos, np.float64).ravel()
    v = np.asarray(vel, np.float64).ravel()
    e = np.asarray(eff, np.float64).ravel()
    # Per-field topics (controller-style) for clients that want ONE signal at high rate —
    # selective SUB delivers only what you subscribe to. 7 joints [j1..j6, gripper].
    for name, arr in (("pos", p), ("vel", v), ("eff", e)):
        frames.append((topic(side, name), np.concatenate((head, arr)).tobytes()))
    # <side>/joint_states — sensor_msgs/JointState style: the same [pos(7), vel(7), eff(7)]
    # bundled in one message (joint names fixed: joint_1..joint_6, gripper; effort = motor torque).
    frames.append((topic(side, "joint_states"), np.concatenate((head, p, v, e)).tobytes()))
    if tau_ext is not None:
        # per-joint EXTERNAL-torque estimate: measured effort − modeled gravity.
        vals = np.asarray(tau_ext, dtype=np.float64).ravel()
        frames.append((topic(side, "tau_ext"), np.concatenate((head, vals)).tobytes()))
    if ee is not None:
        # geometry_msgs/PoseStamped style: [x,y,z, qx,qy,qz,qw] — link_6 in the arm base
        # frame, quaternion xyzw. Same gr100.urdf FK on sim and real (robot/ee_fk.py).
        vals = np.asarray(ee, dtype=np.float64).ravel()
        frames.append((topic(side, "ee_pose"), np.concatenate((head, vals)).tobytes()))
    if ee_wrench is not None:
        # geometry_msgs/WrenchStamped style: [fx,fy,fz, mx,my,mz] — quasi-static EE wrench
        # estimate F = sign·J^-T·tau_ext (robot/ee_fk.py::wrench). MODEL-BASED, NOT an F/T sensor.
        vals = np.asarray(ee_wrench, dtype=np.float64).ravel()
        frames.append((topic(side, "wrench"), np.concatenate((head, vals)).tobytes()))
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

    def publish(self, side, device_ts, pos, vel, eff, tau_ext=None, ee=None, ee_wrench=None):
        # host_ts on the shared system-wide monotonic clock -> cross-arm alignment.
        # This is OPTIONAL telemetry: it must NEVER crash or stall the device control
        # loop (see module docstring). Swallow ALL errors, not just zmq.Again, so a
        # malformed arg (e.g. a bad ts/ee shape) can't take down the arm server.
        try:
            frames = encode_frames(side, device_ts, time.monotonic(),
                                   pos, vel, eff, tau_ext=tau_ext, ee=ee, ee_wrench=ee_wrench)
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
        """Camera frame topic ``cam/<name>/image/compressed`` (≈ sensor_msgs/CompressedImage):
        16-byte [device_ts, host_ts] header + JPEG bytes (bgr8-encoded)."""
        self._send_raw(f"cam/{name}/image/compressed".encode(), device_ts, bytes(jpg_bytes))

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
        """Should we bother encoding + publishing ``cam/<name>/image/compressed``? False
        when no one is subscribed — so the caller skips the expensive cv2.imencode entirely.
        A newly-connected subscriber sees at most a ~1-frame startup delay."""
        return self.has_subscriber(f"cam/{name}/image/compressed".encode())

    def publish_json(self, tp: str, device_ts: float, obj: dict):
        """Low-rate metadata topic (e.g. ``cam/<name>/camera_info`` ≈ sensor_msgs/CameraInfo):
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

        ``data`` depends on the topic:
          * ``.../compressed``  -> raw JPEG bytes (cv2.imdecode-able)
          * ``.../camera_info`` -> dict (parsed JSON metadata)
          * otherwise           -> float64 ndarray (state topics)"""
        tp, pl = self._sock.recv_multipart()
        name = tp.decode()
        if name.endswith("/compressed") or name.endswith("/camera_info"):
            head = np.frombuffer(pl[:16], dtype=np.float64)
            body = pl[16:]
            if name.endswith("/camera_info"):
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
