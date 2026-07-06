"""Tests for the high-rate state broadcaster (zmq.PUB + selective SUB), IF-X Pilot 0.

Covers the two things IF cares about:
  1. wire format roundtrips (encode -> decode), and
  2. SELECTIVE subscription really filters — a SUB on only "left.eff" receives
     left.eff frames and NEVER the topics it did not subscribe to.

Run on SOMA:
    cd ~/Projects/soda-bimanual && python -c \
      "import sys;sys.path.insert(0,'hex_zmq_servers/tests');import test_state_pub as t;\
       [getattr(t,f)() for f in dir(t) if f.startswith('test_')];print('OK')"
"""
import time

import numpy as np

from hex_zmq_servers.robot.state_pub import (
    encode_frames, decode_payload, StatePublisher, StateSubscriber)


def test_encode_decode_roundtrip():
    pos = np.array([0.1, 0.2, 0.3])
    vel = np.array([1.0, 2.0, 3.0])
    eff = np.array([9.0, 8.0, 7.0])
    frames = dict(encode_frames("left", 123.5, 500.25, pos, vel, eff))
    assert set(frames) == {b"left.pos", b"left.vel", b"left.eff", b"left.state"}
    dts, hts, vals = decode_payload(frames[b"left.eff"])
    assert dts == 123.5 and hts == 500.25 and np.allclose(vals, eff)   # both clocks carried
    dts2, hts2, allv = decode_payload(frames[b"left.state"])
    assert dts2 == 123.5 and hts2 == 500.25 and np.allclose(allv, np.concatenate([pos, vel, eff]))


def test_encode_tau_ext_optional_topic():
    pos = vel = eff = np.zeros(3)
    # without tau_ext: 4 topics (unchanged legacy behaviour)
    assert set(dict(encode_frames("left", 1.0, 2.0, pos, vel, eff))) == {
        b"left.pos", b"left.vel", b"left.eff", b"left.state"}
    # with tau_ext: adds the 5th topic, roundtrips values + both clocks
    tau = np.array([0.5, -0.25, 0.0])
    frames = dict(encode_frames("left", 1.0, 2.0, pos, vel, eff, tau_ext=tau))
    assert b"left.tau_ext" in frames
    dts, hts, vals = decode_payload(frames[b"left.tau_ext"])
    assert dts == 1.0 and hts == 2.0 and np.allclose(vals, tau)


def test_encode_ee_optional_topic():
    z = np.zeros(3)
    ee = np.array([0.4, -0.1, 0.3, 0.0, 0.0, 0.0, 1.0])   # [xyz, quat xyzw]
    frames = dict(encode_frames("right", 1.0, 2.0, z, z, z, ee=ee))
    assert b"right.ee" in frames and b"right.tau_ext" not in frames
    dts, hts, vals = decode_payload(frames[b"right.ee"])
    assert vals.shape == (7,) and np.allclose(vals, ee)
    assert abs(np.linalg.norm(vals[3:]) - 1.0) < 1e-9      # unit quaternion carried intact


def test_jpeg_and_info_roundtrip_over_socket():
    import zmq
    port = 15573
    pub = StatePublisher(port)
    sub = StateSubscriber([port], [b"cam."], timeout_ms=300)   # prefix: all camera topics
    fake_jpg = b"\xff\xd8FAKEJPEG\xff\xd9" * 20
    try:
        got_jpg = got_info = None
        deadline = time.time() + 5.0
        while time.time() < deadline and not (got_jpg and got_info):
            pub.publish_jpeg("side", 3.5, fake_jpg)
            pub.publish_json("cam.side.info", 3.5,
                             {"width": 848, "height": 480, "format": "jpeg/bgr8"})
            try:
                tp, dts, hts, data = sub.recv()
                if tp == "cam.side.jpg":
                    got_jpg = (dts, data)
                elif tp == "cam.side.info":
                    got_info = (dts, data)
            except zmq.Again:
                pass
            time.sleep(0.02)
        assert got_jpg and got_jpg[0] == 3.5 and got_jpg[1] == fake_jpg   # bytes intact
        assert got_info and got_info[1]["width"] == 848                    # dict parsed
    finally:
        sub.close()
        pub.close()


def test_pubsub_selective_delivery():
    import zmq
    port = 15571
    pub = StatePublisher(port)
    sub = StateSubscriber([port], [b"left.eff"], timeout_ms=300)   # ONLY left.eff
    try:
        z = np.zeros(3)
        got = []
        deadline = time.time() + 5.0
        while time.time() < deadline and len(got) < 5:
            # publish BOTH arms and ALL params every round
            pub.publish("left", 1.0, z, z, np.array([7.0, 7.0, 7.0]))
            pub.publish("right", 1.0, z, z, np.array([3.0, 3.0, 3.0]))
            try:
                while True:
                    tp, dts, hts, vals = sub.recv()
                    got.append(tp)
                    assert hts > 0.0     # host clock stamped for cross-arm alignment
            except zmq.Again:
                pass
            time.sleep(0.02)
        assert got, "subscriber received nothing (PUB/SUB slow joiner?)"
        # THE point: only the subscribed topic arrives; pos/vel/state/right.* filtered out
        assert all(t == "left.eff" for t in got), f"leaked non-subscribed topics: {set(got)}"
    finally:
        sub.close()
        pub.close()


def test_pubsub_subscribe_all_when_empty_prefix():
    import zmq
    port = 15572
    pub = StatePublisher(port)
    sub = StateSubscriber([port], [b""], timeout_ms=300)   # empty prefix = everything
    try:
        z = np.zeros(2)
        seen = set()
        deadline = time.time() + 5.0
        while time.time() < deadline and len(seen) < 4:
            pub.publish("left", 2.0, z, z, z)
            try:
                while True:
                    tp, _, _, _ = sub.recv()
                    seen.add(tp)
            except zmq.Again:
                pass
            time.sleep(0.02)
        assert {"left.pos", "left.vel", "left.eff", "left.state"} <= seen
    finally:
        sub.close()
        pub.close()
