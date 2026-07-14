#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Dual-arm Archer L6Y MuJoCo server.
# Based on HexMujocoE3DesktopServer dual-arm pattern.
################################################################

import threading
import time

import numpy as np
from collections import deque

try:
    from ..mujoco_base import HexMujocoServerBase
    from .mujoco_firefly_y6_dual import HexMujocoFireflyY6Dual
except (ImportError, ValueError):
    import sys
    from pathlib import Path
    this_file = Path(__file__).resolve()
    project_root = this_file.parents[3]
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))
    from hex_zmq_servers.mujoco.mujoco_base import HexMujocoServerBase
    from hex_zmq_servers.mujoco.firefly_y6.mujoco_firefly_y6_dual import HexMujocoFireflyY6Dual

NET_CONFIG = {
    "ip": "127.0.0.1",
    "port": 12345,
    "realtime_mode": False,
    "deque_maxlen": 10,
    "client_timeout_ms": 200,
    "server_timeout_ms": 1_000,
    "server_num_workers": 4,
}

MUJOCO_CONFIG = {
    "states_rate": 1000,
    "img_rate": 30,
    "headless": False,
    "sens_ts": True,
}


class HexMujocoFireflyY6DualServer(HexMujocoServerBase):

    def __init__(
        self,
        net_config: dict = NET_CONFIG,
        params_config: dict = MUJOCO_CONFIG,
    ):
        HexMujocoServerBase.__init__(self, net_config)

        # mujoco
        self._device = HexMujocoFireflyY6Dual(
            params_config, net_config.get("realtime_mode", False))

        # values
        self._cmds_left_seq = -1
        self._cmds_right_seq = -1
        self._states_left_queue = deque(maxlen=self._deque_maxlen)
        self._states_right_queue = deque(maxlen=self._deque_maxlen)
        self._states_obj_queue = deque(maxlen=self._deque_maxlen)
        # Command queues are LATEST-ONLY (maxlen=1), matching the real device
        # (HexRobotServerBase). With maxlen>1 + FIFO popleft, a burst of jog targets
        # buffers and the device replays them oldest-first when it falls behind —
        # the "move to next pose, then jump back / oscillate between poses" bug.
        # Always acting on the freshest target is correct for control.
        self._cmds_left_queue = deque(maxlen=1)
        self._cmds_right_queue = deque(maxlen=1)
        self._rgb_left_queue = deque(maxlen=self._deque_maxlen)
        self._depth_left_queue = deque(maxlen=self._deque_maxlen)
        self._rgb_right_queue = deque(maxlen=self._deque_maxlen)
        self._depth_right_queue = deque(maxlen=self._deque_maxlen)
        self._side_rgb_queue = deque(maxlen=self._deque_maxlen)
        self._side_depth_queue = deque(maxlen=self._deque_maxlen)

        # --- ROS-style high-rate state PUB (sim parity with the real arm servers) ---
        # Lets `soda stream`, the EE-wrench estimate and topic recording work WITHOUT
        # hardware. Publishes <side>/{pos,vel,eff,joint_states,tau_ext,ee_pose,wrench} on
        # the SAME ports the real servers use (site.yaml pub_port: left 12348, right
        # 12349). Everything below fails soft — a PUB/estimator error never stops the sim.
        self._pub_ports = {
            "left":  int(net_config.get("pub_port_left", 12348)),
            "right": int(net_config.get("pub_port_right", 12349)),
        }
        self._pub_hz = float(net_config.get("pub_hz", 100.0))
        self._pub_sign = float(net_config.get("ee_wrench_sign", 1.0))
        self._state_pubs = {}
        self._pub_fk = None
        self._pub_stop = threading.Event()
        self._pub_thread = None

        # Cameras — publish the MuJoCo-rendered frames as cam/<name>/image/compressed on
        # the per-cam ports (site.yaml cameras.*.pub_port 12352/12353/12354) so
        # `soda stream` auto-discovers them too. cam/<name> names mirror the device's.
        self._cam_specs = [
            ("left_wrist",  int(net_config.get("cam_pub_port_left",  12352)), self._rgb_left_queue),
            ("right_wrist", int(net_config.get("cam_pub_port_right", 12353)), self._rgb_right_queue),
            ("side",        int(net_config.get("cam_pub_port_side",  12354)), self._side_rgb_queue),
        ]
        self._cam_pubs = {}     # name -> StatePublisher
        self._cv2 = None

    def work_loop(self):
        self._start_state_pub()
        try:
            self._device.work_loop([
                self._states_left_queue,
                self._states_right_queue,
                self._states_obj_queue,
                self._cmds_left_queue,
                self._cmds_right_queue,
                self._rgb_left_queue,
                self._depth_left_queue,
                self._rgb_right_queue,
                self._depth_right_queue,
                self._side_rgb_queue,
                self._side_depth_queue,
                self._stop_event,
            ])
        finally:
            self._pub_stop.set()
            # JOIN the pub thread BEFORE closing its sockets — ZMQ sockets are not
            # thread-safe, so closing one while _state_pub_loop is mid-publish() on it
            # is undefined behavior (can segfault). Only close once the owner has stopped.
            if self._pub_thread is not None:
                self._pub_thread.join(timeout=2.0)
            for _p in list(self._state_pubs.values()) + list(self._cam_pubs.values()):
                try:
                    _p.close()
                except Exception:
                    pass
            self._device.close()

    def _start_state_pub(self):
        """Bring up the per-arm state PUB + the FK/gravity estimator, then start the
        publisher thread. Fail-soft: any import/setup error just disables the PUB."""
        try:
            from ...robot.state_pub import StatePublisher
            from ...robot.ee_fk import EEPoseFK
        except Exception:
            try:
                from hex_zmq_servers.robot.state_pub import StatePublisher
                from hex_zmq_servers.robot.ee_fk import EEPoseFK
            except Exception as e:
                print(f"\033[93m[sim] state PUB disabled (import failed: {e})\033[0m")
                return
        try:
            for side, port in self._pub_ports.items():
                self._state_pubs[side] = StatePublisher(int(port))
                print(f"\033[36m[sim] state PUB {side} on :{int(port)} "
                      f"(<side>/pos,vel,eff,joint_states,tau_ext,ee_pose,wrench)\033[0m")
        except Exception as e:
            print(f"\033[93m[sim] state PUB bind failed: {e}\033[0m")
            return
        try:
            self._pub_fk = EEPoseFK()     # ee_pose + wrench + gravity estimator
        except Exception as e:
            self._pub_fk = None           # pos/vel/eff/joint_states still publish
            print(f"\033[93m[sim] ee_pose/tau_ext/wrench estimate off ({e})\033[0m")
        try:
            import cv2
            self._cv2 = cv2
            for name, port, _q in self._cam_specs:
                self._cam_pubs[name] = StatePublisher(int(port))
            print(f"\033[36m[sim] camera PUB on {[p for _n, p, _q in self._cam_specs]} "
                  f"(cam/<name>/image/compressed)\033[0m")
        except Exception as e:
            self._cam_pubs = {}
            print(f"\033[93m[sim] camera PUB off ({e}); arm topics still published\033[0m")
        self._pub_thread = threading.Thread(target=self._state_pub_loop,
                                            name="sim-state-pub", daemon=True)
        self._pub_thread.start()

    def _state_pub_loop(self):
        """Snoop the latest per-arm state (pos/vel/eff) from the queues and publish the
        ROS-style topics at ``pub_hz``. The FK/gravity estimate (ee_pose/tau_ext/wrench)
        is computed ONLY while that topic is subscribed — same subscriber-gating as the
        real arm server, so an idle sim pays nothing for pinocchio."""
        queues = {"left": self._states_left_queue, "right": self._states_right_queue}
        dt = 1.0 / max(self._pub_hz, 1.0)
        while not self._pub_stop.is_set():
            t0 = time.perf_counter()
            for side, pub in self._state_pubs.items():
                q = queues.get(side)
                try:
                    ts, _cnt, states = q[-1]            # peek latest, no pop
                    states = np.asarray(states, dtype=np.float64)
                    pos, vel, eff = states[:, 0], states[:, 1], states[:, 2]
                except (IndexError, TypeError, ValueError, KeyError):
                    continue
                tau_ext = ee = ee_wrench = None
                if self._pub_fk is not None:
                    sd = side.encode()
                    want_tau = pub.has_subscriber(sd + b"/tau_ext")
                    want_wr = pub.has_subscriber(sd + b"/wrench")
                    want_ee = pub.has_subscriber(sd + b"/ee_pose")
                    qa = pos[:6]                         # 6 arm joints; gripper is index 6
                    if want_tau or want_wr:
                        try:
                            tau_ext = eff.copy()
                            tau_ext[:6] = eff[:6] - self._pub_fk.gravity(qa)
                        except Exception:
                            tau_ext = None
                    if want_ee:
                        try:
                            ee = self._pub_fk.compute(qa)
                        except Exception:
                            ee = None
                    if want_wr and tau_ext is not None:
                        try:
                            ee_wrench = self._pub_fk.wrench(qa, tau_ext[:6], self._pub_sign)
                        except Exception:
                            ee_wrench = None
                pub.publish(side, ts, pos, vel, eff,
                            tau_ext=tau_ext, ee=ee, ee_wrench=ee_wrench)
            # Cameras: JPEG-encode the latest MuJoCo frame per cam ONLY when subscribed
            # (jpeg_wanted gate), mirroring the device's own cam publish.
            if self._cv2 is not None:
                for name, _port, q in self._cam_specs:
                    pub = self._cam_pubs.get(name)
                    if pub is None or not pub.jpeg_wanted(name):
                        continue
                    try:
                        cts, _c, rgb = q[-1]                 # peek latest, no pop
                        ok, buf = self._cv2.imencode(".jpg", np.asarray(rgb))
                    except (IndexError, TypeError, ValueError):
                        continue
                    if ok:
                        dts = (float(cts.get("s", 0)) + float(cts.get("ns", 0)) * 1e-9
                               if isinstance(cts, dict) else float(cts))
                        try:
                            pub.publish_jpeg(name, dts, buf.tobytes())
                        except Exception:
                            pass
            # Always yield a small floor so a slow tick (subscribed FK/wrench/JPEG) or a
            # high pub_hz can never busy-spin the GIL and starve the MuJoCo control loop.
            self._pub_stop.wait(max(dt - (time.perf_counter() - t0), 5e-4))

    def _get_states(self, recv_hdr: dict):
        try:
            seq = recv_hdr["args"]
        except KeyError:
            print(f"\033[91m{recv_hdr['cmd']} requires `args`\033[0m")
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

        robot_name = recv_hdr["cmd"].split("_")[2]
        if robot_name == "left":
            queue = self._states_left_queue
        elif robot_name == "right":
            queue = self._states_right_queue
        elif robot_name == "obj":
            queue = self._states_obj_queue
        else:
            raise ValueError(
                f"unknown robot name: {robot_name} in {recv_hdr['cmd']}")

        try:
            ts, count, states = queue[
                -1] if self._realtime_mode else queue.popleft()
        except IndexError:
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None
        except Exception as e:
            print(f"\033[91m{recv_hdr['cmd']} failed: {e}\033[0m")
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

        delta = (count - seq) % self._max_seq_num
        if delta >= 0 and delta < 1e6:
            return {
                "cmd": f"{recv_hdr['cmd']}_ok",
                "ts": ts,
                "args": count
            }, states
        else:
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

    def _set_cmds(self, recv_hdr: dict, recv_buf: np.ndarray):
        seq = recv_hdr.get("args", None)
        if self._seq_clear_flag:
            self._seq_clear_flag = False
            self._cmds_left_seq = -1
            self._cmds_right_seq = -1
            return self.no_ts_hdr(recv_hdr, False), None

        robot_name = recv_hdr["cmd"].split("_")[2]
        if robot_name == "left":
            queue = self._cmds_left_queue
            cmds_seq = self._cmds_left_seq
        elif robot_name == "right":
            queue = self._cmds_right_queue
            cmds_seq = self._cmds_right_seq
        else:
            raise ValueError(f"unknown robot name: {robot_name}")

        if seq is not None and seq > cmds_seq:
            delta = (seq - cmds_seq) % self._max_seq_num
            if delta >= 0 and delta < 1e6:
                if robot_name == "left":
                    self._cmds_left_seq = seq
                elif robot_name == "right":
                    self._cmds_right_seq = seq
                queue.append((recv_hdr["ts"], seq, recv_buf))
                return self.no_ts_hdr(recv_hdr, True), None
            else:
                return self.no_ts_hdr(recv_hdr, False), None
        else:
            return self.no_ts_hdr(recv_hdr, False), None

    def _get_frame(self, recv_hdr: dict):
        try:
            seq = recv_hdr["args"]
        except KeyError:
            print(f"\033[91m{recv_hdr['cmd']} requires `args`\033[0m")
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

        split_cmd = recv_hdr["cmd"].split("_")
        depth_flag = split_cmd[1] == "depth"
        camera_name = split_cmd[2]
        if camera_name == "left":
            queue = self._depth_left_queue if depth_flag else self._rgb_left_queue
        elif camera_name == "right":
            queue = self._depth_right_queue if depth_flag else self._rgb_right_queue
        else:
            raise ValueError(f"unknown camera name: {camera_name}")

        try:
            ts, count, img = queue[
                -1] if self._realtime_mode else queue.popleft()
        except IndexError:
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None
        except Exception as e:
            print(f"\033[91m{recv_hdr['cmd']} failed: {e}\033[0m")
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

        delta = (count - seq) % self._max_seq_num
        if delta >= 0 and delta < 1e6:
            return {
                "cmd": f"{recv_hdr['cmd']}_ok",
                "ts": ts,
                "args": count
            }, img
        else:
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

    def _get_side_frame(self, recv_hdr: dict):
        try:
            seq = recv_hdr["args"]
        except KeyError:
            print(f"\033[91m{recv_hdr['cmd']} requires `args`\033[0m")
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

        depth_flag = "depth" in recv_hdr["cmd"]
        queue = self._side_depth_queue if depth_flag else self._side_rgb_queue
        try:
            ts, count, img = queue[
                -1] if self._realtime_mode else queue.popleft()
        except IndexError:
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None
        except Exception as e:
            print(f"\033[91m{recv_hdr['cmd']} failed: {e}\033[0m")
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

        delta = (count - seq) % self._max_seq_num
        if delta >= 0 and delta < 1e6:
            return {
                "cmd": f"{recv_hdr['cmd']}_ok",
                "ts": ts,
                "args": count
            }, img
        else:
            return {"cmd": f"{recv_hdr['cmd']}_failed"}, None

    def _process_request(self, recv_hdr: dict, recv_buf: np.ndarray):
        cmd = recv_hdr["cmd"]
        if cmd == "is_working":
            return self.no_ts_hdr(recv_hdr,
                                  self._device.is_working()), None
        elif cmd == "seq_clear":
            return self.no_ts_hdr(recv_hdr, self._seq_clear()), None
        elif cmd == "reset":
            return self.no_ts_hdr(recv_hdr, self._device.reset()), None
        elif cmd == "set_control_mode":
            ok = self._device.set_control_mode(recv_hdr.get("args"))
            return self.no_ts_hdr(recv_hdr, ok), None
        elif cmd == "get_dofs":
            dofs = self._device.get_dofs()
            return self.no_ts_hdr(recv_hdr, dofs is not None), dofs
        elif cmd == "get_limits":
            limits = self._device.get_limits()
            return self.no_ts_hdr(recv_hdr, limits is not None), limits
        elif cmd in ("set_cmds_left", "set_cmds_right"):
            return self._set_cmds(recv_hdr, recv_buf)
        elif cmd in ("get_states_left", "get_states_right", "get_states_obj"):
            return self._get_states(recv_hdr)
        elif cmd == "get_intri":
            intri = self._device.get_intri()
            return self.no_ts_hdr(recv_hdr, intri is not None), intri
        elif cmd in ("get_rgb_left", "get_depth_left", "get_rgb_right",
                     "get_depth_right"):
            return self._get_frame(recv_hdr)
        elif cmd == "get_side_intri":
            side_intri = self._device.get_side_intri()
            return self.no_ts_hdr(recv_hdr,
                                  side_intri is not None), side_intri
        elif cmd in ("get_side_rgb", "get_side_depth"):
            return self._get_side_frame(recv_hdr)
        else:
            raise ValueError(f"unknown command: {cmd}")


if __name__ == "__main__":
    import argparse, json
    from hex_zmq_servers.zmq_base import hex_server_helper

    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    hex_server_helper(cfg, HexMujocoFireflyY6DualServer)
