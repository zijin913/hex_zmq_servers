#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-14
################################################################

import os
import time
import threading
import numpy as np
from collections import deque

from ..robot_base import HexRobotBase
from ..mit_control import MitArmSafety, idle_gone_stale, build_safe_hold_cmd
from ...hex_launch import hex_log, HEX_LOG_LEVEL

from hex_robo_utils import (
    HexRate,
    hex_ts_delta_ms,
    hex_ts_now,
)
from hex_device import HexDeviceApi, Arm, Hands
from hex_device.motor_base import CommandType

ROBOT_CONFIG = {
    "device_ip": "172.18.8.161",
    "device_port": 8439,
    "control_hz": 250,
    "arm_type": "archer_y6",
    "mit_kp": [200.0, 200.0, 200.0, 75.0, 15.0, 15.0, 20.0],
    "mit_kd": [12.5, 12.5, 12.5, 6.0, 0.31, 0.31, 1.0],
    "sens_ts": True,
}

HEX_DEVICE_TYPE_DICT = {
    "archer_y6": 25,
    "archer_d6y": 16,
    "archer_l6y": 17,
    "firefly_y6": 27,
    "firefly_y6_h1": 27,  # 实机 launcher 配置(left/right_arm_cfg.json)用此别名
    "hello": 26,
}


class HexRobotHexarm(HexRobotBase):

    def __init__(
        self,
        robot_config: dict = ROBOT_CONFIG,
        realtime_mode: bool = False,
    ):
        HexRobotBase.__init__(self, realtime_mode)

        try:
            device_ip = robot_config["device_ip"]
            device_port = robot_config["device_port"]
            control_hz = robot_config["control_hz"]
            arm_type = HEX_DEVICE_TYPE_DICT[robot_config["arm_type"]]
            self.__sens_ts = robot_config["sens_ts"]
        except KeyError as ke:
            missing_key = ke.args[0]
            raise ValueError(
                f"robot_config is not valid, missing key: {missing_key}")

        self.__mit_kp = robot_config.get(
            "mit_kp",
            [200.0, 200.0, 200.0, 75.0, 15.0, 15.0, 20.0],
        )
        self.__mit_kd = robot_config.get(
            "mit_kd",
            [12.5, 12.5, 12.5, 6.0, 0.31, 0.31, 1.0],
        )

        # Gripper (Hands device) compliance for zero-gravity hand-posing. The
        # gripper is POSITION-controlled (its MIT kp/kd/torque are ignored) and
        # its internal position servo is too stiff to backdrive by hand. When a
        # client requests compliance (gripper-joint kp≈0, as the zero-gravity cli
        # sends) we relax it per gripper_compliant_mode:
        #   "torque"   — command zero torque so the motor goes limp and the claws
        #                backdrive by hand (default; the only thing that actually
        #                frees a stiff position servo). Won't hold a pose while
        #                limp. Falls back to a position hold if the firmware
        #                ignores torque commands.
        #   "position" — keep POSITION control but widen the Hands torque gate
        #                (set_pos_torque -> gripper_compliant_torque) so the
        #                streamed current angle tracks the hand. Gentler, but can
        #                still feel stiff if the motor's position loop is hard.
        self.__gripper_compliant_mode = str(
            robot_config.get("gripper_compliant_mode", "torque")).lower()
        self.__gripper_compliant_torque = float(
            robot_config.get("gripper_compliant_torque", 50.0))
        self.__gripper_default_torque = float(
            robot_config.get("gripper_default_torque", 3.0))
        self.__gripper_gate = None  # last set_pos_torque value (lazy-applied)

        # Idle-hold keep-alive (opt-in). When the client command stream has a
        # gap (teleop clutch released, pause, jitter, or no client at all),
        # re-send the last command / hold the measured pose so the firmware's
        # API watchdog keeps getting commands. Without this the arm parks with
        # PscApiCommunicationTimeout, the SDK churns reconnects, and the native
        # hex_device layer can segfault and kill the server. Default off so
        # behavior is unchanged unless a config opts in.
        self.__idle_hold = bool(robot_config.get("idle_hold", False))
        self.__idle_hold_period_ms = 1000.0 / float(
            robot_config.get("idle_hold_hz", 200.0))

        # Client-death safe-stop. After this many ms with no FRESH client command,
        # stop re-sending the last command (which would pin the arm rigid at its
        # last kp — up to ~200 — against whatever it was touching) and instead
        # stream a gravity-compensated COMPLIANT hold of the measured pose (soft
        # float; gripper stays clamped so a grasped payload is not dropped). Must
        # exceed the longest legitimate command gap (teleop clutch / policy pause /
        # inter-chunk). None/<=0 keeps the legacy hold-last-forever behavior.
        _ihm = robot_config.get("idle_hold_max_ms", 2000.0)
        self.__idle_hold_max_ms = (float(_ihm) if _ihm is not None
                                   and float(_ihm) > 0 else None)
        self.__idle_safe_kp_scale = float(
            robot_config.get("idle_safe_kp_scale", 0.15))
        self.__idle_safe_kd_scale = float(
            robot_config.get("idle_safe_kd_scale", 0.5))
        self.__idle_safe_active = False   # engaged-safe-hold flag (log once)

        # High-rate state broadcaster (zmq.PUB, optional). When pub_port is set,
        # publish per-parameter topics "<side>.{pos,vel,eff,state}" on every fresh
        # tick for clients that want push-based, selectively-subscribable feedback at
        # the device loop rate (alongside the REQ/REP get_states path). NON-BLOCKING:
        # a slow/absent subscriber drops frames and never stalls this control loop.
        self.__pub_side = str(robot_config.get("side", "arm"))
        # arm-PTP -> host-monotonic offset for the PUB device_ts (see __ptp_to_mono);
        # lazy-init on the first frame, then a decaying-max filter tracks slow PTP drift.
        self.__ptp_mono_offset = None
        self.__ptp_mono_offset_leak = 1e-7   # ~50 us/s decay @ 500 Hz
        self.__ts_to_float = None
        self.__state_pub = None
        self.__ee_fk = None
        _pub_port = robot_config.get("pub_port")
        if _pub_port:
            try:
                from ..state_pub import StatePublisher, _ts_to_float
                self.__state_pub = StatePublisher(int(_pub_port))
                self.__ts_to_float = _ts_to_float
                print(f"\033[36m[hexarm] state PUB on :{int(_pub_port)} "
                      f"(side={self.__pub_side})\033[0m")
            except Exception as e:
                print(f"\033[33m[hexarm] state PUB disabled: {e}\033[0m")
            # EE pose topic (<side>/ee_pose): link_6 in the arm base frame, quat xyzw.
            # Same gr100.urdf-based FK as the sim device -> identical conventions.
            try:
                from ..ee_fk import EEPoseFK
                self.__ee_fk = EEPoseFK()
            except Exception as e:
                print(f"\033[33m[hexarm] ee topic disabled: {e}\033[0m")

        # work_loop spin rate. Default 2000Hz oversamples 4x — arm state only
        # updates at the report rate (~control_hz). That wasted spinning is a
        # pure-Python GIL hog that starves the SDK's websocket-read thread in
        # the same process, so the controller can't push frames (ENOBUFS) and
        # the arm parks. Matching control_hz frees the GIL for the read thread.
        self.__work_loop_hz = float(
            robot_config.get("work_loop_hz", 2000.0))
        # Emit a state frame when the arm and gripper sample timestamps are within this
        # tolerance (was an exact <1ns match that dropped a whole cycle on any misalignment).
        # A slow 1-DOF gripper a few ms stale is harmless; default 1.5 control periods.
        self.__ts_pair_tol_ms = float(
            robot_config.get("ts_pair_tol_ms", 1.5 * 1000.0 / control_hz))

        # Gravity feedforward (opt-in). MIT commands here carry zero torque
        # feedforward, so position-only commands settle below target by g/kp and
        # a hold (target≈measured) droops. When enabled, add tau_g(q) computed
        # from the firefly_y6 + gr100 URDF (pinocchio) to the arm torque so the
        # arm holds its commanded pose. Benefits every client (teleop included).
        # gravity_comp_scale<1 leaves a controlled residual sag. Default off so
        # behavior is unchanged unless a config opts in.
        self.__gravity_comp = bool(robot_config.get("gravity_comp", False))
        self.__grav_scale = float(robot_config.get("gravity_comp_scale", 1.0))

        # <side>/wrench = ee_wrench_sign · J(q)^-T · tau_ext — quasi-static Cartesian
        # force estimate published on the state stream. Sign default +1 matches
        # tools/ee_wrench_check.py's raw solve; flip to -1 after the known-weight
        # calibration so a downward hung load reads -z (external wrench applied to the
        # robot). Only computed when a client is subscribed (see work_loop).
        self.__ee_wrench_sign = float(robot_config.get("ee_wrench_sign", 1.0))

        # Reference slew (anti-lunge) — IDENTICAL to the sim device so sim predicts
        # real. Cap how far the commanded arm target may lead the MEASURED angle
        # (rad); kp*(q_des-q) then stays inside the motor torque envelope, so a
        # far / stale / aggressive setpoint becomes a smooth bounded-speed pursuit
        # instead of saturating and lunging. Default 0.1 (the sim default) so the
        # real arm is protected even without an explicit cfg; <=0 disables. Only
        # binds on a far jump — dense policy/home/jog steps are well under it.
        _mpe = robot_config.get("max_pos_err", 0.1)
        self.__max_pos_err = float(_mpe) if _mpe and float(_mpe) > 0 else None

        # Control mode: "position" | "joint_impedance" | "torque". Decided purely by the
        # command column-count in __set_cmds (soda_os shapes the 5-col MIT command), so
        # the device needs no per-mode branch; the flag is kept for state/logging only.
        self.__control_mode = str(
            robot_config.get("control_mode", "position")).lower()

        # Shared MIT-arm safety (gravity feedforward + effort/slew clamp). Built from
        # the SAME config keys (incl. gravity_comp_scale_lowstiff) via from_config so
        # the sim and real devices read everything identically — the sim is a faithful
        # pre-flight test. (robot/mit_control.py)
        self.__safety = MitArmSafety.from_config(robot_config)

        # Latest measured arm position (cached by work_loop) — the max_pos_err guard
        # needs measured q, which __set_cmds doesn't otherwise receive.
        self.__last_q = None

        # Pinocchio model — needed for gravity feedforward.
        self.__pin = None
        self.__grav_model = None
        self.__grav_data = None
        if self.__gravity_comp:
            try:
                import pinocchio as pin
                urdf = os.path.join(os.path.dirname(__file__),
                                    "urdf", "firefly_y6", "gr100.urdf")
                self.__pin = pin
                self.__grav_model = pin.buildModelFromUrdf(urdf)
                self.__grav_data = self.__grav_model.createData()
                self.__grav_model.gravity.linear = np.array([0.0, 0.0, -9.81])
                hex_log(HEX_LOG_LEVEL["info"],
                        f"[hexarm] pinocchio ON (gravity_comp={self.__gravity_comp} "
                        f"scale={self.__grav_scale}, control_mode={self.__control_mode})")
            except Exception as e:
                print(f"\033[91m[hexarm] pinocchio init failed: {e}\033[0m")
                self.__gravity_comp = False

        # variables
        # hex_arm variables
        self.__hex_api: HexDeviceApi | None = None
        self.__arm: Arm | None = None
        self.__gripper: Hands | None = None

        # buffer
        self.__arm_state_buffer: dict | None = None
        self.__gripper_state_buffer: dict | None = None

        # open device
        self.__hex_api = HexDeviceApi(
            ws_url=f"ws://{device_ip}:{device_port}",
            control_hz=control_hz,
        )

        # open arm
        while self.__hex_api.find_device_by_robot_type(arm_type) is None:
            print("\033[33mArm not found\033[0m")
            time.sleep(1)
        self.__arm = self.__hex_api.find_device_by_robot_type(arm_type)
        self.__arm.start()

        # try to open gripper
        self.__gripper = self.__hex_api.find_optional_device_by_id(1)
        if self.__gripper is None:
            print("\033[33mGripper not found\033[0m")
        else:
            # Override SDK's hardcoded gripper position limit if requested.
            # hex_device SDK ships GR100 with [0, 0.57] which is conservative;
            # real hardware can rotate further before mechanical stop.
            # Set robot_config["gripper_max_position"] to bypass the SDK clamp.
            gmax = robot_config.get("gripper_max_position")
            if gmax is not None:
                try:
                    self.__gripper._hands_limit[1] = float(gmax)
                    print(f"\033[33m[hexarm] gripper limit override: upper={float(gmax)}\033[0m")
                except (AttributeError, IndexError) as e:
                    print(f"\033[33m[hexarm] failed to override gripper limit: {e}\033[0m")

        # variables init
        arm_dofs = len(self.__arm)
        self._dofs = [arm_dofs]
        self._limits = np.array(self.__arm.get_joint_limits()).reshape(
            -1, 3, 2)
        self.__motor_idx = {"robot_arm": np.arange(arm_dofs).tolist()}
        if self.__gripper is not None:
            gripper_dofs = len(self.__gripper)
            self._dofs.append(gripper_dofs)
            gripper_limits = np.array(
                self.__gripper.get_joint_limits()).reshape(-1, 3, 2)
            self._limits = np.concatenate([self._limits, gripper_limits],
                                          axis=0)
            self.__motor_idx["robot_gripper"] = (np.arange(gripper_dofs) +
                                                 arm_dofs).tolist()

        # modify variables
        self._dofs = np.array(self._dofs)
        self._dofs_sum = self._dofs.sum()
        self._limits = np.ascontiguousarray(np.asarray(self._limits)).reshape(
            self._dofs_sum, 3, 2)
        self.__mit_kp = np.ascontiguousarray(np.asarray(self.__mit_kp))
        self.__mit_kd = np.ascontiguousarray(np.asarray(self.__mit_kd))
        if self.__mit_kp.shape[0] < self._dofs_sum or self.__mit_kd.shape[
                0] < self._dofs_sum:
            raise ValueError(
                "The length of mit_kp and mit_kd must be greater than or equal to the number of motors"
            )
        elif self.__mit_kp.shape[0] > self._dofs_sum or self.__mit_kd.shape[
                0] > self._dofs_sum:
            print(
                f"\033[33mThe length of mit_kp and mit_kd is greater than the number of motors\033[0m"
            )
            self.__mit_kp = self.__mit_kp[:self._dofs_sum]
            self.__mit_kd = self.__mit_kd[:self._dofs_sum]

        # Precompute the compliant safe-hold gains used on client death: arm joints
        # get a fraction of the MIT gains (soft float that resists sag; 0 = pure
        # zero-gravity float), the gripper keeps its full kp so it stays clamped.
        _arm_idx = self.__motor_idx["robot_arm"]
        self.__idle_safe_kp = self.__mit_kp[_arm_idx] * self.__idle_safe_kp_scale
        self.__idle_safe_kd = self.__mit_kd[_arm_idx] * self.__idle_safe_kd_scale
        _g = self.__motor_idx.get("robot_gripper")
        self.__idle_safe_grip_kp = float(self.__mit_kp[_g[0]]) if _g else 0.0
        self.__idle_safe_grip_kd = float(self.__mit_kd[_g[0]]) if _g else 0.0

        # start work loop
        self._working.set()

    def work_loop(self, hex_queues: list[deque | threading.Event]):
        states_queue = hex_queues[0]
        cmds_queue = hex_queues[1]
        stop_event = hex_queues[2]

        last_states_ts = hex_ts_now()
        states_count = 0
        last_cmds_seq = -1
        last_cmds = None       # most recent client command (idle-hold source)
        hold_pos = None        # latest measured pose (live)
        idle_target = None     # latched pose held while idle (fixed, not chased)
        last_send_ts = hex_ts_now()
        last_real_cmd_ts = hex_ts_now()  # last FRESH client command (safe-stop timer)
        rate = HexRate(self.__work_loop_hz)
        # --- optional hot-loop profiler (SODA_LOOP_PROFILE=1): per-stage SDK
        # call latency (read = get_simple_motor_status, write = __set_cmds ->
        # motor_command), dumped every 5 s OFF the hot path. Near-zero cost when
        # off (one bool test/tick). GATE 1: does motor_command enqueue (us) or
        # block on a controller ack (ms)?  No behaviour change; timing only.
        import os as _os, time as _time, threading as _threading
        _prof_on = bool(_os.environ.get("SODA_LOOP_PROFILE"))
        if _prof_on:
            _N = max(2000, int(self.__work_loop_hz) * 5)
            _rd = np.zeros(_N); _wr = np.zeros(_N); _lp = np.zeros(_N)
            _idx = {"r": 0, "w": 0, "l": 0}
            _pstop = _threading.Event()

            def _prof_dump():
                while not _pstop.wait(5.0):
                    def _s(a, n):
                        v = a[:min(n, _N)]; v = v[v > 0]
                        if v.size == 0:
                            return "n=0"
                        return (f"p50={np.percentile(v, 50) / 1e3:7.1f} "
                                f"p99={np.percentile(v, 99) / 1e3:7.1f} "
                                f"max={v.max() / 1e3:8.1f}us n={v.size}")
                    print(f"[PROF] read  {_s(_rd, _idx['r'])}\n"
                          f"[PROF] write {_s(_wr, _idx['w'])}\n"
                          f"[PROF] loop  {_s(_lp, _idx['l'])}", flush=True)
            _threading.Thread(target=_prof_dump, name="loop_prof",
                              daemon=True).start()
        # --- optional RT control path (SODA_RT_CONTROL=1): pin THIS control loop
        # (the device main thread) to an isolated core (left->6 / right->7, to
        # match isolcpus=6,7) + SCHED_FIFO-80, and force the PUB decouple on so
        # the serialize/send never blocks the 1 kHz loop. Best-effort + guarded:
        # any failure logs and falls back to normal scheduling — a physical arm
        # must never fail to start over RT setup. Requires the box provisioned by
        # scripts/rt/rt_setup.sh (isolcpus + rtprio). SODA_RT_CORE overrides.
        _rt_on = bool(_os.environ.get("SODA_RT_CONTROL"))
        if _rt_on:
            try:
                _rc = _os.environ.get("SODA_RT_CORE")
                _core = int(_rc) if _rc else (6 if self.__pub_side == "left" else 7)
                _os.sched_setaffinity(0, {_core})
                _os.sched_setscheduler(0, _os.SCHED_FIFO, _os.sched_param(80))
                print(f"[hexarm] RT control loop -> core {_core}, SCHED_FIFO-80")
            except Exception as e:
                print(f"[hexarm] RT setup failed ({e}); default scheduling")
        # --- PUB decouple (SODA_PUB_THREAD=1, or implied by SODA_RT_CONTROL): move
        # the ~0.5 ms state serialize/encode/send OFF this hot loop to a non-RT
        # publisher thread (latest-wins, coalescing). The hot loop only hands off
        # the latest frame (states.copy + notify, ~us).
        self.__pub_thread_on = ((bool(_os.environ.get("SODA_PUB_THREAD")) or _rt_on)
                                and self.__state_pub is not None)
        self.__pub_rt = _rt_on
        # SODA_EMIT_STATS: source-side emit-rate diagnostic — device-read vs PUB-send /s
        # (pub_send < device_read => publisher coalescing; device_read < ~500 => SDK/work-loop limit).
        self.__emit_stats = bool(_os.environ.get("SODA_EMIT_STATS"))
        self.__fresh_n = 0
        self.__emit_n = 0
        self.__emit_t0 = _time.perf_counter()
        if self.__pub_thread_on:
            self.__pub_cv = _threading.Condition()
            self.__pub_slot = None
            self.__pub_stop = _threading.Event()
            _threading.Thread(target=self.__pub_worker, name="pub_worker",
                              daemon=True).start()
        while self._working.is_set() and not stop_event.is_set():
            # states
            _t0 = _time.perf_counter_ns() if _prof_on else 0
            ts, states = self.__get_states()
            if _prof_on:
                _rd[_idx["r"] % _N] = _time.perf_counter_ns() - _t0
                _idx["r"] += 1
            if states is not None:
                hold_pos = states[:, 0]
                # Cache measured arm q for the max_pos_err guard (which runs inside
                # __set_cmds and otherwise has no access to measured state).
                arm_ids = self.__motor_idx["robot_arm"]
                self.__last_q = states[arm_ids, 0]
                if hex_ts_delta_ms(ts, last_states_ts) > 1e-6:
                    last_states_ts = ts
                    if self.__emit_stats:
                        self.__fresh_n += 1
                    states_queue.append((ts, states_count, states))
                    states_count = (states_count + 1) % self._max_seq_num
                    # Re-anchor the PUB device_ts from the arm PTP clock to host
                    # CLOCK_MONOTONIC so host_ts - device_ts is a true sensor->host
                    # latency, not a clock-domain offset. REQ/REP keeps the raw PTP ts.
                    pub_ts = self.__ptp_to_mono(ts) if self.__state_pub is not None else ts
                    # High-rate PUB broadcast of this fresh frame (non-blocking).
                    if self.__state_pub is not None and self.__pub_thread_on:
                        # Hand off to the non-RT publisher thread (cheap copy +
                        # notify); serialize/encode/send runs OFF this hot loop.
                        with self.__pub_cv:
                            self.__pub_slot = (pub_ts, states.copy())
                            self.__pub_cv.notify()
                    elif self.__state_pub is not None:
                        # The ESTIMATE layer (pinocchio: gravity, FK, Jacobian) is computed
                        # ONLY when a client is subscribed to that topic — it stays OFF this
                        # GIL-shared work loop (SDK read thread) when nobody is watching, like
                        # the camera JPEG encode. pos/vel/eff/joint_states are cheap slices and
                        # are always published (ZMQ drops them if unsubscribed).
                        _sd = self.__pub_side
                        sp = self.__state_pub
                        sub_tau = sp.has_subscriber(f"{_sd}/tau_ext".encode())
                        sub_wrench = sp.has_subscriber(f"{_sd}/wrench".encode())
                        sub_ee = sp.has_subscriber(f"{_sd}/ee_pose".encode())
                        # tau_ext (measured effort − modeled gravity): a motor-current + model
                        # ESTIMATE, NOT an F/T sensor. Needed for /tau_ext AND /wrench.
                        tau_ext = None
                        if self.__pin is not None and (sub_tau or sub_wrench):
                            try:
                                tau_ext = states[:, 2].astype(np.float64).copy()
                                tau_ext[arm_ids] -= self.__gravity_fn(states[arm_ids, 0])
                            except Exception:
                                tau_ext = None
                        # ee_pose (forwardKinematics) only when /ee_pose is subscribed.
                        ee = None
                        if self.__ee_fk is not None and sub_ee:
                            try:
                                ee = self.__ee_fk.compute(states[arm_ids, 0])
                            except Exception:
                                ee = None
                        # wrench (Jacobian + solve) only when /wrench is subscribed.
                        ee_wrench = None
                        if self.__ee_fk is not None and tau_ext is not None and sub_wrench:
                            try:
                                ee_wrench = self.__ee_fk.wrench(
                                    states[arm_ids, 0], tau_ext[arm_ids], self.__ee_wrench_sign)
                            except Exception:
                                ee_wrench = None
                        self.__state_pub.publish(self.__pub_side, pub_ts,
                                                 states[:, 0], states[:, 1], states[:, 2],
                                                 tau_ext=tau_ext, ee=ee, ee_wrench=ee_wrench)
                        if self.__emit_stats:
                            self.__count_emit()

            # cmds
            cmds_pack = None
            try:
                cmds_pack = cmds_queue[
                    -1] if self._realtime_mode else cmds_queue.popleft()
            except IndexError:
                pass
            sent = False
            if cmds_pack is not None:
                ts, seq, cmds = cmds_pack
                if seq != last_cmds_seq:
                    last_cmds_seq = seq
                    last_cmds = cmds
                    idle_target = None   # active stream: re-latch fresh on next idle
                    last_real_cmd_ts = hex_ts_now()  # client alive: reset safe-stop timer
                    if self.__idle_safe_active:      # leaving safe-hold on a fresh cmd
                        self.__idle_safe_active = False
                        hex_log(HEX_LOG_LEVEL["info"],
                                "[hexarm] client command resumed -> exit safe-hold")
                    # 命令时间戳新鲜度校验已移除:始终下发(see fork history)
                    # Never let a malformed command kill the control loop / server.
                    try:
                        _tw = _time.perf_counter_ns() if _prof_on else 0
                        self.__set_cmds(cmds)
                        if _prof_on:
                            _wr[_idx["w"] % _N] = _time.perf_counter_ns() - _tw
                            _idx["w"] += 1
                    except Exception as e:
                        print(f"\033[91m[hexarm] set_cmds error: {e}\033[0m")
                    last_send_ts = hex_ts_now()
                    sent = True

            # idle-hold keep-alive — feed the firmware's API watchdog when the
            # client command stream has a gap, so the arm holds position instead
            # of parking (PscApiCommunicationTimeout) and churning the SDK into a
            # native crash. Throttled to idle_hold_hz.
            #
            # Short gap (< idle_hold_max_ms): re-send the last real command if
            # there was one; otherwise hold a *latched* pose captured once when
            # the arm went idle. (A fixed target builds real kp*(target-measured)
            # torque and holds; feeding the LIVE measured pose with no gravity FF
            # would droop — but see the safe-hold below, which carries gravity FF.)
            #
            # Stale (>= idle_hold_max_ms, i.e. the client is presumed DEAD): drop
            # to a gravity-compensated COMPLIANT hold of the measured pose instead
            # of pinning the arm rigid at its last kp forever — safe-stop on client
            # death. The gripper stays clamped so a grasped payload is not dropped.
            if self.__idle_hold and not sent and hex_ts_delta_ms(
                    hex_ts_now(), last_send_ts) >= self.__idle_hold_period_ms:
                if idle_gone_stale(
                        hex_ts_delta_ms(hex_ts_now(), last_real_cmd_ts),
                        self.__idle_hold_max_ms) and hold_pos is not None:
                    hold = build_safe_hold_cmd(
                        hold_pos,
                        self.__motor_idx["robot_arm"],
                        self.__motor_idx.get("robot_gripper"),
                        self.__idle_safe_kp, self.__idle_safe_kd,
                        self.__idle_safe_grip_kp, self.__idle_safe_grip_kd)
                    if not self.__idle_safe_active:
                        self.__idle_safe_active = True
                        hex_log(HEX_LOG_LEVEL["info"],
                                "[hexarm] client idle > idle_hold_max_ms -> "
                                "compliant safe-hold (gripper stays clamped)")
                elif last_cmds is not None:
                    hold = last_cmds
                else:
                    if idle_target is None and hold_pos is not None:
                        idle_target = hold_pos.copy()   # latch once
                    hold = idle_target
                if hold is not None:
                    try:
                        _tw = _time.perf_counter_ns() if _prof_on else 0
                        self.__set_cmds(hold)
                        if _prof_on:
                            _wr[_idx["w"] % _N] = _time.perf_counter_ns() - _tw
                            _idx["w"] += 1
                    except Exception as e:
                        print(f"\033[91m[hexarm] idle-hold set_cmds error: {e}\033[0m")
                    last_send_ts = hex_ts_now()

            # sleep
            if _prof_on:
                _lp[_idx["l"] % _N] = _time.perf_counter_ns() - _t0
                _idx["l"] += 1
            rate.sleep()

        # stop the PUB worker before teardown
        if self.__pub_thread_on:
            self.__pub_stop.set()
            with self.__pub_cv:
                self.__pub_cv.notify()
        # close
        self.close()

    def __pub_worker(self):
        """Non-RT publisher: drain the latest state frame handed off by the hot
        control loop and run __do_publish OFF that thread (latest-wins /
        coalescing). Keeps the ~0.5 ms serialize/encode/send off the 1 kHz path."""
        if self.__pub_rt:
            # RT path: keep the feedback publisher OFF the isolated control cores
            # (6,7) NOR the reserved arm-NIC-IRQ core; FIFO-40 on the housekeeping
            # cores (all minus isolcpus) -> responsive without ever
            # preempting the control loop. Guarded (never crash the publisher).
            try:
                import os as _os
                _iso = set()
                try:
                    for _p in open("/sys/devices/system/cpu/isolated").read().strip().split(","):
                        if "-" in _p:
                            _a, _b = _p.split("-"); _iso.update(range(int(_a), int(_b) + 1))
                        elif _p:
                            _iso.add(int(_p))
                except Exception:
                    pass
                _hk = {c for c in range(_os.cpu_count() or 1) if c not in _iso} or {0}
                _os.sched_setaffinity(0, _hk)
                _os.sched_setscheduler(0, _os.SCHED_FIFO, _os.sched_param(40))
            except Exception as e:
                print(f"[hexarm] pub_worker RT setup failed ({e})")
        while not self.__pub_stop.is_set():
            with self.__pub_cv:
                while self.__pub_slot is None and not self.__pub_stop.is_set():
                    self.__pub_cv.wait(0.1)
                if self.__pub_stop.is_set():
                    return
                ts, states = self.__pub_slot
                self.__pub_slot = None
            try:
                self.__do_publish(ts, states)
            except Exception as e:
                print(f"\033[91m[hexarm] pub_worker error: {e}\033[0m")

    def __do_publish(self, ts, states):
        """Serialize + (subscriber-gated pinocchio) estimate + PUB-send one state
        frame. Called inline (SODA_PUB_THREAD off) or from __pub_worker (on)."""
        arm_ids = self.__motor_idx["robot_arm"]
        _sd = self.__pub_side
        sp = self.__state_pub
        sub_tau = sp.has_subscriber(f"{_sd}/tau_ext".encode())
        sub_wrench = sp.has_subscriber(f"{_sd}/wrench".encode())
        sub_ee = sp.has_subscriber(f"{_sd}/ee_pose".encode())
        tau_ext = None
        if self.__pin is not None and (sub_tau or sub_wrench):
            try:
                tau_ext = states[:, 2].astype(np.float64).copy()
                tau_ext[arm_ids] -= self.__gravity_fn(states[arm_ids, 0])
            except Exception:
                tau_ext = None
        ee = None
        if self.__ee_fk is not None and sub_ee:
            try:
                ee = self.__ee_fk.compute(states[arm_ids, 0])
            except Exception:
                ee = None
        ee_wrench = None
        if self.__ee_fk is not None and tau_ext is not None and sub_wrench:
            try:
                ee_wrench = self.__ee_fk.wrench(
                    states[arm_ids, 0], tau_ext[arm_ids], self.__ee_wrench_sign)
            except Exception:
                ee_wrench = None
        self.__state_pub.publish(self.__pub_side, ts,
                                 states[:, 0], states[:, 1], states[:, 2],
                                 tau_ext=tau_ext, ee=ee, ee_wrench=ee_wrench)
        if self.__emit_stats:
            self.__count_emit()

    def __count_emit(self):
        """SODA_EMIT_STATS: print source-side rates once/sec. device_read = fresh device
        frames the hot loop saw; pub_send = frames actually serialized + PUB-sent. pub_send
        < device_read => the publisher is coalescing (serialize/send can't keep up under
        load); device_read < ~500 => the SDK read / work loop is the limit, not a consumer."""
        self.__emit_n += 1
        _now = time.perf_counter()
        _dt = _now - self.__emit_t0
        if _dt >= 1.0:
            print(f"[emit] {self.__pub_side}: device_read={self.__fresh_n / _dt:6.1f}/s  "
                  f"pub_send={self.__emit_n / _dt:6.1f}/s", flush=True)
            self.__fresh_n = 0
            self.__emit_n = 0
            self.__emit_t0 = _now

    def __ptp_to_mono(self, ts):
        """Re-express an arm-clock (PTP) device timestamp in the host CLOCK_MONOTONIC
        base so a downstream ``host_ts - device_ts`` is a real sensor->host latency, not
        a clock-domain offset. The arm firmware stamps samples on its PTP hardware clock
        (hex_device_api) while the host publishes host_ts on ``time.monotonic()`` — two
        epochs. ``ptp - monotonic`` = OFFSET - read_age <= OFFSET, so a decaying-max over
        it recovers the constant OFFSET from the least-delayed sample (scheduling delays
        only shrink the term, so the max is robust to them); the slow leak tracks PTP
        drift. Subtract OFFSET -> device_ts in the monotonic base. No-op when the arm
        falls back to a host clock (OFFSET ~ 0). Cheap: one monotonic() + a few floats."""
        ptp = self.__ts_to_float(ts)
        raw = ptp - time.monotonic()
        est = self.__ptp_mono_offset
        if est is None or raw > est:
            est = raw
        else:
            est -= self.__ptp_mono_offset_leak
        self.__ptp_mono_offset = est
        return ptp - est

    def __get_states(self) -> tuple[np.ndarray | None, dict | None]:
        if self.__arm is None:
            return None, None

        # (arm_dofs, 3) # pos vel eff
        self.__arm_state_buffer = self.__arm.get_simple_motor_status()

        # (gripper_dofs, 3) # pos vel eff
        if self.__gripper is not None:
            self.__gripper_state_buffer = self.__gripper.get_simple_motor_status(
            )

        arm_ready = self.__arm_state_buffer is not None
        gripper_ready = self.__gripper is None or self.__gripper_state_buffer is not None
        if arm_ready and gripper_ready:
            arm_ts = self.__arm_state_buffer['ts']
            gripper_ts = self.__gripper_state_buffer[
                'ts'] if self.__gripper is not None else arm_ts

            delta_ms = hex_ts_delta_ms(arm_ts, gripper_ts)
            if np.fabs(delta_ms) < self.__ts_pair_tol_ms:
                pos = self.__arm_state_buffer['pos']
                vel = self.__arm_state_buffer['vel']
                eff = self.__arm_state_buffer['eff']

                if self.__gripper is not None:
                    pos = np.concatenate(
                        [pos, self.__gripper_state_buffer['pos']])
                    vel = np.concatenate(
                        [vel, self.__gripper_state_buffer['vel']])
                    eff = np.concatenate(
                        [eff, self.__gripper_state_buffer['eff']])

                state = np.array([pos, vel, eff]).T
                self.__arm_state_buffer, self.__gripper_state_buffer = None, None
                return arm_ts if self.__sens_ts else hex_ts_now(), state
            elif delta_ms > 0.0:
                self.__gripper_state_buffer = None
                return None, None
            else:
                self.__arm_state_buffer = None
                return None, None

        return None, None

    def __set_cmds(self, cmds: np.ndarray) -> bool:
        # cmds: (n)
        # [pos_0, ..., pos_n]
        # cmds: (n, 2)
        # [[pos_0, tor_0], ..., [pos_n, tor_n]]
        # cmds: (n, 5)
        # [[pos_0, vel_0, tor_0, kp_0, kd_0], ..., [pos_n, vel_n, tor_n, kp_n, kd_n]]
        if self.__arm is None:
            print("\033[91mArm not found\033[0m")
            return False

        if cmds.shape[0] < self._dofs_sum:
            print(
                "\033[91mThe length of joint_angles must be greater than or equal to the number of motors\033[0m"
            )
            return False
        elif cmds.shape[0] > self._dofs_sum:
            print(
                f"\033[33mThe length of joint_angles is greater than the number of motors\033[0m"
            )
            cmds = cmds[:self._dofs_sum]

        cmd_pos = None
        tar_vel = np.zeros(self._dofs_sum)
        cmd_tor = np.zeros(self._dofs_sum)
        cmd_kp = self.__mit_kp.copy()
        cmd_kd = self.__mit_kd.copy()
        if len(cmds.shape) == 1:
            cmd_pos = cmds
        elif len(cmds.shape) == 2:
            if cmds.shape[1] == 2:
                cmd_pos = cmds[:, 0]
                cmd_tor = cmds[:, 1]
            elif cmds.shape[1] == 5:
                cmd_pos = cmds[:, 0]
                tar_vel = cmds[:, 1]
                cmd_tor = cmds[:, 2]
                cmd_kp = cmds[:, 3]
                cmd_kd = cmds[:, 4]
            else:
                raise ValueError(f"The shape of cmds is invalid: {cmds.shape}")
        else:
            raise ValueError(f"The shape of cmds is invalid: {cmds.shape}")

        # arm
        arm_cmd_pos = cmd_pos[self.__motor_idx["robot_arm"]]
        # Reference slew (anti-lunge), IDENTICAL to the sim device: keep the
        # commanded arm target within max_pos_err of the MEASURED angle so
        # kp*(q_des-q) stays inside the motor torque envelope — a far/stale/
        # aggressive setpoint pursues smoothly instead of saturating and lunging.
        # Applied before the joint-limit clamp (matches the sim ordering).
        if self.__max_pos_err is not None and self.__last_q is not None:
            q_meas = np.asarray(self.__last_q, dtype=np.float64)
            arm_cmd_pos = np.asarray(arm_cmd_pos, dtype=np.float64)
            if q_meas.shape == arm_cmd_pos.shape:
                arm_cmd_pos = q_meas + np.clip(arm_cmd_pos - q_meas,
                                               -self.__max_pos_err,
                                               self.__max_pos_err)
        arm_tar_pos = self._apply_pos_limits(
            arm_cmd_pos,
            self._limits[self.__motor_idx["robot_arm"], 0, 0],
            self._limits[self.__motor_idx["robot_arm"], 0, 1],
        )
        # Gravity feedforward (zero-stiffness-safe: measured pose + unity scale when
        # kp≈0) + effort/slew clamp, via the SHARED MitArmSafety so the sim device
        # does exactly the same thing. No-op for gravity unless gravity_comp is on.
        arm_tor = self.__safety.apply(
            cmd_tor[self.__motor_idx["robot_arm"]],
            arm_tar_pos, self.__last_q,
            cmd_kp[self.__motor_idx["robot_arm"]], self.__gravity_fn)
        arm_cmd = self.__arm.construct_mit_command(
            arm_tar_pos,
            tar_vel[self.__motor_idx["robot_arm"]],
            arm_tor,
            cmd_kp[self.__motor_idx["robot_arm"]],
            cmd_kd[self.__motor_idx["robot_arm"]],
        )
        self.__arm.motor_command(CommandType.MIT, arm_cmd)

        # gripper — POSITION control by default (Hands honors only POSITION; its
        # MIT/torque-feedforward inputs are ignored). For a compliant gripper
        # (gripper-joint kp≈0, e.g. zero-gravity hand-posing) relax it per
        # gripper_compliant_mode: "torque" commands zero torque so the motor goes
        # limp and backdrives by hand; "position" widens the Hands torque gate so
        # the streamed current angle tracks the hand.
        if self.__gripper is not None:
            try:
                g_idx = self.__motor_idx["robot_gripper"]
                want_compliant = bool(
                    np.all(np.asarray(cmd_kp)[g_idx] <= 1e-6))
                if want_compliant and self.__gripper_compliant_mode == "torque":
                    self.__gripper.motor_command(
                        CommandType.TORQUE, [0.0] * len(g_idx))
                    self.__gripper_gate = None  # re-apply gate when we return
                else:
                    gate = (self.__gripper_compliant_torque if want_compliant
                            else self.__gripper_default_torque)
                    if gate != self.__gripper_gate:
                        self.__gripper.set_pos_torque(gate)
                        self.__gripper_gate = gate
                    self.__gripper.motor_command(
                        CommandType.POSITION, cmd_pos[g_idx])
            except (ValueError, Exception):
                # Hands device may raise if motor data not yet available
                pass

        return True

    # ==================== control mode + safety + Cartesian impedance ==========

    def set_control_mode(self, mode: str) -> bool:
        """Runtime control-mode switch (one-shot ZMQ cmd). The modes only change how
        soda_os shapes the streamed command; the device command path is identical, so
        this just records the mode and resets the torque-slew history across the switch."""
        mode = str(mode).lower()
        if mode not in ("position", "joint_impedance", "torque"):
            print(f"\033[91m[hexarm] unknown control_mode {mode!r}\033[0m")
            return False
        self.__control_mode = mode
        self.__safety._last_tau.clear()  # reset slew history across a mode change
        hex_log(HEX_LOG_LEVEL["info"], f"[hexarm] control_mode -> {mode}")
        return True

    def clear_fault(self) -> bool:
        """Clear a latched parking-stop / fault and re-enter MIT so the arm resumes
        WITHOUT a physical power-cycle. Best-effort against the closed hex_device SDK
        (getattr-guarded); the streamed MIT commands then re-establish control."""
        if self.__arm is None:
            return False
        ok = True
        try:
            if hasattr(self.__arm, "clear_parking_stop"):
                self.__arm.clear_parking_stop()
            if hasattr(self.__arm, "enable_mit"):
                self.__arm.enable_mit()
            self.__idle_safe_active = False   # let the work loop command again
            hex_log(HEX_LOG_LEVEL["info"], "[hexarm] clear_fault -> re-enabled MIT")
        except Exception as e:
            print(f"\033[91m[hexarm] clear_fault error: {e}\033[0m")
            ok = False
        return ok

    def get_fault(self) -> np.ndarray:
        """Fault descriptor for the ZMQ buffer: float64 [active(0/1),
        remotely_clearable(0/1)]. Best-effort from the SDK status summary
        (parking_stop_detail); a parking stop is remotely clearable."""
        active, clearable = 0.0, 1.0
        try:
            if self.__arm is not None and hasattr(self.__arm, "get_status_summary"):
                s = self.__arm.get_status_summary() or {}
                active = 1.0 if s.get("parking_stop_detail") else 0.0
        except Exception:
            pass
        return np.array([active, clearable], dtype=np.float64)

    def __gravity_fn(self, q: np.ndarray) -> np.ndarray:
        """Generalized gravity at q — the pinocchio closure handed to MitArmSafety."""
        return self.__pin.computeGeneralizedGravity(
            self.__grav_model, self.__grav_data, np.asarray(q, dtype=np.float64))

    def close(self):
        if not self._working.is_set():
            return
        self._working.clear()
        if self.__state_pub is not None:
            self.__state_pub.close()
        self.__arm.stop()
        self.__hex_api.close()
        hex_log(HEX_LOG_LEVEL["info"], "HexRobotHexarm closed")
