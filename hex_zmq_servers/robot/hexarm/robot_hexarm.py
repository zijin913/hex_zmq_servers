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
from ..mit_control import MitArmSafety
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

        # work_loop spin rate. Default 2000Hz oversamples 4x — arm state only
        # updates at the report rate (~control_hz). That wasted spinning is a
        # pure-Python GIL hog that starves the SDK's websocket-read thread in
        # the same process, so the controller can't push frames (ENOBUFS) and
        # the arm parks. Matching control_hz frees the GIL for the read thread.
        self.__work_loop_hz = float(
            robot_config.get("work_loop_hz", 2000.0))

        # Gravity feedforward (opt-in). MIT commands here carry zero torque
        # feedforward, so position-only commands settle below target by g/kp and
        # a hold (target≈measured) droops. When enabled, add tau_g(q) computed
        # from the firefly_y6 + gr100 URDF (pinocchio) to the arm torque so the
        # arm holds its commanded pose. Benefits every client (teleop included).
        # gravity_comp_scale<1 leaves a controlled residual sag. Default off so
        # behavior is unchanged unless a config opts in.
        self.__gravity_comp = bool(robot_config.get("gravity_comp", False))
        self.__grav_scale = float(robot_config.get("gravity_comp_scale", 1.0))

        # Reference slew (anti-lunge) — IDENTICAL to the sim device so sim predicts
        # real. Cap how far the commanded arm target may lead the MEASURED angle
        # (rad); kp*(q_des-q) then stays inside the motor torque envelope, so a
        # far / stale / aggressive setpoint becomes a smooth bounded-speed pursuit
        # instead of saturating and lunging. Default 0.1 (the sim default) so the
        # real arm is protected even without an explicit cfg; <=0 disables. Only
        # binds on a far jump — dense policy/home/jog steps are well under it.
        _mpe = robot_config.get("max_pos_err", 0.1)
        self.__max_pos_err = float(_mpe) if _mpe and float(_mpe) > 0 else None

        # Control mode: "position" | "joint_impedance" | "torque" | "cart_impedance".
        # JOINT modes (position/joint_impedance/torque) are decided purely by the
        # command column-count in __set_cmds, so the device needs no flag for them —
        # soda_os shapes the 5-col MIT command. The flag exists so the device can
        # switch INTO cart_impedance, where it must reinterpret the streamed payload
        # as a Cartesian reference and run the task-space PD itself at the loop rate.
        self.__control_mode = str(
            robot_config.get("control_mode", "position")).lower()

        # Device-side torque safety (applies to EVERY mode's torque feedforward,
        # closest to the hardware). effort_limit clamps |tau| per arm joint; tau_slew
        # caps the per-tick change (anti-step); cart_force_clamp bounds the Cartesian
        # wrench. All opt-in — absent → no software clamp (legacy behaviour); the
        # motor firmware still enforces its own hardware current limit underneath.
        ts = robot_config.get("torque_safety", {}) or {}
        self.__cart_f_clamp = float(ts.get("cart_force_clamp", 150.0))
        # Rotational wrench ceiling (N·m): bounds ‖F[3:6]‖ (the orientation-error term
        # that drives the WRIST via Jᵀ). Default finite (was unbounded) so an orientation
        # error can't drive the wrist to the effort cap; lower for commissioning.
        self.__cart_t_clamp = float(ts.get("cart_torque_clamp", 30.0))
        # Small FIRMWARE-LOCAL joint damping for cart_impedance (kp stays 0). In cart the
        # arm joints get kp=kd=0, so the ONLY damping is the software task-space D term,
        # which on real feeds back the raw/quantized, transport-DELAYED joint velocity →
        # the low-inertia wrist (J5/J6) breaks into a high-frequency limit cycle. A small
        # kd in the MIT command closes the damping in the FIRMWARE at its own high rate
        # with ~0 delay (guaranteed passive), killing the buzz without adding stiffness.
        # Per-joint; tune UP the wrist entries only until the buzz stops. None/0 disables.
        self.__cart_joint_kd = np.asarray(
            ts.get("cart_joint_kd", [0.5, 0.5, 0.5, 0.3, 0.2, 0.2]), dtype=np.float64)
        # One-pole low-pass on the joint velocity feeding the cart D term: high-frequency
        # velocity-estimate NOISE through the (delayed) software D term keeps re-exciting
        # the wrist limit cycle (the residual buzz firmware kd alone leaves). alpha∈[0,1):
        # higher = smoother but more phase lag (too high re-destabilizes). 0.8 ≈ 18 Hz @500Hz.
        self.__cart_dq_alpha = float(ts.get("cart_dq_filter_alpha", 0.8))
        self.__cart_dq_filt = None
        # SOFTWARE joint-space damping (the openarm-follower pattern): instead of a firmware
        # kd on the RAW motor velocity (a noise amplifier on the light wrist) OR task-space
        # Jᵀ·D·J (ill-conditioned at the wrist), subtract a per-joint B on the FILTERED,
        # deadbanded velocity here in software. Pair with cart_joint_kd=0 and rot task-space
        # D≈0 so this is the only damping. Robot-specific — tune; these are openarm starts.
        self.__cart_joint_b = np.asarray(
            ts.get("cart_joint_b", [4.0, 3.8, 2.6, 2.8, 0.3, 0.3]), dtype=np.float64)
        # Velocity deadband (rad/s): zero the damping/D velocity below this so quantization
        # NOISE at rest can't drive a damping torque (kills the at-rest buzz). 0 disables.
        self.__cart_vel_deadband = float(ts.get("cart_vel_deadband", 0.02))
        # rank 5: run the Cartesian impedance LAW at control_hz inside work_loop against the
        # freshest measured q/dq, instead of only when a (slow, jittery) command arrives or the
        # 30Hz idle-hold fires. At a low/delayed recompute rate the software damping term is NOT
        # passive and injects energy (lunge); at control_hz it is. The streamed cart reference
        # only updates this cached TARGET; the law runs at loop rate.
        self.__last_cart_ref = None
        self.__cart_control_period_ms = 1000.0 / max(1.0, float(control_hz))
        # One-shot: on entering cart_impedance, force the FIRST cart tick's error to 0 so
        # the device holds its OWN measured pose (independent of any skew in the soda-side
        # bumpless reference); subsequent ticks track the streamed reference.
        self.__cart_entry_seed = False
        # Shared MIT-arm safety (gravity feedforward + effort/slew clamp). Built from
        # the SAME config keys (incl. gravity_comp_scale_lowstiff) via from_config so
        # the sim and real devices read everything identically — the sim is a faithful
        # pre-flight test. (robot/mit_control.py)
        self.__safety = MitArmSafety.from_config(robot_config)

        # Latest measured arm state (cached by work_loop) — the Cartesian PD needs
        # measured q/dq, which __set_cmds doesn't otherwise receive.
        self.__last_q = None
        self.__last_dq = None
        self.__ee_frame_id = None

        # Pinocchio model — needed for gravity feedforward AND/OR Cartesian impedance.
        self.__pin = None
        self.__grav_model = None
        self.__grav_data = None
        need_pin = self.__gravity_comp or self.__control_mode == "cart_impedance"
        if need_pin:
            try:
                import pinocchio as pin
                urdf = os.path.join(os.path.dirname(__file__),
                                    "urdf", "firefly_y6", "gr100.urdf")
                self.__pin = pin
                self.__grav_model = pin.buildModelFromUrdf(urdf)
                self.__grav_data = self.__grav_model.createData()
                self.__grav_model.gravity.linear = np.array([0.0, 0.0, -9.81])
                self.__ee_frame_id = self.__resolve_ee_frame()
                hex_log(HEX_LOG_LEVEL["info"],
                        f"[hexarm] pinocchio ON (gravity_comp={self.__gravity_comp} "
                        f"scale={self.__grav_scale}, control_mode={self.__control_mode})")
            except Exception as e:
                print(f"\033[91m[hexarm] pinocchio init failed: {e}\033[0m")
                self.__gravity_comp = False
                if self.__control_mode == "cart_impedance":
                    print("\033[91m[hexarm] cart_impedance needs pinocchio — "
                          "falling back to position\033[0m")
                    self.__control_mode = "position"

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
        last_cart_ts = hex_ts_now()   # rank 5: throttle the high-rate cart recompute to control_hz
        rate = HexRate(self.__work_loop_hz)
        while self._working.is_set() and not stop_event.is_set():
            # states
            ts, states = self.__get_states()
            if states is not None:
                hold_pos = states[:, 0]
                # Cache measured arm q/dq for the Cartesian-impedance PD (which runs
                # inside __set_cmds and otherwise has no access to measured state).
                arm_ids = self.__motor_idx["robot_arm"]
                self.__last_q = states[arm_ids, 0]
                self.__last_dq = states[arm_ids, 1]
                if hex_ts_delta_ms(ts, last_states_ts) > 1e-6:
                    last_states_ts = ts
                    states_queue.append((ts, states_count, states))
                    states_count = (states_count + 1) % self._max_seq_num

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
                    # 命令时间戳新鲜度校验已移除:始终下发(see fork history)
                    # Never let a malformed command kill the control loop / server.
                    try:
                        self.__set_cmds(cmds)
                    except Exception as e:
                        print(f"\033[91m[hexarm] set_cmds error: {e}\033[0m")
                    last_send_ts = hex_ts_now()
                    sent = True

            # idle-hold keep-alive — feed the firmware's API watchdog when the
            # client command stream has a gap, so the arm holds position instead
            # of parking (PscApiCommunicationTimeout) and churning the SDK into a
            # native crash. Throttled to idle_hold_hz. Re-send the last real
            # command if there was one; otherwise hold a *latched* pose captured
            # once when the arm went idle. NOTE: never feed the live measured
            # pose here — __set_cmds issues an MIT/impedance command with no
            # gravity feedforward, so target==measured every tick means ~zero
            # restoring torque and the arm droops under gravity, chasing its own
            # sag downward. A fixed target builds real kp*(target-measured)
            # torque and holds.
            if self.__idle_hold and not sent and hex_ts_delta_ms(
                    hex_ts_now(), last_send_ts) >= self.__idle_hold_period_ms:
                if last_cmds is not None:
                    hold = last_cmds
                else:
                    if idle_target is None and hold_pos is not None:
                        idle_target = hold_pos.copy()   # latch once
                    hold = idle_target
                if hold is not None:
                    try:
                        self.__set_cmds(hold)
                    except Exception as e:
                        print(f"\033[91m[hexarm] idle-hold set_cmds error: {e}\033[0m")
                    last_send_ts = hex_ts_now()

            # sleep
            rate.sleep()

        # close
        self.close()

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
            if np.fabs(delta_ms) < 1e-6:
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

        # Cartesian impedance: the streamed payload is a task-space reference
        # (pose + Cartesian K/D + grip), NOT joint targets — handle on its own path
        # before the joint-length check below would mangle it.
        if self.__control_mode == "cart_impedance" and self.__pin is not None:
            return self.__set_cmds_cart(cmds)

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
        """Runtime control-mode switch (one-shot ZMQ cmd). JOINT modes only change
        how soda_os shapes the streamed command (the device path is the same);
        cart_impedance makes the device run the task-space PD and needs pinocchio,
        which is built at init only when configured — so it can't be entered at
        runtime unless the device started with pinocchio available."""
        mode = str(mode).lower()
        if mode not in ("position", "joint_impedance", "torque", "cart_impedance"):
            print(f"\033[91m[hexarm] unknown control_mode {mode!r}\033[0m")
            return False
        if mode == "cart_impedance" and self.__pin is None:
            print("\033[91m[hexarm] cart_impedance needs pinocchio — start the server "
                  "with control_mode/gravity_comp so it is built at init\033[0m")
            return False
        self.__control_mode = mode
        self.__safety._last_tau.clear()  # reset slew history across a mode change
        # ...EXCEPT seed the cart torque-slew baseline, or the FIRST cart tick bypasses
        # the rate limiter: MitArmSafety.clamp only slews when a prior value exists, and
        # cart uses an ISOLATED slew_key="cart" that the joint path (slew_key="default")
        # never seeds. An empty bucket let the first cart tick dump the full Jᵀ·F+gravity
        # torque in ONE 500Hz tick (bounded only by the per-joint effort cap, not by
        # slew_Nm_per_tick) — a violent wrist lunge ("爆冲") on entry. Baseline it to the
        # gravity the cart loop already applies so the first step is just the (≈0 at a
        # bumpless entry) spring force and RAMPS from there. (Sim never lunged because it
        # reuses one slew key across modes and doesn't clear — this restores that parity.)
        if mode == "cart_impedance":
            self.__cart_entry_seed = True  # first cart tick holds the device's own pose (err=0)
            self.__cart_dq_filt = None     # re-seed the velocity low-pass on entry
            self.__last_cart_ref = None    # wait for the fresh bumpless ref before the high-rate loop runs
            try:
                if self.__last_q is not None and self.__pin is not None:
                    seed = min(self.__grav_scale, 1.0) * self.__gravity_fn(self.__last_q)
                    self.__safety._last_tau["cart"] = np.asarray(seed, dtype=np.float64)
                else:
                    self.__safety._last_tau["cart"] = np.zeros(6)
            except Exception as e:
                print(f"\033[91m[hexarm] cart slew seed failed: {e}\033[0m")
                self.__safety._last_tau["cart"] = np.zeros(6)
        hex_log(HEX_LOG_LEVEL["info"], f"[hexarm] control_mode -> {mode}")
        return True

    def __resolve_ee_frame(self):
        """Frame id for the Cartesian FK/Jacobian. Prefer the arm's last link
        (link_6); fall back to the model's last frame."""
        m = self.__grav_model
        for name in ("link_6", "link6", "tool0", "ee_link", "gripper_base", "tcp"):
            try:
                if m.existFrame(name):
                    return m.getFrameId(name)
            except Exception:
                pass
        return m.nframes - 1

    def __gravity_fn(self, q: np.ndarray) -> np.ndarray:
        """Generalized gravity at q — the pinocchio closure handed to MitArmSafety."""
        return self.__pin.computeGeneralizedGravity(
            self.__grav_model, self.__grav_data, np.asarray(q, dtype=np.float64))

    @staticmethod
    def __quat_wxyz_to_mat(quat: np.ndarray) -> np.ndarray:
        w, x, y, z = quat / (np.linalg.norm(quat) + 1e-12)
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def __set_cmds_cart(self, cmds: np.ndarray) -> bool:
        """Cartesian impedance for ONE arm. Streamed payload (length >= 20):
            [px,py,pz, qw,qx,qy,qz, Kx,Ky,Kz,Krx,Kry,Krz, Dx,Dy,Dz,Drx,Dry,Drz, grip]
        tau = Jᵀ(K·Δx + D·(−ẋ)) + gravity, sent as an MIT command with kp=kd=0 so the
        computed torque is the only thing driving the arm. Orientation error uses the
        SO(3) log-map. Jacobian is LOCAL_WORLD_ALIGNED (EE point, base-aligned axes),
        matching the cart_vel adapter.

        ⚠️ UNVALIDATED on hardware — validate in sim, then on a real arm under e-stop
        with low K, before trusting it.
        """
        if self.__last_q is None or self.__pin is None:
            return False
        ref = np.asarray(cmds, dtype=np.float64).reshape(-1)
        if ref.shape[0] < 20:
            print(f"\033[91m[hexarm] cart cmd needs >=20 values, got {ref.shape[0]}\033[0m")
            return False
        pos_des, quat_des, K, D, grip = ref[0:3], ref[3:7], ref[7:13], ref[13:19], ref[19]
        q = np.asarray(self.__last_q, dtype=np.float64)
        dq = (np.asarray(self.__last_dq, dtype=np.float64)
              if self.__last_dq is not None else np.zeros_like(q))
        pin, m, d, fid = self.__pin, self.__grav_model, self.__grav_data, self.__ee_frame_id
        try:
            pin.forwardKinematics(m, d, q)
            pin.updateFramePlacements(m, d)
            x_cur = np.asarray(d.oMf[fid].translation)
            R_cur = np.asarray(d.oMf[fid].rotation)
            J = np.asarray(pin.computeFrameJacobian(
                m, d, q, fid, pin.LOCAL_WORLD_ALIGNED))[:, :q.shape[0]]  # 6 x n
            R_des = self.__quat_wxyz_to_mat(quat_des)
            if self.__cart_entry_seed:
                # First tick after entering cart: hold THIS pose exactly (err=0), so any
                # skew between the soda-side bumpless reference and the device's measured
                # pose can't produce an entry wrench. Subsequent ticks track the stream.
                err = np.zeros(6)
                self.__cart_entry_seed = False
            else:
                err = np.concatenate([pos_des - x_cur, pin.log3(R_des @ R_cur.T)])  # 6
            # Low-pass the joint velocity feeding the (delayed, noisy) software D term so
            # velocity-estimate noise can't sustain the wrist limit cycle; the firmware kd
            # dissipates, this stops the re-excitation.
            a = self.__cart_dq_alpha
            if self.__cart_dq_filt is None or self.__cart_dq_filt.shape != dq.shape:
                self.__cart_dq_filt = dq.copy()
            else:
                self.__cart_dq_filt = a * self.__cart_dq_filt + (1.0 - a) * dq
            # Deadband the filtered velocity so at-rest quantization noise drives no damping.
            vf = self.__cart_dq_filt.copy()
            vf[np.abs(vf) < self.__cart_vel_deadband] = 0.0
            xdot = J @ vf                                                        # 6
            F = K * err - D * xdot                                               # diag K/D
            fmag = np.linalg.norm(F[:3])
            if fmag > self.__cart_f_clamp:
                F[:3] *= self.__cart_f_clamp / fmag
            # Rotational wrench clamp: bound the orientation-error term that drives the
            # wrist via Jᵀ, so a spurious/large orientation error can't spike the wrist.
            tmag = np.linalg.norm(F[3:6])
            if tmag > self.__cart_t_clamp:
                F[3:6] *= self.__cart_t_clamp / tmag
            # gravity (always, at measured q, scale<=1.0 since kp=0) + shared clamp.
            grav = min(self.__grav_scale, 1.0) * pin.computeGeneralizedGravity(m, d, q)
            # Damping = SOFTWARE joint-space B on the filtered/deadbanded velocity (NOT a
            # firmware kd on raw velocity, NOT task-space Jᵀ D J on the light wrist) — the
            # openarm-follower pattern that keeps the wrist quiet.
            arm_tor = self.__safety.clamp(
                J.T @ F + grav - self.__cart_joint_b[:q.shape[0]] * vf, slew_key="cart")
        except Exception as e:
            print(f"\033[91m[hexarm] cart impedance failed: {e}\033[0m")
            return False
        zeros = np.zeros(q.shape[0])
        kd_cart = self.__cart_joint_kd[:q.shape[0]]  # firmware-local damping (kp stays 0)
        self.__arm.motor_command(
            CommandType.MIT,
            self.__arm.construct_mit_command(q, zeros, arm_tor, zeros, kd_cart))
        if self.__gripper is not None:
            try:
                g_idx = self.__motor_idx["robot_gripper"]
                if self.__gripper_default_torque != self.__gripper_gate:
                    self.__gripper.set_pos_torque(self.__gripper_default_torque)
                    self.__gripper_gate = self.__gripper_default_torque
                self.__gripper.motor_command(CommandType.POSITION, [float(grip)] * len(g_idx))
            except (ValueError, Exception):
                pass
        return True

    def close(self):
        if not self._working.is_set():
            return
        self._working.clear()
        self.__arm.stop()
        self.__hex_api.close()
        hex_log(HEX_LOG_LEVEL["info"], "HexRobotHexarm closed")
