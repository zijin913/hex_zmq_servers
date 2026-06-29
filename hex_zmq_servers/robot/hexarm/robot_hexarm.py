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
        rate = HexRate(self.__work_loop_hz)
        while self._working.is_set() and not stop_event.is_set():
            # states
            ts, states = self.__get_states()
            if states is not None:
                hold_pos = states[:, 0]
                # Cache measured arm q for the max_pos_err guard (which runs inside
                # __set_cmds and otherwise has no access to measured state).
                arm_ids = self.__motor_idx["robot_arm"]
                self.__last_q = states[arm_ids, 0]
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

    def __gravity_fn(self, q: np.ndarray) -> np.ndarray:
        """Generalized gravity at q — the pinocchio closure handed to MitArmSafety."""
        return self.__pin.computeGeneralizedGravity(
            self.__grav_model, self.__grav_data, np.asarray(q, dtype=np.float64))

    def close(self):
        if not self._working.is_set():
            return
        self._working.clear()
        self.__arm.stop()
        self.__hex_api.close()
        hex_log(HEX_LOG_LEVEL["info"], "HexRobotHexarm closed")
