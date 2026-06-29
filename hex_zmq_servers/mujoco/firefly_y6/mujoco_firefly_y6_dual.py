#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Dual-arm Archer L6Y MuJoCo simulation device.
# Based on HexMujocoFireflyY6 (single-arm) and HexMujocoE3Desktop (dual-arm pattern).
################################################################

import os
import copy
import threading
import cv2
import numpy as np
from collections import deque

import mujoco
from mujoco import viewer

from ..mujoco_base import HexMujocoBase
from ...zmq_base import (
    hex_ns_now,
    hex_zmq_ts_now,
    ns_to_hex_zmq_ts,
    hex_zmq_ts_delta_ms,
    HexRate,
)
from ...hex_launch import hex_log, HEX_LOG_LEVEL
from ...robot.mit_control import MitArmSafety
from hex_robo_utils import HexCtrlUtilMitJoint as CtrlUtil

MUJOCO_CONFIG = {
    "states_rate": 1000,
    "img_rate": 30,
    "tau_ctrl": False,
    "mit_kp": [200.0, 200.0, 200.0, 75.0, 15.0, 15.0, 20.0],
    "mit_kd": [12.5, 12.5, 12.5, 6.0, 0.31, 0.31, 1.0],
    "cam_type": ["realsense", "realsense"],  # [left, right]
    "headless": False,
    "sens_ts": True,
}
CAMERA_CONFIG = {
    "empty": (False, False),
    "rgb": (True, False),
    "berxel": (True, True),
    "realsense": (True, True),
}


class HexMujocoFireflyY6Dual(HexMujocoBase):

    def __init__(
        self,
        mujoco_config: dict = MUJOCO_CONFIG,
        realtime_mode: bool = False,
    ):
        HexMujocoBase.__init__(self, realtime_mode)

        try:
            states_rate = mujoco_config["states_rate"]
            img_rate = mujoco_config["img_rate"]
            self.__tau_ctrl = mujoco_config["tau_ctrl"]
            self.__mit_kp = mujoco_config["mit_kp"]
            self.__mit_kd = mujoco_config["mit_kd"]
            self.__cam_type = mujoco_config["cam_type"]
            self.__headless = mujoco_config["headless"]
            self.__sens_ts = mujoco_config["sens_ts"]
        except KeyError as ke:
            missing_key = ke.args[0]
            raise ValueError(
                f"mujoco_config is not valid, missing key: {missing_key}")

        # mujoco init
        scene_name = mujoco_config.get("scene_name", "scene_dual")
        model_path = os.path.join(os.path.dirname(__file__),
                                  f"model/{scene_name}.xml")
        self.__model = mujoco.MjModel.from_xml_path(model_path)
        self.__data = mujoco.MjData(self.__model)
        self.__sim_rate = int(1.0 / self.__model.opt.timestep)

        # State indices — Firefly Y6 dual: 8 qpos per arm (6 arm + 2 gripper claws via equality)
        # Left arm: joint_1-6 at qpos 0-5, gripper_left_joint_1 (driven) at qpos 6, gripper_right_joint_1 (mimic) at qpos 7
        # Right arm: same layout offset by 8 → qpos 8-15
        # Objects start at qpos 16 (6 objects × 7 qpos per free joint = 42)
        self.__state_left_idx = [0, 1, 2, 3, 4, 5, 6]
        self.__state_right_idx = [8, 9, 10, 11, 12, 13, 14]
        self.__obj_pose_idx = [16, 17, 18, 19, 20, 21, 22]
        self.__ctrl_left_idx = [0, 1, 2, 3, 4, 5, 6]
        self.__ctrl_right_idx = [7, 8, 9, 10, 11, 12, 13]
        self._limits = np.stack(
            [
                self.__model.jnt_range[self.__state_left_idx, :],
                self.__model.jnt_range[self.__state_right_idx, :],
            ],
            axis=0,
        )

        # Sim-plant joint dynamics (rotor inertia + friction) — CONFIG-DRIVEN ESTIMATES.
        # These make the sim RESPOND to torque like the real arm (impedance overshoot/
        # settling, back-drive feel); the real device has no equivalent (its firmware PD
        # acts on the physical joint). They are physically-motivated ESTIMATES until the
        # real values are obtained from CAD / HEXFELLOW (zero-risk path) — see the json
        # _comment_joint_dyn. Override here so a future real value is a one-line json edit,
        # no MJCF surgery. armature_i = J_rotor * gear_i^2 (reflected rotor inertia).
        # Applied to both arms' 6 joint DOFs (hinges: dof idx == qpos idx); gripper kept.
        def _override_dof(cfg_key, model_attr):
            vals = mujoco_config.get(cfg_key)
            if not vals:
                return
            arr = getattr(self.__model, model_attr)
            for idx in (self.__state_left_idx[:6], self.__state_right_idx[:6]):
                for k in range(min(6, len(vals))):
                    arr[idx[k]] = float(vals[k])
        _override_dof("arm_armature", "dof_armature")
        _override_dof("arm_frictionloss", "dof_frictionloss")
        _override_dof("arm_damping", "dof_damping")

        # Per-joint effort cap (Nm). Defaults to the MJCF motor forcerange so the
        # software clamp matches the plant: a far/aggressive setpoint can no longer
        # demand an unbounded PD torque. Also fed to MitArmSafety via torque_safety.
        _ts = mujoco_config.get("torque_safety", {}) or {}
        _eff = _ts.get("effort_limit_Nm")
        self.__effort_limit = (np.asarray(_eff, dtype=np.float64)
                               if _eff is not None else None)
        if not self.__tau_ctrl:
            self.__mit_kp = np.ascontiguousarray(np.asarray(self.__mit_kp))
            self.__mit_kd = np.ascontiguousarray(np.asarray(self.__mit_kd))
            # Clamp the PD output (kp*err + kd*derr) BEFORE the gravity feedforward
            # is added, so an aggressive setpoint can't saturate-and-lunge.
            self.__mit_ctrl = CtrlUtil(self.__effort_limit)

        # Control mode (sim parity with the real device). Joint modes ride the
        # existing 5-col MIT path; cart_impedance runs a task-space PD via pinocchio
        # (same gr100 URDF + math as the real arm) so sim/real behave identically.
        self.__control_mode = str(mujoco_config.get("control_mode", "position")).lower()
        # Cartesian wrench clamp (N). Read from torque_safety.cart_force_clamp to match
        # the REAL device (robot_hexarm reads it from there); fall back to the legacy
        # top-level key, then 150.0. Keeps the two plants reading the SAME knob.
        _ts0 = mujoco_config.get("torque_safety", {}) or {}
        self.__cart_f_clamp = float(_ts0.get("cart_force_clamp",
                                    mujoco_config.get("cart_force_clamp", 150.0)))
        # Rotational wrench ceiling (N·m) — bounds ‖F[3:6]‖ (orientation-error term that
        # drives the wrist). Matches the REAL device (robot_hexarm cart_torque_clamp).
        self.__cart_t_clamp = float(_ts0.get("cart_torque_clamp",
                                    mujoco_config.get("cart_torque_clamp", 30.0)))
        # One-shot: first cart tick holds the device's own pose (err=0) — parity with real.
        self.__cart_entry_seed = False
        # Small firmware-local joint kd for cart_impedance (kp stays 0) — parity with the
        # REAL device, which needs it to kill the delayed-velocity-feedback wrist buzz.
        # Harmless in sim (clean velocity), kept identical so the sim predicts real.
        self.__cart_joint_kd = np.asarray(
            _ts0.get("cart_joint_kd", [0.5, 0.5, 0.5, 0.3, 0.2, 0.2]), dtype=np.float64)
        # Velocity low-pass for the cart D term — parity with real (harmless here, qvel is clean).
        self.__cart_dq_alpha = float(_ts0.get("cart_dq_filter_alpha", 0.8))
        self.__cart_dq_filt = None
        # Software joint-space damping B + velocity deadband (openarm-follower pattern) —
        # parity with real. All cart damping is this B on the filtered/deadbanded velocity.
        self.__cart_joint_b = np.asarray(
            _ts0.get("cart_joint_b", [4.0, 3.8, 2.6, 2.8, 0.3, 0.3]), dtype=np.float64)
        self.__cart_vel_deadband = float(_ts0.get("cart_vel_deadband", 0.02))
        self.__gravity_comp = bool(mujoco_config.get("gravity_comp", False))
        # Reference slew: max distance (rad) the COMMANDED arm-joint target may sit
        # ahead of the MEASURED angle. Bounds kp*(q_des-q) below the motor force
        # limit (28/200 = 0.14 rad saturates J1-3), turning a far setpoint — a big
        # jog, a stale target, an aggressive waypoint — into a smooth bounded-speed
        # pursuit instead of a torque-saturating lunge. <=0 disables it. Transparent
        # to dense policy/home trajectories (their per-tick step is far smaller).
        _mpe = mujoco_config.get("max_pos_err", 0.1)
        self.__max_pos_err = float(_mpe) if _mpe and float(_mpe) > 0 else None
        # Shared MIT-arm safety — the SAME clamp + zero-stiffness logic the real device
        # runs (robot/mit_control.py). The gravity FEEDFORWARD, however, comes from
        # MuJoCo itself in sim (see __mj_gravity), because the plant here is MuJoCo and
        # the gr100 URDF model doesn't match its MJCF inertia.
        self.__safety = MitArmSafety.from_config(mujoco_config)
        self.__pin = None
        self.__pin_model = None
        self.__pin_data = None
        self.__ee_fid = None
        if self.__gravity_comp or self.__control_mode == "cart_impedance":
            self.__init_pin()
        # GR100 lobster claw: client commands and URDF/MJCF range are now both [0, 0.69]
        # (measured upper limit). Keep ratio=1.0 — no scaling. If you ever want to
        # advertise a smaller range than URDF allows, set ratio = (advertised / urdf_max).
        self.__gripper_ratio = 1.0
        self._limits[0, -1] *= self.__gripper_ratio
        self._limits[1, -1] *= self.__gripper_ratio
        self._dofs = np.array([
            len(self.__state_left_idx),
            len(self.__state_right_idx),
        ])
        keyframe_id = mujoco.mj_name2id(
            self.__model,
            mujoco.mjtObj.mjOBJ_KEY,
            "home",
        )
        self.__state_init = {
            "qpos": self.__model.key_qpos[keyframe_id],
            "qvel": np.zeros_like(self.__data.qvel),
            "ctrl": np.zeros_like(self.__data.ctrl),
        }
        self.__data.qpos = self.__state_init["qpos"]
        self.__data.qvel = self.__state_init["qvel"]
        self.__data.ctrl = self.__state_init["ctrl"]
        self.__states_trig_thresh = int(self.__sim_rate / states_rate)

        # Camera init — left and right wrist cameras
        self.__img_trig_thresh = int(self.__sim_rate / img_rate)
        self.__width, self.__height = 640, 400
        left_cam_id = mujoco.mj_name2id(self.__model,
                                         mujoco.mjtObj.mjOBJ_CAMERA,
                                         "left_end_camera")
        right_cam_id = mujoco.mj_name2id(self.__model,
                                          mujoco.mjtObj.mjOBJ_CAMERA,
                                          "right_end_camera")
        left_fovy_rad = self.__model.cam_fovy[left_cam_id] * np.pi / 180.0
        right_fovy_rad = self.__model.cam_fovy[right_cam_id] * np.pi / 180.0
        left_focal = 0.5 * self.__height / np.tan(left_fovy_rad / 2.0)
        right_focal = 0.5 * self.__height / np.tan(right_fovy_rad / 2.0)
        self._intri = np.array([
            [left_focal, left_focal, self.__width / 2, self.__height / 2],
            [right_focal, right_focal, self.__width / 2, self.__height / 2],
        ])

        self.__left_rgb, self.__left_depth = CAMERA_CONFIG.get(
            self.__cam_type[0], (False, False))
        self.__right_rgb, self.__right_depth = CAMERA_CONFIG.get(
            self.__cam_type[1], (False, False))
        self.__rgb_cam, self.__depth_cam = None, None
        has_rgb = self.__left_rgb or self.__right_rgb
        has_depth = self.__left_depth or self.__right_depth
        if has_rgb:
            self.__rgb_cam = mujoco.Renderer(self.__model, self.__height,
                                             self.__width)
        if has_depth:
            self.__depth_cam = mujoco.Renderer(self.__model, self.__height,
                                               self.__width)
            self.__depth_cam.enable_depth_rendering()

        # Side camera init (fixed third-person view)
        self.__side_width, self.__side_height = 640, 400
        self.__side_rgb_cam, self.__side_depth_cam = None, None
        self.__use_side_cam = False
        self._side_intri = None
        side_cam_id = mujoco.mj_name2id(self.__model,
                                         mujoco.mjtObj.mjOBJ_CAMERA,
                                         "side_camera")
        if side_cam_id >= 0:
            side_fovy_rad = self.__model.cam_fovy[side_cam_id] * np.pi / 180.0
            side_focal = 0.5 * self.__side_height / np.tan(
                side_fovy_rad / 2.0)
            self._side_intri = np.array([
                side_focal, side_focal, self.__side_width / 2,
                self.__side_height / 2
            ])
            if has_rgb:
                self.__side_rgb_cam = mujoco.Renderer(self.__model,
                                                      self.__side_height,
                                                      self.__side_width)
            if has_depth:
                self.__side_depth_cam = mujoco.Renderer(
                    self.__model, self.__side_height, self.__side_width)
                self.__side_depth_cam.enable_depth_rendering()
            self.__use_side_cam = True

        # Viewer init
        mujoco.mj_forward(self.__model, self.__data)
        if not self.__headless:
            self.__viewer = viewer.launch_passive(self.__model, self.__data)

        # Start work loop
        self._working.set()

    def __del__(self):
        HexMujocoBase.__del__(self)

    def reset(self) -> bool:
        self.__data.qpos = self.__state_init["qpos"]
        self.__data.qvel = self.__state_init["qvel"]
        self.__data.ctrl = self.__state_init["ctrl"]
        mujoco.mj_forward(self.__model, self.__data)
        if not self.__headless:
            self.__viewer.sync()
        return True

    def work_loop(self, hex_queues: list[deque | threading.Event]):
        states_left_queue = hex_queues[0]
        states_right_queue = hex_queues[1]
        states_obj_queue = hex_queues[2]
        cmds_left_queue = hex_queues[3]
        cmds_right_queue = hex_queues[4]
        left_rgb_queue = hex_queues[5]
        left_depth_queue = hex_queues[6]
        right_rgb_queue = hex_queues[7]
        right_depth_queue = hex_queues[8]
        side_rgb_queue = hex_queues[9]
        side_depth_queue = hex_queues[10]
        stop_event = hex_queues[11]

        last_states_ts = {"s": 0, "ns": 0}
        states_left_count = 0
        states_right_count = 0
        states_obj_count = 0
        last_cmds_left_seq = -1
        last_cmds_right_seq = -1
        left_rgb_count = 0
        left_depth_count = 0
        right_rgb_count = 0
        right_depth_count = 0
        side_rgb_count = 0
        side_depth_count = 0
        cmds_left = None
        cmds_right = None

        rate = HexRate(self.__sim_rate)
        states_trig_count = 0
        img_trig_count = 0
        side_img_trig_count = 0
        side_img_trig_thresh = self.__img_trig_thresh * 3  # side cam at ~10Hz
        self.__bias_ns = hex_ns_now() - self.__data.time * 1_000_000_000
        init_ts = self.__mujoco_ts() if self.__sens_ts else hex_zmq_ts_now()
        # Initial placeholder frames
        empty_rgb = np.zeros((self.__height, self.__width, 3), dtype=np.uint8)
        empty_depth = np.zeros((self.__height, self.__width), dtype=np.uint16)
        left_rgb_queue.append((init_ts, 0, empty_rgb.copy()))
        left_depth_queue.append((init_ts, 0, empty_depth.copy()))
        right_rgb_queue.append((init_ts, 0, empty_rgb.copy()))
        right_depth_queue.append((init_ts, 0, empty_depth.copy()))
        if self.__use_side_cam:
            side_empty_rgb = np.zeros(
                (self.__side_height, self.__side_width, 3), dtype=np.uint8)
            side_empty_depth = np.zeros(
                (self.__side_height, self.__side_width), dtype=np.uint16)
            side_rgb_queue.append((init_ts, 0, side_empty_rgb))
            side_depth_queue.append((init_ts, 0, side_empty_depth))

        while self._working.is_set() and not stop_event.is_set():
            states_trig_count += 1
            if states_trig_count >= self.__states_trig_thresh:
                states_trig_count = 0

                # States
                ts, states_left, states_right, states_obj = self.__get_states()
                if states_left is not None:
                    if hex_zmq_ts_delta_ms(ts, last_states_ts) > 1e-6:
                        last_states_ts = ts
                        states_left_queue.append(
                            (ts, states_left_count, states_left))
                        states_left_count = (states_left_count +
                                             1) % self._max_seq_num
                        states_right_queue.append(
                            (ts, states_right_count, states_right))
                        states_right_count = (states_right_count +
                                              1) % self._max_seq_num
                        states_obj_queue.append(
                            (ts, states_obj_count, states_obj))
                        states_obj_count = (states_obj_count +
                                            1) % self._max_seq_num

                # Commands — left
                cmds_left_pack = None
                try:
                    cmds_left_pack = cmds_left_queue[
                        -1] if self._realtime_mode else cmds_left_queue.popleft(
                        )
                except IndexError:
                    pass
                if cmds_left_pack is not None:
                    ts, seq, cmds_left_get = cmds_left_pack
                    if seq != last_cmds_left_seq:
                        last_cmds_left_seq = seq
                        if hex_zmq_ts_delta_ms(hex_zmq_ts_now(), ts) < 200.0:
                            cmds_left = cmds_left_get.copy()
                if cmds_left is not None:
                    try:
                        self.__set_cmds(cmds_left, "left")
                    except Exception as e:
                        print(f"\033[91m[mujoco_dual] set_cmds(left) error: {e}\033[0m")

                # Commands — right
                cmds_right_pack = None
                try:
                    cmds_right_pack = cmds_right_queue[
                        -1] if self._realtime_mode else cmds_right_queue.popleft(
                        )
                except IndexError:
                    pass
                if cmds_right_pack is not None:
                    ts, seq, cmds_right_get = cmds_right_pack
                    if seq != last_cmds_right_seq:
                        last_cmds_right_seq = seq
                        if hex_zmq_ts_delta_ms(hex_zmq_ts_now(), ts) < 200.0:
                            cmds_right = cmds_right_get.copy()
                if cmds_right is not None:
                    try:
                        self.__set_cmds(cmds_right, "right")
                    except Exception as e:
                        print(f"\033[91m[mujoco_dual] set_cmds(right) error: {e}\033[0m")

            img_trig_count += 1
            if img_trig_count >= self.__img_trig_thresh:
                img_trig_count = 0

                # Left camera
                if self.__left_rgb:
                    ts, rgb_img = self.__get_rgb("left_end_camera")
                    if rgb_img is not None:
                        left_rgb_queue.append((ts, left_rgb_count, rgb_img))
                        left_rgb_count = (left_rgb_count +
                                          1) % self._max_seq_num
                if self.__left_depth:
                    ts, depth_img = self.__get_depth("left_end_camera")
                    if depth_img is not None:
                        left_depth_queue.append(
                            (ts, left_depth_count, depth_img))
                        left_depth_count = (left_depth_count +
                                            1) % self._max_seq_num

                # Right camera
                if self.__right_rgb:
                    ts, rgb_img = self.__get_rgb("right_end_camera")
                    if rgb_img is not None:
                        right_rgb_queue.append((ts, right_rgb_count, rgb_img))
                        right_rgb_count = (right_rgb_count +
                                           1) % self._max_seq_num
                if self.__right_depth:
                    ts, depth_img = self.__get_depth("right_end_camera")
                    if depth_img is not None:
                        right_depth_queue.append(
                            (ts, right_depth_count, depth_img))
                        right_depth_count = (right_depth_count +
                                             1) % self._max_seq_num

            # Side camera at lower rate
            side_img_trig_count += 1
            if self.__use_side_cam and side_img_trig_count >= side_img_trig_thresh:
                side_img_trig_count = 0
                if self.__left_rgb or self.__right_rgb:
                    ts, side_rgb_img = self.__get_rgb("side_camera")
                    if side_rgb_img is not None:
                        side_rgb_queue.append(
                            (ts, side_rgb_count, side_rgb_img))
                        side_rgb_count = (side_rgb_count +
                                          1) % self._max_seq_num
                if (self.__left_depth
                        or self.__right_depth) and self.__side_depth_cam:
                    ts, side_depth_img = self.__get_depth("side_camera")
                    if side_depth_img is not None:
                        side_depth_queue.append(
                            (ts, side_depth_count, side_depth_img))
                        side_depth_count = (side_depth_count +
                                            1) % self._max_seq_num

            # MuJoCo step
            mujoco.mj_step(self.__model, self.__data)
            if not self.__headless:
                self.__viewer.sync()

            rate.sleep()

        self.close()

    def __get_states(self):
        pos = copy.deepcopy(self.__data.qpos)
        vel = copy.deepcopy(self.__data.qvel)
        eff = copy.deepcopy(self.__data.qfrc_actuator)
        pos[self.__state_left_idx[-1]] *= self.__gripper_ratio
        pos[self.__state_right_idx[-1]] *= self.__gripper_ratio
        return self.__mujoco_ts() if self.__sens_ts else hex_zmq_ts_now(
        ), np.array([
            pos[self.__state_left_idx],
            vel[self.__state_left_idx],
            eff[self.__state_left_idx],
        ]).T, np.array([
            pos[self.__state_right_idx],
            vel[self.__state_right_idx],
            eff[self.__state_right_idx],
        ]).T, self.__data.qpos[self.__obj_pose_idx].copy()

    def __init_pin(self) -> bool:
        """Load the gr100 URDF into pinocchio for the Cartesian PD (same model the
        real device uses, so sim and hardware run identical math)."""
        try:
            import pinocchio as pin
            urdf = os.path.join(os.path.dirname(__file__), "..", "..",
                                "robot", "hexarm", "urdf", "firefly_y6", "gr100.urdf")
            self.__pin = pin
            self.__pin_model = pin.buildModelFromUrdf(urdf)
            self.__pin_data = self.__pin_model.createData()
            self.__pin_model.gravity.linear = np.array([0.0, 0.0, -9.81])
            m = self.__pin_model
            self.__ee_fid = m.nframes - 1
            for name in ("link_6", "link6", "tool0", "ee_link", "gripper_base", "tcp"):
                try:
                    if m.existFrame(name):
                        self.__ee_fid = m.getFrameId(name)
                        break
                except Exception:
                    pass
            hex_log(HEX_LOG_LEVEL["info"], "[mujoco_dual] cart_impedance pinocchio ready")
            return True
        except Exception as e:
            print(f"\033[91m[mujoco_dual] cart pinocchio init failed: {e}\033[0m")
            self.__pin = None
            return False

    def __mj_gravity(self, q, state_idx):
        """MuJoCo's own bias force (gravity + Coriolis) at the arm joints, read from the
        LIVE data — already computed by each mj_step, so it is FREE.

        Earlier this did a full-scene scratch ``mj_forward`` per command; since the
        work_loop re-applies the last command EVERY tick at the ~1 kHz sim rate, that
        added ~2 k mj_forward/s on top of mj_step → the sim fell below real-time →
        high latency, and the device's 200 ms freshness check then dropped the stale
        commands and re-applied the old one (the "乱跳以前的位置" jumps). The live
        ``qfrc_bias`` is the current-state bias: at zero stiffness q == measured so it
        is exact; in position/impedance the arm tracks (good approx, Kp absorbs the
        residual). The 6 arm joints are hinges before the free-joint objects, so
        qpos index == dof index for them. ``q`` is unused (kept for the closure API)."""
        return self.__data.qfrc_bias[state_idx[:6]].copy()

    def set_control_mode(self, mode: str) -> bool:
        mode = str(mode).lower()
        if mode not in ("position", "joint_impedance", "torque", "cart_impedance"):
            print(f"\033[91m[mujoco_dual] unknown control_mode {mode!r}\033[0m")
            return False
        if mode == "cart_impedance" and self.__pin is None and not self.__init_pin():
            return False
        self.__control_mode = mode
        if mode == "cart_impedance":
            self.__cart_entry_seed = True  # first cart tick holds own pose (err=0) — parity with real
            self.__cart_dq_filt = None     # re-seed the velocity low-pass on entry
        hex_log(HEX_LOG_LEVEL["info"], f"[mujoco_dual] control_mode -> {mode}")
        return True

    @staticmethod
    def __quat_wxyz_to_mat(quat: np.ndarray) -> np.ndarray:
        w, x, y, z = quat / (np.linalg.norm(quat) + 1e-12)
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ])

    def __cart_cmd(self, cmds: np.ndarray, state_idx):
        """Build per-joint MIT arrays from a Cartesian reference (length >= 20):
        [pos(3), quat_wxyz(4), K(6), D(6), grip]. Arm joints get kp=kd=0 + the
        Cartesian tau as feedforward; the gripper keeps default gains at `grip`."""
        ref = np.asarray(cmds, dtype=np.float64).reshape(-1)
        arm_ids = state_idx[:6]
        q = np.asarray(self.__data.qpos[arm_ids], dtype=np.float64)
        if ref.shape[0] < 20:
            # Malformed Cartesian reference — e.g. a 7-vec JOINT command sent while the
            # device is in cart_impedance (go-home / manual move). DO NOT index past
            # the end (that crashed the sim). Hold the current pose: arm floats on
            # gravity (kp=0), gripper keeps its default gain.
            print(f"\033[91m[mujoco_dual] cart cmd needs >=20 values, got "
                  f"{ref.shape[0]} — holding pose\033[0m")
            grip_hold = (float(self.__data.qpos[state_idx[6]])
                         if len(state_idx) > 6 else 0.0)
            return (np.concatenate([q, [grip_hold]]), np.zeros(7), np.zeros(7),
                    np.concatenate([np.zeros(6), [self.__mit_kp[6]]]),
                    np.concatenate([np.zeros(6), [self.__mit_kd[6]]]))
        pos_des, quat_des, K, D, grip = ref[0:3], ref[3:7], ref[7:13], ref[13:19], ref[19]
        dq = np.asarray(self.__data.qvel[arm_ids], dtype=np.float64)
        pin, m, d, fid = self.__pin, self.__pin_model, self.__pin_data, self.__ee_fid
        tau = np.zeros(6)
        try:
            pin.forwardKinematics(m, d, q)
            pin.updateFramePlacements(m, d)
            x_cur = np.asarray(d.oMf[fid].translation)
            R_cur = np.asarray(d.oMf[fid].rotation)
            J = np.asarray(pin.computeFrameJacobian(
                m, d, q, fid, pin.LOCAL_WORLD_ALIGNED))[:, :6]
            R_des = self.__quat_wxyz_to_mat(quat_des)
            if self.__cart_entry_seed:
                # First tick after entering cart: hold THIS pose exactly (err=0) — parity
                # with the real device, so a skewed bumpless reference can't kick on entry.
                err = np.zeros(6)
                self.__cart_entry_seed = False
            else:
                err = np.concatenate([pos_des - x_cur, pin.log3(R_des @ R_cur.T)])
            a = self.__cart_dq_alpha   # low-pass dq for the D term (parity with real anti-buzz)
            if self.__cart_dq_filt is None or self.__cart_dq_filt.shape != dq.shape:
                self.__cart_dq_filt = dq.copy()
            else:
                self.__cart_dq_filt = a * self.__cart_dq_filt + (1.0 - a) * dq
            vf = self.__cart_dq_filt.copy()                 # deadband (parity with real)
            vf[np.abs(vf) < self.__cart_vel_deadband] = 0.0
            F = K * err - D * (J @ vf)
            fmag = np.linalg.norm(F[:3])
            if fmag > self.__cart_f_clamp:
                F[:3] *= self.__cart_f_clamp / fmag
            tmag = np.linalg.norm(F[3:6])   # rotational wrench clamp (drives the wrist via Jᵀ)
            if tmag > self.__cart_t_clamp:
                F[3:6] *= self.__cart_t_clamp / tmag
            # tau = Jᵀ F - SOFTWARE joint-space B damping (filtered/deadbanded velocity);
            # gravity added by the shared MitArmSafety in __set_cmds.
            tau = J.T @ F - self.__cart_joint_b[:6] * vf
        except Exception as e:
            print(f"\033[91m[mujoco_dual] cart impedance failed: {e}\033[0m")
        cmd_pos = np.concatenate([q, [grip]])
        tar_vel = np.zeros(7)
        cmd_tor = np.concatenate([tau, [0.0]])
        cmd_kp = np.concatenate([np.zeros(6), [self.__mit_kp[6]]])
        # arm joints: kp=0 + small firmware-local kd (anti-buzz, parity with real); gripper default
        cmd_kd = np.concatenate([self.__cart_joint_kd[:6], [self.__mit_kd[6]]])
        return cmd_pos, tar_vel, cmd_tor, cmd_kp, cmd_kd

    def __set_cmds(self, cmds: np.ndarray, robot_name: str):
        if robot_name == "left":
            ctrl_idx = self.__ctrl_left_idx
            state_idx = self.__state_left_idx
            limit_idx = 0
        elif robot_name == "right":
            ctrl_idx = self.__ctrl_right_idx
            state_idx = self.__state_right_idx
            limit_idx = 1
        else:
            raise ValueError(f"unknown robot name: {robot_name}")

        tau_cmds = None
        if not self.__tau_ctrl:
            if self.__control_mode == "cart_impedance" and self.__pin is not None:
                # Cartesian payload — build the per-joint MIT arrays so the existing
                # MIT controller below turns them into actuator torque: arm joints
                # kp=kd=0 + Cartesian tau as feedforward; gripper held in position.
                cmd_pos, tar_vel, cmd_tor, cmd_kp, cmd_kd = self.__cart_cmd(cmds, state_idx)
            else:
                cmd_pos = None
                tar_vel = np.zeros(cmds.shape[0])
                cmd_tor = np.zeros(cmds.shape[0])
                cmd_kp = self.__mit_kp.copy()
                cmd_kd = self.__mit_kd.copy()
                if len(cmds.shape) == 1:
                    cmd_pos = cmds.copy()
                elif len(cmds.shape) == 2:
                    if cmds.shape[1] == 2:
                        cmd_pos = cmds[:, 0].copy()
                        cmd_tor = cmds[:, 1].copy()
                    elif cmds.shape[1] == 5:
                        cmd_pos = cmds[:, 0].copy()
                        tar_vel = cmds[:, 1].copy()
                        cmd_tor = cmds[:, 2].copy()
                        cmd_kp = cmds[:, 3].copy()
                        cmd_kd = cmds[:, 4].copy()
                    else:
                        raise ValueError(
                            f"The shape of cmds is invalid: {cmds.shape}")
                else:
                    raise ValueError(
                        f"The shape of cmds is invalid: {cmds.shape}")
            # Reference slew (anti-lunge): keep the commanded ARM target within
            # max_pos_err of the MEASURED angle so kp*(q_des-q) stays inside the
            # motor force envelope. Applied BEFORE the joint-limit clamp so the
            # limiter only ever sees an in-range, near-current target.
            if self.__max_pos_err is not None and cmd_pos is not None:
                q_meas6 = np.asarray(self.__data.qpos[state_idx[:6]],
                                     dtype=np.float64)
                cmd_pos = np.asarray(cmd_pos, dtype=np.float64).copy()
                cmd_pos[:6] = q_meas6 + np.clip(
                    cmd_pos[:6] - q_meas6, -self.__max_pos_err, self.__max_pos_err)
            tar_pos = self._apply_pos_limits(
                cmd_pos,
                self._limits[limit_idx, :, 0],
                self._limits[limit_idx, :, 1],
            )
            tar_pos[-1] /= self.__gripper_ratio
            # Gravity feedforward (zero-stiffness-safe) + effort/slew clamp on the ARM
            # feedforward via the SHARED MitArmSafety — identical to the real device,
            # so torque/impedance behaves the same in sim. cmd_tor[:6] = arm ff.
            q_meas = np.asarray(self.__data.qpos[state_idx[:6]], dtype=np.float64)
            grav_fn = ((lambda qq: self.__mj_gravity(qq, state_idx))
                       if self.__gravity_comp else None)
            if self.__control_mode == "cart_impedance":
                # Cartesian path mirrors the REAL device (robot_hexarm.__set_cmds_cart):
                # add FULL gravity (min(grav_scale,1.0)=1.0), NOT the zero-stiffness
                # lowstiff (0.6) scale — in task space the arm must hold level and the
                # Cartesian spring only fights disturbances. cmd_kp[:6]=0 here, so going
                # through MitArmSafety.apply would WRONGLY take the kp~0 lowstiff branch
                # (0.6*g) and the sim arm would sag ~40% more than real. So add gravity
                # explicitly at 1.0 then clamp() only (no lowstiff re-add) — exact parity.
                if grav_fn is not None:
                    cmd_tor[:6] = cmd_tor[:6] + min(self.__safety._grav_scale, 1.0) * grav_fn(q_meas)
                cmd_tor[:6] = self.__safety.clamp(cmd_tor[:6], slew_key=robot_name)
            else:
                cmd_tor[:6] = self.__safety.apply(
                    cmd_tor[:6], tar_pos[:6], q_meas, cmd_kp[:6], grav_fn,
                    slew_key=robot_name)
            tau_cmds = self.__mit_ctrl(
                cmd_kp,
                cmd_kd,
                tar_pos,
                tar_vel,
                self.__data.qpos[state_idx],
                self.__data.qvel[state_idx],
                cmd_tor,
            )
        else:
            tau_cmds = cmds.copy()
        self.__data.ctrl[ctrl_idx] = tau_cmds

    def __get_rgb(self, camera_name: str):
        self.__rgb_cam.update_scene(self.__data, camera_name)
        rgb_img = self.__rgb_cam.render()
        return self.__mujoco_ts() if self.__sens_ts else hex_zmq_ts_now(
        ), cv2.cvtColor(rgb_img, cv2.COLOR_RGB2BGR)

    def __get_depth(self, camera_name: str):
        self.__depth_cam.update_scene(self.__data, camera_name)
        depth_m = self.__depth_cam.render().astype(np.float32)
        depth_img = np.clip(depth_m * 1000.0, 0, 65535).astype(np.uint16)
        return self.__mujoco_ts() if self.__sens_ts else hex_zmq_ts_now(
        ), depth_img

    def get_side_intri(self):
        return self._side_intri

    def __mujoco_ts(self):
        mujoco_ts = self.__data.time * 1_000_000_000 + self.__bias_ns
        return ns_to_hex_zmq_ts(mujoco_ts)

    def close(self):
        if not self._working.is_set():
            return
        self._working.clear()
        if self.__rgb_cam is not None:
            self.__rgb_cam.close()
        if self.__depth_cam is not None:
            self.__depth_cam.close()
        if self.__side_rgb_cam is not None:
            self.__side_rgb_cam.close()
        if self.__side_depth_cam is not None:
            self.__side_depth_cam.close()
        if not self.__headless:
            self.__viewer.close()
        hex_log(HEX_LOG_LEVEL["info"], "HexMujocoFireflyY6Dual closed")
