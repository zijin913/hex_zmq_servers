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
        if not self.__tau_ctrl:
            self.__mit_kp = np.ascontiguousarray(np.asarray(self.__mit_kp))
            self.__mit_kd = np.ascontiguousarray(np.asarray(self.__mit_kd))
            self.__mit_ctrl = CtrlUtil()
        # GR100 lobster claw: client commands gripper in [0, 0.55]; URDF/MJCF range is [0, 0.57].
        # Compress advertised range to match client expectation.
        self.__gripper_ratio = 0.55 / 0.57
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
                    self.__set_cmds(cmds_left, "left")

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
                    self.__set_cmds(cmds_right, "right")

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
            tar_pos = self._apply_pos_limits(
                cmd_pos,
                self._limits[limit_idx, :, 0],
                self._limits[limit_idx, :, 1],
            )
            tar_pos[-1] /= self.__gripper_ratio
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
