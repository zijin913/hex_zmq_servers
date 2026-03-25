#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# SO-101 Leader → HexArm L6Y Cartesian Space Teleop Client
#
# Pipeline:
#   SO-101 joints (5) → Pinocchio FK → EE pose
#   → incremental delta from home → workspace scale
#   → HexArm target pose → Analytic IK → HexArm joints (6)
################################################################

import argparse
import json
import time

import cv2
import numpy as np
import pinocchio as pin
from hex_zmq_servers import (
    HexRate,
    HEX_LOG_LEVEL,
    hex_log,
    HexRobotSO101Client,
    HexMujocoArcherL6YClient,
)
from hex_robo_utils import HexDynUtil as DynUtil
from hex_robo_utils import part2trans, trans2part, trans_inv


def wait_client_working(client, timeout: float = 5.0) -> bool:
    for _ in range(int(timeout * 10)):
        if client.is_working():
            if hasattr(client, "seq_clear"):
                client.seq_clear()
            return True
        else:
            time.sleep(0.1)
    return False


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    """Convert 3x3 rotation matrix to quaternion [qw, qx, qy, qz]."""
    quat = pin.Quaternion(R)
    return np.array([quat.w, quat.x, quat.y, quat.z])


class SO101FK:
    """Forward kinematics for SO-101 using Pinocchio."""

    def __init__(self, urdf_path: str):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.ee_frame_id = self.model.getFrameId("gripper_frame_link")
        # SO-101 has 6 joints (5 arm + 1 gripper), we only use first 5 for FK
        self.arm_nq = 5
        print(f"SO-101 FK loaded: {self.model.nq} DOF, "
              f"EE frame: gripper_frame_link (id={self.ee_frame_id})")

    def compute(self, arm_joints: np.ndarray) -> np.ndarray:
        """
        Compute FK for 5 arm joints.

        Args:
            arm_joints: 5 arm joint angles in radians

        Returns:
            4x4 homogeneous transform of end-effector
        """
        q = np.zeros(self.model.nq)
        q[:self.arm_nq] = arm_joints[:self.arm_nq]
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.ee_frame_id)
        return self.data.oMf[self.ee_frame_id].homogeneous.copy()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    try:
        so101_urdf_path = cfg["so101_urdf_path"]
        hexarm_model_path = cfg["hexarm_model_path"]
        last_link = cfg["last_link"]
        workspace_scale = cfg.get("workspace_scale", 1.5)
        gripper_scale = cfg.get("gripper_scale", 1.0)
        so101_net_cfg = cfg["so101_net_cfg"]
        mujoco_net_cfg = cfg["mujoco_net_cfg"]
    except KeyError as ke:
        missing_key = ke.args[0]
        raise ValueError(f"cfg is not valid, missing key: {missing_key}")

    # Initialize clients
    so101_client = HexRobotSO101Client(net_config=so101_net_cfg)
    mujoco_client = HexMujocoArcherL6YClient(net_config=mujoco_net_cfg)

    # Initialize kinematics
    so101_fk = SO101FK(so101_urdf_path)
    dyn_util = DynUtil(hexarm_model_path, last_link)

    # HexArm home configuration
    HEXARM_HOME = np.array([0.0, -0.785, 2.2, 0.5, 0.0, 0.0])

    # Compute reference poses at home
    # SO-101 home: all zeros (leader at rest position)
    so101_home_T = so101_fk.compute(np.zeros(5))

    # HexArm home: compute FK
    hexarm_fk_list = dyn_util.forward_kinematics(HEXARM_HOME)
    hexarm_home_pos = hexarm_fk_list[-1][0]  # position of last link
    hexarm_home_quat = hexarm_fk_list[-1][1]  # quaternion of last link
    hexarm_home_T = part2trans(hexarm_home_pos, hexarm_home_quat)

    print(f"SO-101 home EE: {so101_home_T[:3, 3].round(4)}")
    print(f"HexArm home EE: {hexarm_home_pos.round(4)}")
    print(f"Workspace scale: {workspace_scale}")

    # Wait servers
    if not wait_client_working(so101_client):
        hex_log(HEX_LOG_LEVEL["err"], "SO-101 server is not working")
        return
    if not wait_client_working(mujoco_client):
        hex_log(HEX_LOG_LEVEL["err"], "MuJoCo server is not working")
        return

    # Control loop
    rate = HexRate(250)
    hex_q = None
    current_hexarm_q = HEXARM_HOME.copy()

    try:
        while True:
            # 1. Read SO-101 states (6 values: 5 arm + 1 gripper)
            so101_hdr, so101_states = so101_client.get_states()

            # 2. Read MuJoCo HexArm states
            robot_hdr, robot_states = mujoco_client.get_states("robot")

            if robot_states is not None:
                current_hexarm_q = robot_states[:6, 0]

            if so101_states is not None:
                so101_arm = so101_states[:5]  # 5 arm joints
                so101_grip = so101_states[5]  # gripper

                # 3. SO-101 FK → end-effector pose
                so101_T = so101_fk.compute(so101_arm)

                # 4. Compute delta from SO-101 home
                delta_T = np.linalg.inv(so101_home_T) @ so101_T
                delta_pos = delta_T[:3, 3] * workspace_scale
                delta_rot = delta_T[:3, :3]

                # 5. Map to HexArm workspace
                target_T = hexarm_home_T.copy()
                target_T[:3, 3] = hexarm_home_T[:3, 3] + delta_pos
                target_T[:3, :3] = hexarm_home_T[:3, :3] @ delta_rot

                # 6. HexArm IK → 6 joint angles
                target_pos = target_T[:3, 3]
                target_quat = rot_to_quat(target_T[:3, :3])

                success, ik_q = dyn_util.inverse_kinematics_analytic(
                    (target_pos, target_quat), current_hexarm_q)

                if success:
                    hex_q = np.zeros(7)  # 6 arm + 1 gripper
                    hex_q[:6] = ik_q
                    hex_q[6] = np.clip(so101_grip * gripper_scale, 0.0, 1.33)

            # 7. Gravity compensation + send commands
            if robot_states is not None and hex_q is not None:
                arm_q = robot_states[:, 0][:-1]
                arm_dq = robot_states[:, 1][:-1]
                _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
                tau_comp = np.zeros(7)
                tau_comp[:-1] = c_mat @ arm_dq + g_vec
                cmds = np.concatenate(
                    (hex_q.reshape(-1, 1), tau_comp.reshape(-1, 1)), axis=1)
                mujoco_client.set_cmds(cmds)

            # 8. Display camera (if available)
            rgb_hdr, rgb = mujoco_client.get_rgb()
            if rgb_hdr is not None:
                cv2.imshow("rgb_img", rgb)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break

            rate.sleep()
    finally:
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
