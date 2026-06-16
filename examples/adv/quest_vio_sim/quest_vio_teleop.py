#!/usr/bin/env python3
"""
Meta Quest 3 → MuJoCo HexArm 笛卡尔空间遥操 (仿真版)

结构与 iphone_vio_sim/iphone_vio_teleop.py 对齐，只替换：
    - 数据源: iPhoneVIOReceiver(UDP:5005)  →  QuestReader(ADB)
    - 坐标映射: ARKit 多步修正  →  Quest 控制器单 3x3 重映射
    - 新增: --hand {left,right} 选择控制器, --quest-ip 切 Wi-Fi 模式

使用方法:
    # 终端 1: 启动 MuJoCo 仿真
    source hex_zmq_servers/.venv/bin/activate
    python hex_zmq_servers/examples/basic/mujoco_archer_l6y/launch.py

    # 终端 2: 运行本脚本
    source hex_zmq_servers/.venv/bin/activate
    python hex_zmq_servers/examples/adv/quest_vio_sim/quest_vio_teleop.py --hand right

    # Quest: USB 连上电脑, 已经用 git-lfs 装好 APK, adb devices 能看到头显

键盘控制 (焦点在 OpenCV 窗口):
    q: 退出
    r: 重新居中 (当前 Quest 位姿 → 新参考点)
    g: 切换夹爪开/合
    +/-: 调整位置缩放
    p: position+rotation 模式
    o: position-only 模式
    b: rotation-only 模式

Quest 控制器按键:
    扳机 (index trigger)  : clutch (按住才跟随)
    握把 (grip trigger)   : 夹爪 analog 0~1
    A / X (右/左手)        : home (回零)
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

SODA_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
sys.path.insert(0, SODA_ROOT)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))
# oculus_reader 在 third_party/
sys.path.insert(0, os.path.join(SODA_ROOT, "third_party", "oculus_reader"))

from hex_zmq_servers import HexRate, HexMujocoArcherL6YClient, HEXARM_URDF_PATH_DICT
from hex_robo_utils import part2trans, trans2part, trans_inv
from utils.analytic_ik import HexDynUtilL6Y
from quest_vio_common.quest_reader import QuestReader
# 坐标映射的唯一真源 —— 改这里会同时影响 collect_demos、sim、real
from soda_os.controllers.quest_vio import (
    R_REMAP_QUEST_DEFAULT as R_REMAP,
    ROT_SIGN_FLIP_DEFAULT as ROT_SIGN_FLIP,
)

# ============================================================
# 配置
# ============================================================
ARM_TYPE = "archer_l6y"
GRIPPER_TYPE = "gp100"
HEXARM_URDF = HEXARM_URDF_PATH_DICT[f"{ARM_TYPE}_{GRIPPER_TYPE}"]
HEXARM_HOME = np.array([0.0, -0.785, 2.2, 0.5, 0.0, 0.0])
HEX_LOWER = np.array([-2.86, -2.09, 0.0, -1.57, -1.57, -3.14])
HEX_UPPER = np.array([2.86, 1.57, 3.16, 1.57, 1.57, 3.14])

MUJOCO_PORT = 12345

# 映射参数 (仿真版略宽松)
POSITION_SCALE = 1.2
INTERP_ERR_LIMIT = 0.01
POS_DEADZONE = 0.002
ROT_DEADZONE = 0.07

DROPOUT_TIMEOUT = 0.5

# R_REMAP 已在顶部从 soda_os.controllers.quest_vio 导入 —— 统一在那里维护


def orthogonalize(R):
    U, _, Vt = np.linalg.svd(R)
    R_orth = U @ Vt
    if np.linalg.det(R_orth) < 0:
        U[:, -1] *= -1
        R_orth = U @ Vt
    return R_orth


def rot_to_quat(R):
    import pinocchio as pin
    R_orth = orthogonalize(R)
    q = pin.Quaternion(R_orth)
    return np.array([q.w, q.x, q.y, q.z])


def interp_joint(cur_q, tar_q, err_limit=0.1):
    err = tar_q - cur_q
    max_err = np.fabs(err).max()
    if max_err < err_limit:
        return tar_q.copy(), False
    return cur_q + (err / max_err) * err_limit, True


def interp_arm(cur_q, tar_joint, grip_val, err_limit=0.1):
    mid = np.zeros(7)
    mid[:6], interp_flag = interp_joint(cur_q[:6], tar_joint, err_limit=err_limit)
    mid[6], _ = interp_joint(np.array([cur_q[6]]), np.array([grip_val]), err_limit=err_limit)
    return mid, interp_flag


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--hand", choices=["left", "right"], default="right",
                        help="使用哪只 Quest 控制器")
    parser.add_argument("--quest-ip", type=str, default=None,
                        help="Wi-Fi IP (默认走 USB/ADB)")
    args = parser.parse_args()

    END_POSE_LINK6 = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    solver = HexDynUtilL6Y(HEXARM_URDF, "link_6", end_pose=END_POSE_LINK6)
    print(f"Analytic IK solver loaded: {HEXARM_URDF}")

    vio = QuestReader(hand=args.hand, ip_address=args.quest_ip)

    mujoco_client = HexMujocoArcherL6YClient(net_config={
        "ip": "127.0.0.1", "port": MUJOCO_PORT,
        "realtime_mode": False, "deque_maxlen": 10,
        "client_timeout_ms": 200, "server_timeout_ms": 1000,
        "server_num_workers": 4,
    })
    print(f"等待 MuJoCo server (port {MUJOCO_PORT})...")
    for _ in range(50):
        if mujoco_client.is_working():
            break
        time.sleep(0.1)
    else:
        print("MuJoCo 未响应！")
        vio.close()
        return
    print("MuJoCo 已连接!")

    # ---- Smoothstep 回 home ----
    print("\n平滑移动到 home (5秒)...")
    tau_comp = np.zeros(7)
    start_q = np.zeros(7)
    start_q[:6] = HEXARM_HOME
    for _ in range(20):
        hdr, states = mujoco_client.get_states("robot")
        if states is not None:
            start_q = states[:, 0].copy()
            arm_q = states[:, 0][:-1]
            arm_dq = states[:, 1][:-1]
            _, c_mat, g_vec, _, _ = solver.dynamic_params(arm_q, arm_dq)
            tau_comp = np.concatenate((c_mat @ arm_dq + g_vec, np.zeros(1)))
            break
        time.sleep(0.05)

    target_q = np.zeros(7)
    target_q[:6] = HEXARM_HOME

    n_steps = 500  # 5 sec @ 100Hz
    for i in range(n_steps):
        t = i / n_steps
        alpha = 3 * t * t - 2 * t * t * t
        interp_q = start_q + alpha * (target_q - start_q)

        hdr, states = mujoco_client.get_states("robot")
        if states is not None:
            arm_q = states[:, 0][:-1]
            arm_dq = states[:, 1][:-1]
            _, c_mat, g_vec, _, _ = solver.dynamic_params(arm_q, arm_dq)
            tau_comp = np.concatenate((c_mat @ arm_dq + g_vec, np.zeros(1)))

        cmds = np.concatenate((interp_q.reshape(-1, 1), tau_comp.reshape(-1, 1)), axis=1)
        mujoco_client.set_cmds(cmds)
        time.sleep(0.01)
    print("已到 home!")

    # ---- Home EE 位姿 ----
    home_fk = solver.forward_kinematics(HEXARM_HOME)
    home_pos, home_quat = home_fk[-1]
    T_home_ee = part2trans(home_pos, home_quat)

    # ---- 等 Quest 数据 ----
    print(f"\n等待 Quest 3 {args.hand} 控制器数据...")
    print("请戴上头显并确保 APK 已在后台运行。")
    while True:
        T_quest, ts, cnt, _, _, _ = vio.get_pose()
        if T_quest is not None:
            print(f"收到 Quest 数据! ({cnt} 帧)")
            break
        time.sleep(0.1)

    T_quest_ref = T_quest.copy()
    T_quest_ref_inv = np.linalg.inv(T_quest_ref)

    # ---- 控制状态 ----
    tar_joint = HEXARM_HOME.copy()
    tar_pos = home_pos.copy()
    tar_quat = home_quat.copy()
    smooth_pos = home_pos.copy()
    smooth_quat = home_quat.copy()
    grip_flag = False
    last_cmd_q = np.zeros(7)
    last_cmd_q[:6] = HEXARM_HOME
    last_cmd_q[6] = 0.2
    cur_q = last_cmd_q.copy()
    tau_comp = np.zeros(7)
    mode = "both"
    pos_scale = POSITION_SCALE
    last_recv_count = 0
    last_recv_time = time.time()
    was_clutch_active = False
    clutch_base_T = T_home_ee.copy()
    SMOOTH_ALPHA = 0.05

    print()
    print("=" * 60)
    print(f"Quest 3 遥操已启动 ({args.hand} 控制器)!")
    print(f"  位置缩放: {pos_scale:.1f}x, 模式: {mode}")
    print("  按住扳机启动 clutch | 握把控制夹爪 | A/X 回零")
    print("  键盘: r 重新居中 | g 夹爪切换 | +/- 缩放 | p/o/b 模式 | q 退出")
    print("=" * 60)

    rate = HexRate(100)
    frame_count = 0

    try:
        while True:
            # 1. 读 MuJoCo 状态
            robot_hdr, robot_states = mujoco_client.get_states("robot")
            if robot_hdr is not None:
                cur_q = robot_states[:, 0].copy()
                cur_dq = robot_states[:, 1].copy()
                arm_q = cur_q[:-1]
                arm_dq = cur_dq[:-1]
                _, c_mat, g_vec, _, _ = solver.dynamic_params(arm_q, arm_dq)
                tau_comp = np.concatenate((c_mat @ arm_dq + g_vec, np.zeros(1)))

            # 2. 读 Quest
            T_quest, vio_ts, vio_count, vio_gripper, vio_clutch, vio_home = vio.get_pose()
            now = time.time()

            if T_quest is not None and vio_count > last_recv_count:
                last_recv_count = vio_count
                last_recv_time = now

                if vio_home:
                    tar_pos = home_pos.copy()
                    tar_quat = home_quat.copy()
                    smooth_pos = home_pos.copy()
                    smooth_quat = home_quat.copy()
                    clutch_base_T = T_home_ee.copy()
                    tar_joint = HEXARM_HOME.copy()
                    print(">>> HOME — 回到 home")

                if vio_clutch and not was_clutch_active:
                    T_quest_ref = T_quest.copy()
                    T_quest_ref_inv = np.linalg.inv(T_quest_ref)
                    cur_fk_pos, cur_fk_quat = solver.forward_kinematics(last_cmd_q[:6])[-1]
                    clutch_base_T = part2trans(cur_fk_pos, cur_fk_quat)
                    if frame_count > 0:
                        print(">>> Clutch ON — 从当前位置开始")
                elif not vio_clutch and was_clutch_active:
                    print(">>> Clutch OFF — 冻结")
                was_clutch_active = vio_clutch

                if vio_clutch:
                    delta_T = T_quest_ref_inv @ T_quest
                    delta_pos_q = delta_T[:3, 3]
                    delta_rot_q = delta_T[:3, :3]

                    cur_delta_pos = R_REMAP @ delta_pos_q * pos_scale
                    # 旋转: 相似变换 + HexArm 坐标系下 per-axis 符号翻转
                    mapped = R_REMAP @ delta_rot_q @ R_REMAP.T
                    from scipy.spatial.transform import Rotation as _RF
                    rotvec = _RF.from_matrix(mapped).as_rotvec() * ROT_SIGN_FLIP
                    cur_delta_rot = _RF.from_rotvec(rotvec).as_matrix()

                    if np.linalg.norm(delta_pos_q) < POS_DEADZONE:
                        cur_delta_pos = np.zeros(3)
                    rot_angle = np.arccos(np.clip((np.trace(delta_rot_q) - 1) / 2, -1, 1))
                    if rot_angle < ROT_DEADZONE:
                        cur_delta_rot = np.eye(3)

                    if mode == "both":
                        tar_pos = clutch_base_T[:3, 3] + cur_delta_pos
                        tar_rot = clutch_base_T[:3, :3] @ cur_delta_rot
                        tar_quat = rot_to_quat(tar_rot)
                    elif mode == "pos":
                        tar_pos = clutch_base_T[:3, 3] + cur_delta_pos
                    elif mode == "rot":
                        tar_rot = clutch_base_T[:3, :3] @ cur_delta_rot
                        tar_quat = rot_to_quat(tar_rot)

            elif (now - last_recv_time) > DROPOUT_TIMEOUT:
                if frame_count % 500 == 0:
                    print(f"  [!] Quest 数据丢失 {now - last_recv_time:.1f}s")

            # 3. 低通滤波
            smooth_pos = smooth_pos * (1.0 - SMOOTH_ALPHA) + tar_pos * SMOOTH_ALPHA
            smooth_quat = smooth_quat * (1.0 - SMOOTH_ALPHA) + tar_quat * SMOOTH_ALPHA
            smooth_quat = smooth_quat / np.linalg.norm(smooth_quat)

            # 4. IK
            ik_success, ik_q = solver.inverse_kinematics_analytic(
                (smooth_pos, smooth_quat), last_cmd_q[:6])
            if not ik_success:
                ik_success_num, ik_q_num, ik_err = solver.inverse_kinematics(
                    (smooth_pos, smooth_quat), last_cmd_q[:6], max_iter=50)
                if ik_success_num:
                    ik_success = True
                    ik_q = ik_q_num

            if ik_success:
                tar_joint = np.clip(ik_q, HEX_LOWER, HEX_UPPER)
            else:
                tar_pos, tar_quat = solver.forward_kinematics(last_cmd_q[:6])[-1]
                smooth_pos = tar_pos.copy()
                smooth_quat = tar_quat.copy()

            # 5. 插值 + 夹爪
            if grip_flag:
                grip_val = 1.33
            else:
                grip_val = 0.2 + vio_gripper * (1.33 - 0.2)
            mid_joint, _ = interp_arm(
                last_cmd_q, tar_joint, grip_val, err_limit=INTERP_ERR_LIMIT)
            last_cmd_q = mid_joint.copy()

            cmds = np.concatenate(
                (mid_joint.reshape(-1, 1), tau_comp.reshape(-1, 1)), axis=1)
            mujoco_client.set_cmds(cmds)

            frame_count += 1
            if frame_count % 100 == 0:
                dp = tar_pos - T_home_ee[:3, 3]
                print(f"  [{mode:4s}] scale={pos_scale:.1f}x  "
                      f"delta=[{dp[0]:+.3f} {dp[1]:+.3f} {dp[2]:+.3f}]m  "
                      f"IK:{'OK' if ik_success else 'FAIL'}  "
                      f"grip={vio_gripper:.0%}→{grip_val:.2f}  "
                      f"clutch={'ON' if vio_clutch else 'OFF'}  "
                      f"VIO:{vio_count}")

            # 摄像头显示
            rgb_hdr, rgb = mujoco_client.get_rgb()
            if rgb_hdr is not None:
                cv2.imshow("MuJoCo Quest Teleop", rgb)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord('r'):
                T_cur, _, _, _, _, _ = vio.get_pose()
                if T_cur is not None:
                    T_quest_ref = T_cur.copy()
                    T_quest_ref_inv = np.linalg.inv(T_quest_ref)
                    tar_pos = home_pos.copy()
                    tar_quat = home_quat.copy()
                    smooth_pos = home_pos.copy()
                    smooth_quat = home_quat.copy()
                    clutch_base_T = T_home_ee.copy()
                    print(">>> Recenter! 回到 home")
            elif key == ord('g'):
                grip_flag = not grip_flag
                print(f">>> 夹爪: {'关闭' if grip_flag else '打开'}")
            elif key == ord('+') or key == ord('='):
                pos_scale = min(pos_scale + 0.5, 5.0)
                print(f">>> 缩放: {pos_scale:.1f}x")
            elif key == ord('-'):
                pos_scale = max(pos_scale - 0.5, 0.5)
                print(f">>> 缩放: {pos_scale:.1f}x")
            elif key == ord('p'):
                mode = "both"
                print(">>> position + rotation")
            elif key == ord('o'):
                mode = "pos"
                print(">>> position only")
            elif key == ord('b'):
                mode = "rot"
                print(">>> rotation only")

            rate.sleep()

    except KeyboardInterrupt:
        print("\n退出")
    finally:
        vio.close()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
