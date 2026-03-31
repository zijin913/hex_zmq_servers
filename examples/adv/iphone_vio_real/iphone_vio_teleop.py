#!/usr/bin/env python3
"""
iPhone VIO → 真机 HexArm L6Y 笛卡尔空间遥操

与仿真版 (iphone_vio_sim) 映射逻辑完全一致，
替换为真机 HexArm 客户端 + 更保守的安全参数。

使用方法:
    # SOMA 上启动 server
    python launchers/launch_servers.py

    # 本地运行
    python hex_zmq_servers/examples/adv/iphone_vio_real/iphone_vio_teleop.py --remote
    python hex_zmq_servers/examples/adv/iphone_vio_real/iphone_vio_teleop.py --local

键盘控制 (OpenCV 窗口):
    q: 退出
    r: 重新居中
    g: 切换夹爪
    +/-: 调整位置缩放
    p/o/b: position+rotation / position-only / rotation-only
"""

import os
import sys
import time
import argparse
import numpy as np
import cv2

SODA_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
sys.path.insert(0, SODA_ROOT)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hex_zmq_servers import HexRate, HexRobotHexarmClient, HEXARM_URDF_PATH_DICT
from hex_robo_utils import part2trans, trans2part, trans_inv
from utils.analytic_ik import HexDynUtilL6Y
from iphone_vio_common.vio_receiver import iPhoneVIOReceiver

# ============================================================
# 配置
# ============================================================
ARM_TYPE = "archer_l6y"
GRIPPER_TYPE = "gp100"
HEXARM_URDF = HEXARM_URDF_PATH_DICT[f"{ARM_TYPE}_{GRIPPER_TYPE}"]
HEXARM_HOME = np.array([0.0, -0.785, 2.2, 0.5, 0.0, 0.0])
HEX_LOWER = np.array([-2.86, -2.09, 0.0, -1.57, -1.57, -3.14])
HEX_UPPER = np.array([2.86, 1.57, 3.16, 1.57, 1.57, 3.14])

# 真机 HexArm server
HEXARM_IP = "10.102.211.210"  # SOMA
HEXARM_PORT = 12345
VIO_PORT = 5005

# 映射参数（真机更保守）
POSITION_SCALE = 1.0
ROTATION_SCALE = 1.0
INTERP_ERR_LIMIT = 0.005  # 真机更小，运动更柔和
POS_DEADZONE = 0.02
ROT_DEADZONE = 0.07

DROPOUT_TIMEOUT = 0.5
RECENTER_TIMEOUT = 3.0

# 坐标系映射（与仿真版一致）
R_REMAP = np.array([
    [-1, 0, 0],
    [0, 0, -1],
    [0, -1, 0],
], dtype=np.float64)


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


def interp_joint(cur_q, tar_q, err_limit=0.05):
    err = tar_q - cur_q
    max_err = np.fabs(err).max()
    if max_err < err_limit:
        return tar_q.copy(), False
    else:
        return cur_q + (err / max_err) * err_limit, True


def interp_arm(cur_q, tar_joint, grip_val, err_limit=0.05):
    mid = np.zeros(7)
    mid[:6], interp_flag = interp_joint(cur_q[:6], tar_joint, err_limit=err_limit)
    mid[6], _ = interp_joint(np.array([cur_q[6]]), np.array([grip_val]), err_limit=err_limit)
    return mid, interp_flag


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--remote", action="store_true", help="连接远程 SOMA HexArm")
    parser.add_argument("--local", action="store_true", help="连接本地 HexArm server")
    args = parser.parse_args()

    hexarm_ip = "127.0.0.1" if args.local else HEXARM_IP
    print(f"HexArm server: {hexarm_ip}:{HEXARM_PORT}")

    # ---- 初始化 ----
    END_POSE_LINK6 = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    solver = HexDynUtilL6Y(HEXARM_URDF, "link_6", end_pose=END_POSE_LINK6)
    print(f"Analytic IK solver loaded")

    vio = iPhoneVIOReceiver(port=VIO_PORT)

    hexarm_client = HexRobotHexarmClient(net_config={
        "ip": hexarm_ip, "port": HEXARM_PORT,
        "realtime_mode": False, "deque_maxlen": 10,
        "client_timeout_ms": 200, "server_timeout_ms": 1000,
        "server_num_workers": 4,
    })
    print("HexArm 已连接!")

    # ---- Home 位姿 ----
    home_fk = solver.forward_kinematics(HEXARM_HOME)
    home_pos, home_quat = home_fk[-1]
    T_home_ee = part2trans(home_pos, home_quat)

    # ---- 等待 iPhone ----
    print(f"\n等待 iPhone VIO 数据 (UDP:{VIO_PORT})...")
    while True:
        T_iphone, ts, cnt, _, _, _ = vio.get_pose()
        if T_iphone is not None:
            print(f"收到! ({cnt} 帧)")
            break
        time.sleep(0.1)

    for _ in range(30):
        vio.get_pose()
        time.sleep(0.03)

    # ---- 安全确认 ----
    print()
    print("=" * 60)
    print("即将控制真机 HexArm!")
    print(f"  位置缩放: {POSITION_SCALE:.1f}x")
    print(f"  插值限幅: {np.degrees(INTERP_ERR_LIMIT):.1f}°/帧")
    print("  真机将先平滑移到 home，然后跟随 iPhone。")
    print("  请确保 HexArm 周围无障碍物。")
    print("  按 Enter 开始，Ctrl+C 随时停止。")
    print("=" * 60)
    input()

    # ---- Smoothstep 回 home (10 秒) ----
    print("平滑移动到 home (10秒)...")
    last_tau = np.zeros(7)

    start_q = np.zeros(7)
    start_q[:6] = HEXARM_HOME
    for _ in range(20):
        hdr, states = hexarm_client.get_states()
        if states is not None:
            start_q = states[:, 0].copy()
            arm_q = states[:, 0][:-1]
            arm_dq = states[:, 1][:-1]
            _, c_mat, g_vec, _, _ = solver.dynamic_params(arm_q, arm_dq)
            last_tau[:-1] = c_mat @ arm_dq + g_vec
            break
        time.sleep(0.05)

    target_q = np.zeros(7)
    target_q[:6] = HEXARM_HOME
    print(f"  当前: {np.degrees(start_q[:6]).round(1)}°")
    print(f"  目标: {np.degrees(target_q[:6]).round(1)}°")

    n_steps = 5000  # 10 sec @ 500Hz
    for i in range(n_steps):
        t = i / n_steps
        alpha = 3 * t * t - 2 * t * t * t
        interp_q = start_q + alpha * (target_q - start_q)

        hdr, states = hexarm_client.get_states()
        if states is not None:
            arm_q = states[:, 0][:-1]
            arm_dq = states[:, 1][:-1]
            _, c_mat, g_vec, _, _ = solver.dynamic_params(arm_q, arm_dq)
            last_tau[:-1] = c_mat @ arm_dq + g_vec

        cmds = np.concatenate((interp_q.reshape(-1, 1), last_tau.reshape(-1, 1)), axis=1)
        hexarm_client.set_cmds(cmds)
        time.sleep(0.002)
    print("已到 home!")

    # ---- 记录参考位姿 ----
    T_iphone_ref, _, _, _, _, _ = vio.get_pose()
    if T_iphone_ref is None:
        T_iphone_ref = T_iphone.copy()
    T_iphone_ref_inv = np.linalg.inv(T_iphone_ref)
    print("开始遥操... (Ctrl+C 停止)\n")

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
    mode = "both"
    pos_scale = POSITION_SCALE
    last_recv_count = 0
    last_recv_time = time.time()
    was_clutch_active = False
    clutch_base_T = T_home_ee.copy()
    SMOOTH_ALPHA = 0.05

    # 旋转修正矩阵（与仿真版一致）
    R_corr = np.array([
        [ 0, -1,  0],
        [ 0,  0,  1],
        [-1,  0,  0],
    ], dtype=np.float64)
    swap_yz = np.array([
        [1, 0, 0],
        [0, 0, 1],
        [0, 1, 0],
    ], dtype=np.float64)

    rate = HexRate(500)  # 真机 500Hz
    frame_count = 0

    try:
        while True:
            # 1. 读 HexArm 状态
            hdr, states = hexarm_client.get_states()
            if states is not None:
                cur_q = states[:, 0].copy()
                cur_dq = states[:, 1].copy()
                arm_q = cur_q[:-1]
                arm_dq = cur_dq[:-1]
                _, c_mat, g_vec, _, _ = solver.dynamic_params(arm_q, arm_dq)
                last_tau[:-1] = c_mat @ arm_dq + g_vec

            # 2. 读 iPhone 位姿
            T_iphone, vio_ts, vio_count, vio_gripper, vio_clutch, vio_home = vio.get_pose()
            now = time.time()

            if vio_count > last_recv_count:
                last_recv_count = vio_count
                last_recv_time = now

                # HOME 按钮
                if vio_home:
                    tar_pos = home_pos.copy()
                    tar_quat = home_quat.copy()
                    smooth_pos = home_pos.copy()
                    smooth_quat = home_quat.copy()
                    clutch_base_T = T_home_ee.copy()
                    tar_joint = HEXARM_HOME.copy()
                    print(f">>> HOME — 回到 home")

                # Clutch
                if vio_clutch and not was_clutch_active:
                    T_iphone_ref = T_iphone.copy()
                    T_iphone_ref_inv = np.linalg.inv(T_iphone_ref)
                    cur_fk_pos, cur_fk_quat = solver.forward_kinematics(last_cmd_q[:6])[-1]
                    clutch_base_T = part2trans(cur_fk_pos, cur_fk_quat)
                    if frame_count > 0:
                        print(f">>> Clutch ON")
                elif not vio_clutch and was_clutch_active:
                    print(f">>> Clutch OFF")
                was_clutch_active = vio_clutch

                if vio_clutch:
                    delta_T = T_iphone_ref_inv @ T_iphone
                    delta_pos_iphone = delta_T[:3, 3]
                    delta_rot_iphone = delta_T[:3, :3]

                    # 位置映射
                    cur_delta_pos = R_REMAP @ delta_pos_iphone * pos_scale

                    # 旋转映射（与仿真版一致）
                    delta_rot_corrected = R_corr @ delta_rot_iphone @ R_corr.T
                    mapped = R_REMAP @ delta_rot_corrected @ R_REMAP.T
                    fixed = swap_yz @ mapped @ swap_yz
                    from scipy.spatial.transform import Rotation as RotFix
                    rotvec = RotFix.from_matrix(fixed).as_rotvec()
                    rotvec[0] = -rotvec[0]
                    rotvec[1] = -rotvec[1]
                    cur_delta_rot = RotFix.from_rotvec(rotvec).as_matrix()

                    # 死区
                    if np.linalg.norm(delta_pos_iphone) < POS_DEADZONE:
                        cur_delta_pos = np.zeros(3)
                    rot_angle = np.arccos(np.clip((np.trace(delta_rot_iphone) - 1) / 2, -1, 1))
                    if rot_angle < ROT_DEADZONE:
                        cur_delta_rot = np.eye(3)

                    # 目标
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
                if frame_count % 2500 == 0:
                    print(f"  [!] 数据丢失 {now - last_recv_time:.1f}s")

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

            # 6. 发命令
            cmds = np.concatenate(
                (mid_joint.reshape(-1, 1), last_tau.reshape(-1, 1)), axis=1)
            hexarm_client.set_cmds(cmds)

            # 7. 显示
            frame_count += 1
            if frame_count % 500 == 0:
                dp = tar_pos - T_home_ee[:3, 3]
                print(f"  [{mode:4s}] scale={pos_scale:.1f}x  "
                      f"delta=[{dp[0]:+.3f} {dp[1]:+.3f} {dp[2]:+.3f}]m  "
                      f"IK:{'OK' if ik_success else 'FAIL'}  "
                      f"clutch={'ON' if vio_clutch else 'OFF'}")

            # 8. 键盘
            dummy = np.zeros((60, 300, 3), dtype=np.uint8)
            cv2.putText(dummy, f"VIO [{mode}] {pos_scale:.1f}x clutch:{'ON' if vio_clutch else 'OFF'}",
                        (5, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            cv2.putText(dummy, "r:recenter g:grip q:quit +/-:scale",
                        (5, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
            cv2.imshow("VIO Control", dummy)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord('r'):
                T_cur, _, _, _, _, _ = vio.get_pose()
                if T_cur is not None:
                    T_iphone_ref = T_cur.copy()
                    T_iphone_ref_inv = np.linalg.inv(T_iphone_ref)
                    tar_pos = home_pos.copy()
                    tar_quat = home_quat.copy()
                    smooth_pos = home_pos.copy()
                    smooth_quat = home_quat.copy()
                    clutch_base_T = T_home_ee.copy()
                    print(f">>> Recenter! 回到 home")
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
        print("\n停止遥操")
    finally:
        print("保持当前位置...")
        for _ in range(100):
            hexarm_client.set_cmds(cmds)
            time.sleep(0.002)
        vio.close()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
