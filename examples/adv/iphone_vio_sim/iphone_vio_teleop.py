#!/usr/bin/env python3
"""
iPhone VIO → MuJoCo HexArm 笛卡尔空间遥操

iPhone ARKit 通过 UDP 发送 6-DOF 位姿 → 笛卡尔增量映射 → 解析 IK → MuJoCo 仿真。
参考 joy_sim/cli.py 的笛卡尔 IK 控制架构。

使用方法:
    # 终端 1: 启动 MuJoCo 仿真
    source hex_zmq_servers/.venv/bin/activate
    python hex_zmq_servers/examples/basic/mujoco_archer_l6y/launch.py

    # 终端 2: 运行此脚本
    source hex_zmq_servers/.venv/bin/activate
    python hex_zmq_servers/examples/adv/iphone_vio_sim/iphone_vio_teleop.py

    # iPhone: 打开 CameraPoseTracker app（确保 IP 指向本机）

键盘控制 (焦点在 OpenCV 窗口):
    q: 退出
    r: 重新居中（当前 iPhone 位姿 → 新参考点）
    g: 切换夹爪开/合
    +/-: 调整位置缩放
    p: position+rotation 模式
    o: position-only 模式
    b: rotation-only 模式
"""

import os
import sys
import time
import numpy as np
import cv2

SODA_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
sys.path.insert(0, SODA_ROOT)
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from hex_zmq_servers import HexRate, HexMujocoArcherL6YClient, HEXARM_URDF_PATH_DICT
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

MUJOCO_PORT = 12345
VIO_PORT = 5005

# 映射参数
POSITION_SCALE = 1.2       # iPhone 位移缩放（<1 降低敏感度）
ROTATION_SCALE = 1.0       # iPhone 旋转缩放
INTERP_ERR_LIMIT = 0.01   # interp_joint 每帧最大关节变化 (rad), @100Hz ≈ 286°/s
POS_DEADZONE = 0.002      # 位置死区 (m)，8mm
ROT_DEADZONE = 0.07        # 旋转死区，小于此角度的旋转忽略

# 丢包超时
DROPOUT_TIMEOUT = 0.5      # 秒，超过此时间冻结
RECENTER_TIMEOUT = 3.0     # 秒，超过此时间自动 recenter

# iPhone (Y-up) → HexArm (Z-up) 坐标系映射
# R_REMAP = np.array([
#     [0, 0, -1],   # HexArm X ← -iPhone Z
#     [-1, 0, 0],   # HexArm Y ← -iPhone X
#     [0, 1, 0],    # HexArm Z ← +iPhone Y
# ], dtype=np.float64)

R_REMAP = np.array([
    [-1, 0, 0],  
    [0, 0, -1],  
    [0, -1, 0], 
], dtype=np.float64)


def orthogonalize(R):
    """确保旋转矩阵正交 (Gram-Schmidt)"""
    U, _, Vt = np.linalg.svd(R)
    R_orth = U @ Vt
    if np.linalg.det(R_orth) < 0:
        U[:, -1] *= -1
        R_orth = U @ Vt
    return R_orth


def rot_to_quat(R):
    """3x3 rotation matrix → [qw, qx, qy, qz]"""
    import pinocchio as pin
    R_orth = orthogonalize(R)
    q = pin.Quaternion(R_orth)
    return np.array([q.w, q.x, q.y, q.z])


def interp_joint(cur_q, tar_q, err_limit=0.1):
    """速度限幅关节插值 (来自 joy_sim/cli.py)"""
    err = tar_q - cur_q
    max_err = np.fabs(err).max()
    if max_err < err_limit:
        return tar_q.copy(), False
    else:
        return cur_q + (err / max_err) * err_limit, True


def interp_arm(cur_q, tar_joint, grip_val, err_limit=0.1):
    """手臂 + 夹爪插值"""
    mid = np.zeros(7)
    mid[:6], interp_flag = interp_joint(cur_q[:6], tar_joint, err_limit=err_limit)
    mid[6], _ = interp_joint(np.array([cur_q[6]]), np.array([grip_val]), err_limit=err_limit)
    return mid, interp_flag


def main():
    # ---- 初始化 IK solver ----
    # 用 link_6 frame, 无 TCP offset
    END_POSE_LINK6 = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    solver = HexDynUtilL6Y(HEXARM_URDF, "link_6", end_pose=END_POSE_LINK6)
    print(f"Analytic IK solver loaded: {HEXARM_URDF}")

    # ---- 初始化 VIO 接收器 ----
    vio = iPhoneVIOReceiver(port=VIO_PORT)

    # ---- 连接 MuJoCo ----
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

    # ---- Smoothstep 回 home (5 秒) ----
    print("\n平滑移动到 home (5秒)...")
    tau_comp = np.zeros(7)

    # 读当前位置
    start_q = np.zeros(7)
    start_q[:6] = HEXARM_HOME  # fallback
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
    print(f"  当前: {np.degrees(start_q[:6]).round(1)}°")
    print(f"  目标: {np.degrees(target_q[:6]).round(1)}°")

    n_steps = 500  # 5 sec @ 100Hz
    for i in range(n_steps):
        t = i / n_steps
        alpha = 3 * t * t - 2 * t * t * t  # smoothstep
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
    print(f"HexArm home EE: pos={home_pos.round(4)}, quat={home_quat.round(4)}")

    # ---- 等待 iPhone 数据 ----
    print(f"\n等待 iPhone VIO 数据 (UDP:{VIO_PORT})...")
    print("请打开 iPhone 上的 CameraPoseTracker app。")
    while True:
        T_iphone, ts, cnt, _, _, _ = vio.get_pose()
        if T_iphone is not None:
            print(f"收到 iPhone 数据! (已接收 {cnt} 帧)")
            break
        time.sleep(0.1)

    # ---- 记录 iPhone 参考位姿 ----
    T_iphone_ref = T_iphone.copy()
    T_iphone_ref_inv = np.linalg.inv(T_iphone_ref)
    print(f"iPhone 参考位姿已记录: pos={T_iphone_ref[:3,3].round(3)}")

    # ---- 控制状态 ----
    tar_joint = HEXARM_HOME.copy()
    tar_pos = home_pos.copy()
    tar_quat = home_quat.copy()
    smooth_pos = home_pos.copy()    # 低通滤波后的目标位置
    smooth_quat = home_quat.copy()  # 低通滤波后的目标旋转
    grip_flag = False
    last_cmd_q = np.zeros(7)        # 上一帧发送的命令（用作 IK seed）
    last_cmd_q[:6] = HEXARM_HOME
    last_cmd_q[6] = 0.2
    cur_q = last_cmd_q.copy()
    tau_comp = np.zeros(7)
    mode = "both"  # position + rotation
    pos_scale = POSITION_SCALE
    last_recv_count = 0
    last_recv_time = time.time()
    was_clutch_active = False
    clutch_base_T = T_home_ee.copy()  # clutch 按下时的机械臂 EE 位姿
    SMOOTH_ALPHA = 0.05   # 低通滤波系数 (0=冻结, 1=无滤波, 0.3=适中)

    print()
    print("=" * 60)
    print("iPhone VIO 遥操已启动!")
    print(f"  位置缩放: {pos_scale:.1f}x")
    print(f"  模式: {mode}")
    print("  按 r 重新居中 | g 夹爪 | +/- 缩放 | p/o/b 模式 | q 退出")
    print("=" * 60)
    print()

    # ---- 控制循环 ----
    rate = HexRate(100)  # 仿真 100Hz
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

                # Clutch 状态变化检测
                if vio_clutch and not was_clutch_active:
                    # 刚按下 clutch → recenter iPhone + 以当前机械臂位姿为基准
                    T_iphone_ref = T_iphone.copy()
                    T_iphone_ref_inv = np.linalg.inv(T_iphone_ref)
                    cur_fk_pos, cur_fk_quat = solver.forward_kinematics(last_cmd_q[:6])[-1]
                    clutch_base_T = part2trans(cur_fk_pos, cur_fk_quat)
                    if frame_count > 0:
                        print(f">>> Clutch ON — 从当前位置开始")
                elif not vio_clutch and was_clutch_active:
                    print(f">>> Clutch OFF — 冻结")
                was_clutch_active = vio_clutch

                # 只有 clutch 激活时才计算新的 delta
                if vio_clutch:
                    delta_T = T_iphone_ref_inv @ T_iphone
                    delta_pos_iphone = delta_T[:3, 3]
                    delta_rot_iphone = delta_T[:3, :3]

                    # 坐标系映射
                    cur_delta_pos = R_REMAP @ delta_pos_iphone * pos_scale
                    # 旋转映射：用户坐标轴 = 标准 ARKit 绕 X 转 -90° 再绕 Y 转 90°
                    # 先在 iPhone 空间里做坐标修正，再映射到 HexArm 空间
                    R_corr = np.array([
                        [ 0, -1,  0],
                        [ 0,  0,  1],
                        [-1,  0,  0],
                    ], dtype=np.float64)
                    delta_rot_corrected = R_corr @ delta_rot_iphone @ R_corr.T
                    mapped = R_REMAP @ delta_rot_corrected @ R_REMAP.T
                    # 映射后: pitch↔yaw 互换且三轴全反
                    # 修正: 交换 Y↔Z 并转置（反转方向）
                    swap_yz = np.array([
                        [1, 0, 0],
                        [0, 0, 1],
                        [0, 1, 0],
                    ], dtype=np.float64)
                    fixed = swap_yz @ mapped @ swap_yz
                    # 只有手机前后倾斜（绕 Y）方向反了
                    # 用旋转向量分解，翻转 Y 分量
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

                    # 目标 = clutch 按下时的位姿 + 当前 delta（纯相对）
                    if mode == "both":
                        tar_pos = clutch_base_T[:3, 3] + cur_delta_pos
                        tar_rot = clutch_base_T[:3, :3] @ cur_delta_rot
                        tar_quat = rot_to_quat(tar_rot)
                    elif mode == "pos":
                        tar_pos = clutch_base_T[:3, 3] + cur_delta_pos
                    elif mode == "rot":
                        tar_rot = clutch_base_T[:3, :3] @ cur_delta_rot
                        tar_quat = rot_to_quat(tar_rot)
                # else: clutch 未激活，tar_pos/tar_quat 保持不变（冻结）

            elif (now - last_recv_time) > DROPOUT_TIMEOUT:
                if frame_count % 500 == 0:
                    print(f"  [!] 数据丢失 {now - last_recv_time:.1f}s")

            # 3. 低通滤波目标位置（消除抖动）
            smooth_pos = smooth_pos * (1.0 - SMOOTH_ALPHA) + tar_pos * SMOOTH_ALPHA
            # 四元数用 slerp 近似：小增量时线性插值 + 归一化
            smooth_quat = smooth_quat * (1.0 - SMOOTH_ALPHA) + tar_quat * SMOOTH_ALPHA
            smooth_quat = smooth_quat / np.linalg.norm(smooth_quat)

            # 4. IK 求解（用上一帧命令作为 seed，保证分支连续性）
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
                # IK 失败：snap 回上一帧位姿
                tar_pos, tar_quat = solver.forward_kinematics(last_cmd_q[:6])[-1]
                smooth_pos = tar_pos.copy()
                smooth_quat = tar_quat.copy()

            # 5. 平滑插值（用 last_cmd_q 而非 cur_q，避免反馈振荡）
            # 夹爪：iPhone 音量键控制 (0~1 → 0.2~1.33)，键盘 g 键全开/全闭
            if grip_flag:
                grip_val = 1.33
            else:
                grip_val = 0.2 + vio_gripper * (1.33 - 0.2)  # 线性映射 0~1 → 0.2~1.33
            mid_joint, interp_flag = interp_arm(
                last_cmd_q, tar_joint, grip_val, err_limit=INTERP_ERR_LIMIT)
            last_cmd_q = mid_joint.copy()

            # 6. 发命令
            cmds = np.concatenate(
                (mid_joint.reshape(-1, 1), tau_comp.reshape(-1, 1)), axis=1)
            mujoco_client.set_cmds(cmds)

            # 6. 显示
            frame_count += 1
            if frame_count % 100 == 0:
                dp = tar_pos - T_home_ee[:3, 3]
                print(f"  [{mode:4s}] scale={pos_scale:.1f}x  "
                      f"delta=[{dp[0]:+.3f} {dp[1]:+.3f} {dp[2]:+.3f}]m  "
                      f"IK:{'OK' if ik_success else 'FAIL'}  "
                      f"grip={vio_gripper:.0%}→{grip_val:.2f}  "
                      f"clutch={'ON' if vio_clutch else 'OFF'}  "
                      f"VIO:{vio_count}")

            # 7. 摄像头
            rgb_hdr, rgb = mujoco_client.get_rgb()
            if rgb_hdr is not None:
                cv2.imshow("MuJoCo VIO Teleop", rgb)

            # 8. 键盘
            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord('r'):
                # Recenter
                T_iphone_cur, _, _, _, _, _ = vio.get_pose()
                if T_iphone_cur is not None:
                    T_iphone_ref = T_iphone_cur.copy()
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
                print(f">>> 模式: position + rotation")
            elif key == ord('o'):
                mode = "pos"
                print(f">>> 模式: position only")
            elif key == ord('b'):
                mode = "rot"
                print(f">>> 模式: rotation only")

            rate.sleep()

    except KeyboardInterrupt:
        print("\n退出")
    finally:
        vio.close()
        cv2.destroyAllWindows()
        print("Done.")


if __name__ == "__main__":
    main()
