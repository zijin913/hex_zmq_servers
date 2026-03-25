#!/usr/bin/env python3
"""
SO-101 Leader → 真机 HexArm L6Y 关节空间遥操

与仿真版 (test_so101_to_mujoco.py) 映射逻辑完全一致，
只是把 MuJoCo 客户端换成真机 HexArm 客户端。

使用方法:
    # 终端 1: 启动 HexArm server
    source activate_env.sh
    python launchers/launch_servers.py

    # 终端 2: 运行遥操
    source hex_zmq_servers/.venv/bin/activate
    python hex_zmq_servers/examples/adv/so101_real/test_so101_to_hexarm.py --remote
    python hex_zmq_servers/examples/adv/so101_real/test_so101_to_hexarm.py --local

键盘控制 (终端输入):
    Enter: 确认开始遥操
    Ctrl+C: 安全停止
"""

import os
import sys
import time
import json
import argparse
import numpy as np
import scservo_sdk as scs

SODA_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
sys.path.insert(0, SODA_ROOT)

from hex_zmq_servers import HexRate, HexRobotHexarmClient, HEXARM_URDF_PATH_DICT
from hex_robo_utils import HexDynUtil as DynUtil

# ============================================================
# SO-101 硬件配置
# ============================================================
SO101_DEVICE = "/dev/ttyACM0"
SO101_BAUDRATE = 1_000_000
SO101_MOTOR_IDS = [1, 2, 3, 4, 5, 6]
RANGE_MIDPOINTS = np.array([2029, 2025, 1938, 2095, 2047, 2069])  # leader
SCS_PRESENT_POSITION_ADDR = 56
SCS_PRESENT_POSITION_LEN = 2
SERVO_TO_RAD = np.pi / 2048

# ============================================================
# 标定文件（共用仿真的标定）
# ============================================================
DEFAULT_SIGNS = [-1.0, 1.0, 1.0, 1.0, -1.0]
DEFAULT_SCALES = [1.0, 1.0, 1.0, 1.0, 1.0]
DEFAULT_GRIPPER_SCALE = 1.0

# 优先从 sim 目录加载标定
SIM_CALIB_FILE = os.path.join(os.path.dirname(__file__), '..', 'so101_sim', 'joint_mapping_calib.json')
LOCAL_CALIB_FILE = os.path.join(os.path.dirname(__file__), 'joint_mapping_calib.json')


def load_calib():
    for path in [LOCAL_CALIB_FILE, SIM_CALIB_FILE]:
        if os.path.exists(path):
            with open(path) as f:
                data = json.load(f)
            print(f"已加载标定文件: {path}")
            so101_ref_raw = data.get("so101_ref", None)
            so101_ref = np.array(so101_ref_raw) if so101_ref_raw is not None else None
            return (np.array(data.get("signs", DEFAULT_SIGNS)),
                    np.array(data.get("scales", DEFAULT_SCALES)),
                    data.get("gripper_scale", DEFAULT_GRIPPER_SCALE),
                    so101_ref)
    print("未找到标定文件，使用默认值")
    return np.array(DEFAULT_SIGNS), np.array(DEFAULT_SCALES), DEFAULT_GRIPPER_SCALE, None


# ============================================================
# HexArm 配置
# ============================================================
ARM_TYPE = "archer_l6y"
GRIPPER_TYPE = "gp100"
HEXARM_URDF = HEXARM_URDF_PATH_DICT[f"{ARM_TYPE}_{GRIPPER_TYPE}"]
HEXARM_HOME = np.array([0.0, -0.785, 2.2, 0.5, 0.0, 0.0])

HEX_LOWER = np.array([-2.86, -2.09, 0.0, -1.57, -1.57, -3.14])
HEX_UPPER = np.array([2.86, 1.57, 3.16, 1.57, 1.57, 3.14])

# 真机 HexArm server
REMOTE_IP = "10.102.211.210"  # SOMA robot
LOCAL_IP = "127.0.0.1"
HEXARM_PORT = 12345


def decode_sign_magnitude(raw):
    if raw & 0x8000:
        return -(raw & 0x7FFF)
    return raw


class SO101Reader:
    def __init__(self):
        self.port = scs.PortHandler(SO101_DEVICE)
        self.pkt = scs.PacketHandler(0)
        if not self.port.openPort():
            raise RuntimeError(f"无法打开 {SO101_DEVICE}")
        if not self.port.setBaudRate(SO101_BAUDRATE):
            raise RuntimeError(f"无法设置波特率 {SO101_BAUDRATE}")
        self.sync_read = scs.GroupSyncRead(
            self.port, self.pkt, SCS_PRESENT_POSITION_ADDR, SCS_PRESENT_POSITION_LEN)
        for mid in SO101_MOTOR_IDS:
            self.sync_read.addParam(mid)
        print(f"SO-101 连接: {SO101_DEVICE} @ {SO101_BAUDRATE}")

    def read(self):
        result = self.sync_read.txRxPacket()
        if result != scs.COMM_SUCCESS:
            return None
        raw = []
        for mid in SO101_MOTOR_IDS:
            if self.sync_read.isAvailable(mid, SCS_PRESENT_POSITION_ADDR, SCS_PRESENT_POSITION_LEN):
                v = self.sync_read.getData(mid, SCS_PRESENT_POSITION_ADDR, SCS_PRESENT_POSITION_LEN)
                raw.append(decode_sign_magnitude(v))
            else:
                return None
        centered = np.array(raw) - RANGE_MIDPOINTS
        return centered * SERVO_TO_RAD

    def close(self):
        self.port.closePort()


def main():
    parser = argparse.ArgumentParser()
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--remote", action="store_true", help=f"连接远程 robot ({REMOTE_IP})")
    group.add_argument("--local", action="store_true", help=f"连接本地 server ({LOCAL_IP})")
    args = parser.parse_args()

    hexarm_ip = REMOTE_IP if args.remote else LOCAL_IP
    print(f"HexArm server: {hexarm_ip}:{HEXARM_PORT} ({'remote' if args.remote else 'local'})")

    signs, scales, gripper_scale, so101_ref = load_calib()

    # ---- 初始化 SO-101 ----
    so101 = SO101Reader()
    dyn_util = DynUtil(HEXARM_URDF, "link_6")

    # ---- 连接真机 HexArm ----
    hexarm_client = HexRobotHexarmClient(net_config={
        "ip": hexarm_ip, "port": HEXARM_PORT,
        "realtime_mode": False, "deque_maxlen": 10,
        "client_timeout_ms": 200, "server_timeout_ms": 1000,
        "server_num_workers": 4,
    })

    print(f"等待 HexArm server ({hexarm_ip}:{HEXARM_PORT})...")
    # 构造函数里已经调用了 _wait_for_working()（连接+seq_clear+启动线程）
    print("HexArm 已连接!")

    # ---- 稳定读取 SO-101 ----
    print("等待 SO-101 稳定...")
    for _ in range(30):
        so101.read()
        time.sleep(0.02)

    # ---- so101_ref ----
    if so101_ref is not None:
        print(f"使用已保存的 so101_ref: {np.degrees(so101_ref).round(1)}°")
    else:
        print("⚠ 尚未标定 so101_ref！使用当前读数作为临时 ref。")
        print("  建议先在仿真中完成标定。")
        reading = so101.read()
        if reading is None:
            print("无法读取 SO-101！")
            so101.close()
            return
        so101_ref = reading[:5].copy()

    # ---- 安全确认 ----
    print()
    print("=" * 60)
    print("⚠ 即将控制真机 HexArm！")
    print(f"  signs:     {signs.astype(int).tolist()}")
    print(f"  scales:    {scales.round(2).tolist()}")
    print(f"  so101_ref: {np.degrees(so101_ref).round(1).tolist()}°")
    print()
    print("  真机将先移动到 home 位置，然后开始跟随 SO-101。")
    print("  请确保 HexArm 周围无障碍物。")
    print("  按 Enter 开始，Ctrl+C 随时停止。")
    print("=" * 60)
    input()

    # ---- 先平滑移到 home ----
    print("平滑移动 HexArm 到 home 位置 (5秒)...")
    last_tau = np.zeros(7)

    # 读取当前位置
    start_q = HEXARM_HOME.copy()  # fallback
    for _ in range(20):
        hexarm_hdr, hexarm_states = hexarm_client.get_states()
        if hexarm_states is not None:
            start_q = hexarm_states[:, 0].copy()
            arm_q = hexarm_states[:, 0][:-1]
            arm_dq = hexarm_states[:, 1][:-1]
            _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
            last_tau[:-1] = c_mat @ arm_dq + g_vec
            break
        time.sleep(0.05)

    target_q = np.zeros(7)
    target_q[:6] = HEXARM_HOME
    print(f"  当前: {np.degrees(start_q[:6]).round(1)}°")
    print(f"  目标: {np.degrees(target_q[:6]).round(1)}°")

    # 10 秒缓慢插值（用 smoothstep 而非线性，起止更平滑）
    n_steps = 5000  # 10 sec @ 500Hz
    for i in range(n_steps):
        t = i / n_steps  # 0 → 1
        alpha = 3 * t * t - 2 * t * t * t  # smoothstep: 起止速度为 0
        interp_q = start_q + alpha * (target_q - start_q)

        hexarm_hdr, hexarm_states = hexarm_client.get_states()
        if hexarm_states is not None:
            arm_q = hexarm_states[:, 0][:-1]
            arm_dq = hexarm_states[:, 1][:-1]
            _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
            last_tau[:-1] = c_mat @ arm_dq + g_vec

        cmds = np.concatenate((interp_q.reshape(-1, 1), last_tau.reshape(-1, 1)), axis=1)
        hexarm_client.set_cmds(cmds)
        time.sleep(0.002)
    print("HexArm 已到 home!")
    print("开始遥操... (Ctrl+C 停止)")
    print()

    # ---- 控制循环 ----
    j5_offset = 0.0
    # last_tau 已在 home 阶段初始化，继续复用
    last_cmd_q = target_q.copy()  # 上一帧发送的命令，用于平滑
    SMOOTHING = 0.05  # 低通滤波系数 (0~1, 越小越平滑, 0.05 ≈ 很柔和)
    MAX_JOINT_STEP = np.radians(0.5)  # 每帧最大变化 0.5°（@ 500Hz ≈ 250°/s，安全速度）
    rate = HexRate(500)  # 真机 500Hz
    frame_count = 0

    try:
        while True:
            so101_joints = so101.read()
            if so101_joints is None:
                rate.sleep()
                continue

            so101_arm = so101_joints[:5]
            so101_grip = so101_joints[5]
            so101_delta = so101_arm - so101_ref

            # 关节映射（和仿真完全一致）
            hex_q = np.zeros(7)
            hex_q[:6] = HEXARM_HOME.copy()
            # SO-101 joint 0-3 → HexArm J1-J4
            hex_q[:4] += signs[:4] * scales[:4] * so101_delta[:4]
            # SO-101 joint 4 (wrist_roll) → HexArm J6
            hex_q[5] += signs[4] * scales[4] * so101_delta[4]
            # HexArm J5 固定
            hex_q[4] = HEXARM_HOME[4] + j5_offset

            # 夹爪
            grip_normalized = (so101_grip + 0.97) / 1.94
            hex_q[6] = np.clip((1.0 - grip_normalized) * 1.33, 0.0, 1.33)

            # clamp
            hex_q[:6] = np.clip(hex_q[:6], HEX_LOWER, HEX_UPPER)

            # 双重平滑：低通滤波 + 速度限幅
            # 1. 指数平滑（低通滤波）：new = old * (1-α) + target * α
            smoothed_q = last_cmd_q * (1.0 - SMOOTHING) + hex_q * SMOOTHING
            # 2. 速度限幅：每帧最大变化量
            delta_cmd = smoothed_q - last_cmd_q
            delta_cmd[:6] = np.clip(delta_cmd[:6], -MAX_JOINT_STEP, MAX_JOINT_STEP)
            hex_q = last_cmd_q + delta_cmd
            last_cmd_q = hex_q.copy()

            # 发送（带重力补偿，有新状态就更新 tau，没有也继续发）
            hexarm_hdr, hexarm_states = hexarm_client.get_states()
            if hexarm_states is not None:
                arm_q = hexarm_states[:, 0][:-1]
                arm_dq = hexarm_states[:, 1][:-1]
                _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
                last_tau[:-1] = c_mat @ arm_dq + g_vec
            cmds = np.concatenate((hex_q.reshape(-1, 1), last_tau.reshape(-1, 1)), axis=1)
            hexarm_client.set_cmds(cmds)

            # 打印
            frame_count += 1
            if frame_count % 500 == 0:  # 每秒打印一次
                delta_deg = np.degrees(so101_delta)
                hex_deg = np.degrees(hex_q[:6])
                print(f"  delta: [{delta_deg[0]:+5.1f}° {delta_deg[1]:+5.1f}° "
                      f"{delta_deg[2]:+5.1f}° {delta_deg[3]:+5.1f}° {delta_deg[4]:+5.1f}°] "
                      f"→ hex: [{hex_deg[0]:+6.1f}° {hex_deg[1]:+6.1f}° {hex_deg[2]:+6.1f}° "
                      f"{hex_deg[3]:+6.1f}° {hex_deg[4]:+6.1f}° {hex_deg[5]:+6.1f}°]")

            rate.sleep()

    except KeyboardInterrupt:
        print("\n停止遥操")
    finally:
        # 停止时发送当前位置保持（不突然断力矩）
        print("保持当前位置...")
        hexarm_hdr, hexarm_states = hexarm_client.get_states()
        if hexarm_states is not None:
            hold_q = hexarm_states[:, 0]
            arm_q = hexarm_states[:, 0][:-1]
            arm_dq = hexarm_states[:, 1][:-1]
            _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
            tau_comp = np.concatenate((c_mat @ arm_dq + g_vec, np.zeros(1)))
            cmds = np.concatenate((hold_q.reshape(-1, 1), tau_comp.reshape(-1, 1)), axis=1)
            for _ in range(100):
                hexarm_client.set_cmds(cmds)
                time.sleep(0.002)

        so101.close()
        print("Done.")


if __name__ == "__main__":
    main()
