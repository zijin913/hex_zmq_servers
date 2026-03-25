#!/usr/bin/env python3
"""
SO-101 → MuJoCo HexArm 关节空间绝对映射遥操

直接关节映射（不用 FK/IK），和 Gello 同样的思路：
  hex_q[i] = HEXARM_HOME[i] + signs[i] * scales[i] * (so101_q[i] - so101_ref[i])

so101_ref 是标定常量（SO-101 对应 HexArm home 的姿态），保存在 calib 文件中。

使用方法:
    # 终端 1: 启动 MuJoCo 仿真
    source hex_zmq_servers/.venv/bin/activate
    python hex_zmq_servers/examples/basic/mujoco_archer_l6y/launch.py

    # 终端 2: 运行此脚本
    source hex_zmq_servers/.venv/bin/activate
    python hex_zmq_servers/examples/adv/so101_sim/test_so101_to_mujoco.py

键盘控制 (焦点在 MuJoCo 窗口上):
    1-5: 单关节测试模式（只映射该关节）
    0:   全关节映射模式
    f:   翻转当前关节的 sign（+1 ↔ -1）
    c:   标定 — 记录当前 SO-101 姿态为 ref（对应 HexArm home）
    q:   退出（自动保存标定）
"""

import os
import sys
import time
import json
import numpy as np
import scservo_sdk as scs

SODA_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
sys.path.insert(0, SODA_ROOT)

from hex_zmq_servers import HexRate, HexMujocoArcherL6YClient, HEXARM_URDF_PATH_DICT
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
# 映射参数默认值（会被 calibration 文件覆盖）
# ============================================================
DEFAULT_SIGNS = [-1.0, 1.0, 1.0, 1.0, -1.0]
DEFAULT_SCALES = [1.0, 1.0, 1.0, 1.0, 1.0]
DEFAULT_GRIPPER_SCALE = 1.0

# 标定文件路径（自动保存/加载）
CALIB_FILE = os.path.join(os.path.dirname(__file__), "joint_mapping_calib.json")


def load_calib():
    """加载标定文件，返回 (signs, scales, gripper_scale, so101_ref)"""
    if os.path.exists(CALIB_FILE):
        with open(CALIB_FILE) as f:
            data = json.load(f)
        print(f"已加载标定文件: {CALIB_FILE}")
        so101_ref_raw = data.get("so101_ref", None)
        so101_ref = np.array(so101_ref_raw) if so101_ref_raw is not None else None
        return (np.array(data.get("signs", DEFAULT_SIGNS)),
                np.array(data.get("scales", DEFAULT_SCALES)),
                data.get("gripper_scale", DEFAULT_GRIPPER_SCALE),
                so101_ref)
    return np.array(DEFAULT_SIGNS), np.array(DEFAULT_SCALES), DEFAULT_GRIPPER_SCALE, None


def save_calib(signs, scales, gripper_scale, so101_ref):
    """保存标定到文件"""
    data = {
        "signs": signs.tolist(),
        "scales": scales.tolist(),
        "gripper_scale": gripper_scale,
        "so101_ref": so101_ref.tolist() if so101_ref is not None else None,
    }
    with open(CALIB_FILE, "w") as f:
        json.dump(data, f, indent=2)
    print(f"标定已保存: {CALIB_FILE}")


# ============================================================
# HexArm 配置
# ============================================================
ARM_TYPE = "archer_l6y"
GRIPPER_TYPE = "gp100"
HEXARM_URDF = HEXARM_URDF_PATH_DICT[f"{ARM_TYPE}_{GRIPPER_TYPE}"]
HEXARM_HOME = np.array([0.0, -0.785, 2.2, 0.5, 0.0, 0.0])

HEX_LOWER = np.array([-2.86, -2.09, 0.0, -1.57, -1.57, -3.14])
HEX_UPPER = np.array([2.86, 1.57, 3.16, 1.57, 1.57, 3.14])

MUJOCO_PORT = 12345

JOINT_NAMES = ["shoulder_pan→J1", "shoulder_lift→J2", "elbow_flex→J3",
               "wrist_flex→J4", "wrist_roll→J5"]


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
    import cv2

    signs, scales, gripper_scale, so101_ref = load_calib()

    # ---- 初始化 ----
    so101 = SO101Reader()
    dyn_util = DynUtil(HEXARM_URDF, "link_6")

    mujoco_client = HexMujocoArcherL6YClient(net_config={
        "ip": "127.0.0.1", "port": MUJOCO_PORT,
        "realtime_mode": False, "deque_maxlen": 10,
        "client_timeout_ms": 200, "server_timeout_ms": 1000,
        "server_num_workers": 4,
    })

    print("等待 MuJoCo...")
    for _ in range(50):
        if mujoco_client.is_working():
            break
        time.sleep(0.1)
    else:
        print("MuJoCo 未响应！")
        so101.close()
        return
    print("MuJoCo 已连接!")

    # ---- 稳定读取 SO-101 ----
    print("等待 SO-101 稳定...")
    for _ in range(30):
        so101.read()
        time.sleep(0.02)

    # ---- so101_ref 处理 ----
    if so101_ref is not None:
        print(f"使用已保存的 so101_ref: {np.degrees(so101_ref).round(1)}°")
    else:
        print("⚠ 尚未标定 so101_ref！临时使用当前读数。")
        print("  请将 SO-101 摆到对应 HexArm home 的姿态，然后按 c 键标定。")
        reading = so101.read()
        if reading is None:
            print("无法读取 SO-101！")
            so101.close()
            return
        so101_ref = reading[:5].copy()

    # ---- 控制循环 ----
    mode = 0  # 0=全关节, 1-5=单关节
    j5_offset = 0.0  # J5 键盘增量（w/s 控制）
    J5_STEP = np.radians(3.0)  # 每次按键转 3°
    rate = HexRate(100)
    frame_count = 0

    print()
    print("=" * 60)
    print("关节绝对映射遥操已启动！")
    print(f"  signs:     {signs.astype(int).tolist()}")
    print(f"  scales:    {scales.round(2).tolist()}")
    print(f"  so101_ref: {np.degrees(so101_ref).round(1).tolist()}°")
    print("  按 0: 全关节  |  1-5: 单关节  |  f: 翻转sign  |  c: 标定ref")
    print("  按 a/d: J5 +/-3°  |  q: 退出")
    print("=" * 60)

    try:
        while True:
            so101_joints = so101.read()
            if so101_joints is None:
                rate.sleep()
                continue

            so101_arm = so101_joints[:5]
            so101_grip = so101_joints[5]

            # 绝对映射：delta 相对于标定的 ref（不是启动时读数）
            so101_delta = so101_arm - so101_ref

            # 关节映射
            hex_q = np.zeros(7)
            hex_q[:6] = HEXARM_HOME.copy()

            if mode == 0:
                # SO-101 joint 0-3 → HexArm J1-J4
                hex_q[:4] += signs[:4] * scales[:4] * so101_delta[:4]
                # SO-101 joint 4 (wrist_roll) → HexArm J6
                hex_q[5] += signs[4] * scales[4] * so101_delta[4]
                # HexArm J5 键盘控制 (w/s)
                hex_q[4] = HEXARM_HOME[4] + j5_offset
            else:
                i = mode - 1
                # 单关节模式: J5 键控制的是 SO-101 wrist_roll → HexArm J6
                if i == 4:
                    hex_q[5] += signs[4] * scales[4] * so101_delta[4]
                    hex_q[4] = HEXARM_HOME[4] + j5_offset
                else:
                    hex_q[i] += signs[i] * scales[i] * so101_delta[i]
            # 夹爪映射: SO-101 [-0.97, +0.97] → HexArm [1.33, 0.0]（反向）
            grip_normalized = (so101_grip + 0.97) / 1.94  # → [0, 1]
            hex_q[6] = np.clip((1.0 - grip_normalized) * 1.33, 0.0, 1.33)

            # clamp
            hex_q[:6] = np.clip(hex_q[:6], HEX_LOWER, HEX_UPPER)

            # 发送到 MuJoCo（带重力补偿）
            robot_hdr, robot_states = mujoco_client.get_states("robot")
            if robot_states is not None:
                arm_q = robot_states[:, 0][:-1]
                arm_dq = robot_states[:, 1][:-1]
                _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
                tau_comp = np.zeros(7)
                tau_comp[:-1] = c_mat @ arm_dq + g_vec
                cmds = np.concatenate((hex_q.reshape(-1, 1), tau_comp.reshape(-1, 1)), axis=1)
                mujoco_client.set_cmds(cmds)

            # 显示
            frame_count += 1
            if frame_count % 30 == 0:
                mode_str = "ALL" if mode == 0 else f"J{mode} only ({JOINT_NAMES[mode-1]})"
                delta_deg = np.degrees(so101_delta)
                hex_deg = np.degrees(hex_q[:6])
                so101_deg = np.degrees(so101_arm)
                print(f"  [{mode_str}] signs={signs.astype(int).tolist()}")
                print(f"    SO-101 abs:   [{so101_deg[0]:+6.1f}° {so101_deg[1]:+6.1f}° "
                      f"{so101_deg[2]:+6.1f}° {so101_deg[3]:+6.1f}° {so101_deg[4]:+6.1f}°]")
                print(f"    SO-101 delta:  [{delta_deg[0]:+5.1f}° {delta_deg[1]:+5.1f}° "
                      f"{delta_deg[2]:+5.1f}° {delta_deg[3]:+5.1f}° {delta_deg[4]:+5.1f}°]")
                print(f"    HexArm q:     [{hex_deg[0]:+6.1f}° {hex_deg[1]:+6.1f}° "
                      f"{hex_deg[2]:+6.1f}° {hex_deg[3]:+6.1f}° {hex_deg[4]:+6.1f}° "
                      f"{hex_deg[5]:+6.1f}°]")
                print()

            # 摄像头
            rgb_hdr, rgb = mujoco_client.get_rgb()
            if rgb_hdr is not None:
                cv2.imshow("MuJoCo", rgb)

            key = cv2.waitKey(1)
            if key == ord('q'):
                break
            elif key == ord('0'):
                mode = 0
                print(f">>> 模式: 全关节映射")
            elif key in [ord('1'), ord('2'), ord('3'), ord('4'), ord('5')]:
                mode = key - ord('0')
                print(f">>> 模式: 单关节 J{mode} ({JOINT_NAMES[mode-1]}), sign={signs[mode-1]:+.0f}")
            elif key == ord('f'):
                idx = (mode - 1) if mode > 0 else 0
                signs[idx] *= -1
                print(f">>> 翻转 J{idx+1} sign → {signs[idx]:+.0f}  (signs={signs.astype(int).tolist()})")
            elif key == ord('a'):
                j5_offset += J5_STEP
                print(f">>> J5 += 3° → {np.degrees(HEXARM_HOME[4] + j5_offset):+.1f}°")
            elif key == ord('d'):
                j5_offset -= J5_STEP
                print(f">>> J5 -= 3° → {np.degrees(HEXARM_HOME[4] + j5_offset):+.1f}°")
            elif key == ord('c'):
                # 标定：记录当前 SO-101 位姿为 ref
                reading = so101.read()
                if reading is not None:
                    so101_ref = reading[:5].copy()
                    print(f">>> 已标定 so101_ref: {np.degrees(so101_ref).round(1)}°")
                    print(f"    当前 SO-101 位姿 ↔ HexArm home [{np.degrees(HEXARM_HOME).round(1)}°]")

            rate.sleep()

    except KeyboardInterrupt:
        print("\n退出")
    finally:
        so101.close()
        cv2.destroyAllWindows()
        save_calib(signs, scales, gripper_scale, so101_ref)
        print("Done.")


if __name__ == "__main__":
    main()
