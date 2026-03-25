#!/usr/bin/env python3
"""
快速测试 SO-101 Leader 读数 + FK 输出是否合理。
不需要启动 MuJoCo，只需要 SO-101 硬件连接。

使用方法:
    source hex_zmq_servers/.venv/bin/activate
    python hex_zmq_servers/examples/adv/so101_sim/test_so101_read.py

按 Ctrl+C 退出。
"""

import time
import sys
import os
import numpy as np

# Add project root to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

import scservo_sdk as scs
import pinocchio as pin

# ============================================================
# 配置 — 根据你的实际情况修改
# ============================================================
DEVICE = "/dev/ttyACM0"      # SO-101 串口
BAUDRATE = 1_000_000         # STS3215 默认波特率
MOTOR_IDS = [1, 2, 3, 4, 5, 6]
MOTOR_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex",
               "wrist_flex", "wrist_roll", "gripper"]

# # Midpoints of [range_min, range_max] from LeRobot calibration
# # (homing_offset already applied in motor EEPROM, so we center using range midpoints)
# # Calibration: shoulder_pan [663,3395], shoulder_lift [835,3215], elbow_flex [827,3050],
# #              wrist_flex [926,3265], wrist_roll [0,4095], gripper [1436,2703]
# RANGE_MIDPOINTS = [2029, 2025, 1938, 2095, 2047, 2069]  # leader

# ============================================================
# 标定 midpoints: (range_min + range_max) / 2
# homing_offset 已写入电机 EEPROM，这里只需 range 中点
# ============================================================
# Leader:   [2029, 2025, 1938, 2095, 2047, 2069]
# Follower: [2095, 2055, 1963, 2025, 2047, 2211]
RANGE_MIDPOINTS = [2029, 2025, 1938, 2095, 2047, 2069]  # ← 当前: leader

# SO-101 URDF
URDF_PATH = os.path.join(os.path.dirname(__file__),
                          '..', '..', '..', 'hex_zmq_servers',
                          'robot', 'so101', 'urdf', 'so101.urdf')

# Feetech register
SCS_PRESENT_POSITION_ADDR = 56
SCS_PRESENT_POSITION_LEN = 2
SERVO_TO_RAD = np.pi / 2048  # 4096 steps/rev


def decode_sign_magnitude(raw):
    if raw & 0x8000:
        return -(raw & 0x7FFF)
    return raw


def main():
    # ---- 打开串口 ----
    port = scs.PortHandler(DEVICE)
    pkt = scs.PacketHandler(0)  # Protocol 0 for STS3215

    if not port.openPort():
        print(f"无法打开串口 {DEVICE}")
        print("请检查:")
        print(f"  1. SO-101 是否通过 USB 连接到电脑")
        print(f"  2. 串口设备是否存在: ls {DEVICE}")
        print(f"  3. 权限是否正确: sudo chmod 666 {DEVICE}")
        return

    if not port.setBaudRate(BAUDRATE):
        print(f"无法设置波特率 {BAUDRATE}")
        return

    # ---- 初始化 sync read ----
    sync_read = scs.GroupSyncRead(port, pkt,
                                   SCS_PRESENT_POSITION_ADDR,
                                   SCS_PRESENT_POSITION_LEN)
    for mid in MOTOR_IDS:
        sync_read.addParam(mid)

    # ---- 加载 SO-101 FK ----
    urdf_path = os.path.abspath(URDF_PATH)
    if not os.path.exists(urdf_path):
        print(f"URDF 不存在: {urdf_path}")
        return

    model = pin.buildModelFromUrdf(urdf_path)
    data = model.createData()
    ee_frame_id = model.getFrameId("gripper_frame_link")
    print(f"SO-101 FK loaded: {model.nq} DOF")
    print(f"URDF: {urdf_path}")
    print(f"串口: {DEVICE} @ {BAUDRATE}")
    print(f"Range midpoints: {RANGE_MIDPOINTS}")
    print("=" * 70)
    print("开始读取... (Ctrl+C 退出)")
    print()

    midpoints = np.array(RANGE_MIDPOINTS)

    try:
        while True:
            result = sync_read.txRxPacket()
            if result != scs.COMM_SUCCESS:
                print(f"通信失败: {result}")
                time.sleep(0.1)
                continue

            raw_values = []
            for mid in MOTOR_IDS:
                if sync_read.isAvailable(mid, SCS_PRESENT_POSITION_ADDR,
                                          SCS_PRESENT_POSITION_LEN):
                    raw = sync_read.getData(mid, SCS_PRESENT_POSITION_ADDR,
                                            SCS_PRESENT_POSITION_LEN)
                    raw_values.append(decode_sign_magnitude(raw))
                else:
                    raw_values.append(0)
                    print(f"  Motor {mid} 无数据!")

            raw_arr = np.array(raw_values)
            centered = raw_arr - midpoints
            radians = centered * SERVO_TO_RAD

            # FK (only first 5 joints = arm, skip gripper)
            q = np.zeros(model.nq)
            q[:5] = radians[:5]
            pin.forwardKinematics(model, data, q)
            pin.updateFramePlacement(model, data, ee_frame_id)
            T = data.oMf[ee_frame_id].homogeneous
            ee_pos = T[:3, 3]

            # 打印
            degrees = np.degrees(radians)
            joint_strs = [f"{MOTOR_NAMES[i]}={degrees[i]:+7.1f}°({radians[i]:+5.2f}rad)" for i in range(6)]
            print(f"  {' | '.join(joint_strs[:3])}")
            print(f"  {' | '.join(joint_strs[3:])}")
            print(f"  EE pos: x={ee_pos[0]:.4f} y={ee_pos[1]:.4f} z={ee_pos[2]:.4f} (m)  raw={raw_arr.tolist()}")
            print()

            time.sleep(0.1)  # 10 Hz display

    except KeyboardInterrupt:
        print("\n退出")
    finally:
        port.closePort()


if __name__ == "__main__":
    main()
