#!/usr/bin/env python3
"""
Meta Quest 控制器 Reader

封装 rail-berkeley/oculus_reader，使其对外暴露与 iPhoneVIOReceiver 完全一致的
get_pose() 接口：(T, timestamp, count, gripper, clutch, home)。

数据来源: Quest 头显上运行 com.rail.oculus.teleop APK，通过 ADB logcat
推送控制器位姿和按键状态。第一次使用需要：
    1. adb 可用且 Quest 3 开启 USB 调试
    2. 安装 APK (git-lfs pull 取回 teleop-debug.apk，随后 OculusReader 构造时自动装)

按键映射 (在 __init__ 里可覆盖):
    clutch  = RTr / LTr   (食指扳机布尔值，按住时遥操生效)
    gripper = rightGrip / leftGrip   (握把扳机 0.0~1.0，握紧=夹爪闭合)
    home    = A / X       (右手 A，左手 X)

使用:
    reader = QuestReader(hand="right")       # USB 连接
    reader = QuestReader(hand="right",
                         ip_address="10.0.0.5")  # Wi-Fi 连接
    T, ts, count, grip, clutch, home = reader.get_pose()
"""

import os
import sys
import time
import threading

import numpy as np

# 允许直接运行本文件进行冒烟测试时能找到 third_party/oculus_reader
_THIRD_PARTY = os.path.abspath(
    os.path.join(os.path.dirname(__file__), "..", "..", "..", "..", "third_party", "oculus_reader")
)
if _THIRD_PARTY not in sys.path:
    sys.path.insert(0, _THIRD_PARTY)


class QuestReader:
    """
    与 iPhoneVIOReceiver 接口兼容的 Quest 控制器包装器。

    OculusReader 已经在内部维护了后台线程 + 锁读取 logcat，本类只做：
      1. 在 __init__ 阶段确保 APK 已安装、APK 已运行；
      2. 把 transforms/buttons 映射成 iPhone 那边的 6 元组返回；
      3. 维护一个 count 用于下游判断"是否有新数据"。
    """

    def __init__(
        self,
        hand: str = "right",
        ip_address: str | None = None,
        clutch_key: str | None = None,
        gripper_key: str | None = None,
        home_key: str | None = None,
        shared_reader=None,
    ):
        """
        Args:
            hand: "left" / "right"
            ip_address: Wi-Fi IP (None=USB/ADB)
            shared_reader: 已构造好的 OculusReader 实例（双臂场景复用同一个）；
                传入后忽略 ip_address。
        """
        assert hand in ("left", "right"), f"hand must be 'left' or 'right', got {hand}"
        self.hand = hand
        self._tkey = "r" if hand == "right" else "l"
        self._owns_reader = shared_reader is None

        # 按键默认映射
        self.clutch_key = clutch_key or ("RTr" if hand == "right" else "LTr")
        self.gripper_key = gripper_key or ("rightGrip" if hand == "right" else "leftGrip")
        self.home_key = home_key or ("A" if hand == "right" else "X")

        if shared_reader is not None:
            print(f"QuestReader 启动 (hand={hand}, 复用共享 OculusReader)")
            self._reader = shared_reader
        else:
            print(f"QuestReader 启动 (hand={hand}, transport={'Wi-Fi' if ip_address else 'USB/ADB'})")
            # 延迟导入: 只在真正构造 OculusReader 时才要求 ppadb 可用
            from oculus_reader.reader import OculusReader
            self._reader = OculusReader(ip_address=ip_address, run=True)

        self._lock = threading.Lock()
        self._recv_count = 0
        self._last_signature = None  # 用变换矩阵的部分元素做指纹，判断是否为新帧

    def get_pose(self):
        """
        Returns:
            (T, timestamp, count, gripper, clutch, home)
                T: 4x4 float64 ndarray，或 None (尚未收到数据)
                timestamp: 本地 time.time()
                count: 累计有效帧数（每次 transforms 更新时 +1）
                gripper: 0.0(全开) ~ 1.0(全闭)
                clutch: bool
                home:   bool
        """
        transforms, buttons = self._reader.get_transformations_and_buttons()

        if not transforms or self._tkey not in transforms:
            return None, 0.0, self._recv_count, 0.0, False, False

        T = np.asarray(transforms[self._tkey], dtype=np.float64).copy()

        # 指纹判断是否是新帧 (OculusReader 只在新数据到来时刷新字典，没有 count)
        sig = (float(T[0, 3]), float(T[1, 3]), float(T[2, 3]), float(T[0, 0]))
        with self._lock:
            if sig != self._last_signature:
                self._recv_count += 1
                self._last_signature = sig
            count = self._recv_count

        # 按键解析
        gripper = _extract_analog(buttons.get(self.gripper_key, 0.0))
        clutch = bool(buttons.get(self.clutch_key, False))
        home = bool(buttons.get(self.home_key, False))

        return T, time.time(), count, float(np.clip(gripper, 0.0, 1.0)), clutch, home

    def close(self):
        # 只有"自己构造"的 reader 才负责关闭；共享模式下交给 owner
        if self._owns_reader:
            try:
                self._reader.stop()
            except Exception as e:
                print(f"QuestReader.close() 异常: {e}")


def make_quest_reader_pair(ip_address: str | None = None):
    """
    双臂场景: 构造一个共享 OculusReader, 返回 (reader_left, reader_right)。
    两个 QuestReader 共用同一个 ADB logcat 流, 各自追踪自己那只手的位姿/按键。

    使用:
        reader_left, reader_right = make_quest_reader_pair()
        # 之后像使用两个独立 QuestReader 一样用就行
        # close 任意一个即可 (二者共享底层 reader, 由构造时指定 owner 关闭)
    """
    from oculus_reader.reader import OculusReader
    shared = OculusReader(ip_address=ip_address, run=True)
    reader_left = QuestReader(hand="left", shared_reader=shared)
    reader_right = QuestReader(hand="right", shared_reader=shared)
    # 让 left 负责 shutdown 共享 reader，right 只 close 自己状态
    reader_left._owns_reader = True
    reader_right._owns_reader = False
    return reader_left, reader_right


def _extract_analog(val) -> float:
    """oculus_reader 的 analog 值可能是 tuple((v,)) 或 float 或缺失 (0.0)."""
    if isinstance(val, tuple):
        return float(val[0]) if val else 0.0
    try:
        return float(val)
    except (TypeError, ValueError):
        return 0.0


if __name__ == "__main__":
    # 独立冒烟测试
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--hand", choices=["left", "right"], default="right")
    parser.add_argument("--ip", type=str, default=None, help="Wi-Fi IP (USB 若不指定)")
    args = parser.parse_args()

    reader = QuestReader(hand=args.hand, ip_address=args.ip)
    print("等待 Quest 数据... (Ctrl+C 退出)")
    print("  移动控制器 → 看 pos 变化")
    print("  按住扳机(clutch=True) / 握紧握把(gripper→1.0) / 按 A/X(home=True)")

    last_count = 0
    try:
        while True:
            T, ts, count, grip, clutch, home = reader.get_pose()
            if count != last_count and T is not None:
                pos = T[:3, 3]
                print(
                    f"[{count:5d}] pos=[{pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:+.3f}]  "
                    f"grip={grip:.2f}  clutch={'ON ' if clutch else 'off'}  "
                    f"home={'YES' if home else 'no '}"
                )
                last_count = count
            time.sleep(0.03)
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        reader.close()
