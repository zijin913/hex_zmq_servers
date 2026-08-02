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
    clutch  = rightGrip / leftGrip   (握把，握住时遥操生效；阈值 0.5)
    gripper = RTr / LTr   (食指扳机 0.0~1.0，扣下=夹爪闭合)
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
        # Key mapping — matches the teleop docstring (Grip=clutch, Trigger=gripper).
        # PREVIOUSLY these were swapped: clutch read the Trigger and the gripper read
        # the Grip. The gripper-on-Grip mapping made the gripper command track the
        # analog Grip force, which is also held for clutch, so its natural wobble
        # jittered the gripper 0<->1 (POSITION mode's velocity limit hid it; MIT
        # exposed it as shake). Now: clutch = Grip (held to teleop), gripper =
        # Trigger (index, linear analog — good for a proportional gripper).
        #
        # SAME PHYSICAL BUTTONS, different readouts. The APK reports the index
        # trigger TWICE: 'RTr'/'LTr' is the BOOLEAN press (buttons_parser.py:11)
        # and 'rightTrig'/'leftTrig' is the ANALOG pull in [0,1]
        # (buttons_parser.py:16). The gripper bound the boolean, so despite the
        # comment above promising "linear analog" the command was only ever 0 or
        # 1, softened by the EMA below into a ramp between two endpoints — not a
        # proportional grip. Bind the analog readout of the SAME trigger.
        # The clutch deliberately stays BINARY: it is an on/off engage, and
        # clutch_thresh below is what makes it so.
        self.clutch_key = clutch_key or ("rightGrip" if hand == "right" else "leftGrip")
        self.gripper_key = gripper_key or ("rightTrig" if hand == "right" else "leftTrig")
        # Clutch hysteresis: engage above 0.6, release below 0.4. A single
        # 0.5 threshold let a grip resting near it flip at the Quest frame
        # rate — under hold-to-intervene every flip is a policy<->human
        # authority handoff, so chatter here is chatter on the arms.
        self.clutch_on_thresh = 0.6    # analog Grip must exceed this to ENGAGE
        self.clutch_off_thresh = 0.4   # ... and drop below this to RELEASE
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
        # Last GOOD button values. logcat parses intermittently drop a field (the
        # 150 Hz teleop loop reads faster than the ~72 Hz Quest stream), and a
        # missing field must be read as "no update", NOT as 0 -- returning 0 made
        # the gripper command blink to open, which the fast MIT gripper chased as
        # shake. We hold the last value on a dropout; a field that is PRESENT and 0
        # is a genuine release and passes through.
        self._last_gripper = 0.0
        self._last_clutch = False
        self._last_home = False
        # Low-pass on the analog gripper (trigger). The trigger is a noisy analog
        # signal and a human finger's force wobbles -- worst while gripping an
        # object (the finger is pressing hard). Unfiltered, that wobble drove the
        # gripper command 0.1<->1.0 and the fast MIT gripper chased it as shake.
        # An EMA smooths the command while keeping it continuous (proportional).
        # alpha in (0,1]: lower = smoother/laggier. 1.0 = no filter.
        self._grip_ema = 0.0
        self._grip_alpha = 0.25
        # Endpoint conditioning for the ANALOG trigger. A physical trigger rarely
        # spans exactly [0,1]: it idles a hair above 0 and often tops out just
        # below 1. That did not matter while this read the boolean (0 or 1
        # exactly), but on the analog readout it does, in both directions:
        #   - an idle offset creeps the gripper closed with nobody touching it;
        #   - a top-out below 1.0 means a FULL pull never commands a full close,
        #     which presents exactly as "the gripper will not close".
        # So rescale [deadband, saturate] onto [0,1] and clip. Defaults are near
        # no-ops for a clean trigger; widen them if `--probe-trigger` (see
        # __main__) shows this controller does not reach the endpoints.
        self._grip_deadband = 0.02   # at/below this -> fully open
        self._grip_saturate = 0.98   # at/above this -> fully closed
        self._last_grip_raw = 0.0    # last RAW trigger reading (diagnostics)

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
            # dropout: no fresh pose. hold last button values (teleop skips the
            # tick on T is None anyway, but keep them coherent for any caller).
            return (None, 0.0, self._recv_count,
                    self._last_gripper, self._last_clutch, self._last_home)

        T = np.asarray(transforms[self._tkey], dtype=np.float64).copy()

        # 指纹判断是否是新帧 (OculusReader 只在新数据到来时刷新字典，没有 count)
        sig = (float(T[0, 3]), float(T[1, 3]), float(T[2, 3]), float(T[0, 0]))
        with self._lock:
            if sig != self._last_signature:
                self._recv_count += 1
                self._last_signature = sig
            count = self._recv_count

        # 按键解析 — a field PRESENT in this parse is a real reading (0 = released);
        # a field ABSENT is a dropout -> hold the last value (don't read as 0).
        if self.gripper_key in buttons:
            _raw = float(np.clip(_extract_analog(buttons[self.gripper_key]), 0.0, 1.0))
            self._last_grip_raw = _raw   # pre-conditioning, for --probe-trigger
            # Rescale the trigger's usable span onto a full [0,1] so a released
            # trigger is exactly open and a fully-pulled one is exactly closed.
            _span = max(self._grip_saturate - self._grip_deadband, 1e-6)
            _raw = float(np.clip((_raw - self._grip_deadband) / _span, 0.0, 1.0))
            # EMA low-pass to reject finger/analog wobble (esp. while gripping).
            self._grip_ema += self._grip_alpha * (_raw - self._grip_ema)
            gripper = self._grip_ema
            self._last_gripper = gripper
        else:
            gripper = self._last_gripper
        # clutch reads the analog Grip; threshold so a light touch / wobble doesn't
        # engage or chatter. Hold on dropout.
        if self.clutch_key in buttons:
            _g = _extract_analog(buttons[self.clutch_key])
            _th = self.clutch_off_thresh if self._last_clutch else self.clutch_on_thresh
            clutch = _g > _th
            self._last_clutch = clutch
        else:
            clutch = self._last_clutch
        if self.home_key in buttons:
            home = bool(buttons[self.home_key])
            self._last_home = home
        else:
            home = self._last_home

        return T, time.time(), count, gripper, clutch, home

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
    parser.add_argument("--probe-trigger", action="store_true",
                        help="Measure the trigger's RAW endpoints. Release it fully, "
                             "then pull it fully, and read off min/max. If min > "
                             "_grip_deadband or max < _grip_saturate, widen those.")
    args = parser.parse_args()

    reader = QuestReader(hand=args.hand, ip_address=args.ip)

    if args.probe_trigger:
        print(f"探测扳机端点 (key={reader.gripper_key})  —  先完全松开，再完全扣死。Ctrl+C 结束")
        lo, hi = 1.0, 0.0
        try:
            while True:
                reader.get_pose()
                r = reader._last_grip_raw
                lo, hi = min(lo, r), max(hi, r)
                print(f"\r  raw={r:.4f}   观测到 min={lo:.4f}  max={hi:.4f}", end="", flush=True)
                time.sleep(0.03)
        except KeyboardInterrupt:
            print(f"\n\n  min={lo:.4f}  max={hi:.4f}")
            print(f"  当前 deadband={reader._grip_deadband}  saturate={reader._grip_saturate}")
            if lo > reader._grip_deadband:
                print(f"  ⚠ 松开时 raw 不为 0 → 把 _grip_deadband 调到 ≥ {lo:.3f}，否则夹爪会自己爬")
            if hi < reader._grip_saturate:
                print(f"  ⚠ 扣死时 raw 到不了 1 → 把 _grip_saturate 调到 ≤ {hi:.3f}，否则永远合不拢")
            if lo <= reader._grip_deadband and hi >= reader._grip_saturate:
                print("  ✓ 端点正常，默认值不用改")
        finally:
            reader.close()
        raise SystemExit(0)

    print("等待 Quest 数据... (Ctrl+C 退出)")
    print("  移动控制器 → 看 pos 变化")
    print("  握紧握把(clutch=ON) / 扣食指扳机(gripper 0.00→1.00 连续) / 按 A/X(home=True)")

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