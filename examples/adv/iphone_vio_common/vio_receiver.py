#!/usr/bin/env python3
"""
iPhone VIO UDP 接收器

接收 iPhone ARKit 通过 UDP 发送的 4x4 相机变换矩阵。
Swift 端以列优先 (column-major) 格式发送 16 个 float32 = 64 字节。

使用:
    receiver = iPhoneVIOReceiver(port=5005)
    T, ts, count = receiver.get_pose()
    # T: 4x4 numpy array (float64), ts: float (time.time()), count: int
"""

import socket
import threading
import time
import numpy as np


class iPhoneVIOReceiver:
    """线程安全的 UDP 接收器，后台线程持续接收 iPhone ARKit 位姿数据。"""

    def __init__(self, port=5005):
        self.port = port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", port))
        self.sock.setblocking(False)

        self._lock = threading.Lock()
        self._latest_matrix = None  # 4x4 np.ndarray (float64)
        self._latest_timestamp = 0.0  # time.time()
        self._recv_count = 0
        self._gripper_value = 0.0  # 0.0=全开, 1.0=全闭
        self._clutch_active = False  # clutch 是否激活

        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, daemon=True)
        self._thread.start()

        print(f"VIO Receiver 监听 0.0.0.0:{port}")

    def _recv_loop(self):
        while self._running:
            try:
                data, addr = self.sock.recvfrom(256)
                if len(data) >= 64:  # 16 floats (matrix) + optional 1 float (gripper)
                    floats = np.frombuffer(data, dtype=np.float32).copy()
                    mat = floats[:16].reshape(4, 4)
                    T = mat.T.astype(np.float64)

                    gripper = float(floats[16]) if len(floats) > 16 else 0.0
                    clutch = float(floats[17]) > 0.5 if len(floats) > 17 else True

                    with self._lock:
                        self._latest_matrix = T
                        self._latest_timestamp = time.time()
                        self._recv_count += 1
                        self._gripper_value = np.clip(gripper, 0.0, 1.0)
                        self._clutch_active = clutch
            except BlockingIOError:
                time.sleep(0.001)
            except Exception as e:
                print(f"VIO recv error: {e}")
                time.sleep(0.01)

    def get_pose(self):
        """
        获取最新位姿和夹爪值。

        Returns:
            (T, timestamp, count, gripper):
                T: 4x4 变换矩阵 (float64)，None 表示还没收到数据
                timestamp: 接收时间 (time.time())
                count: 累计接收帧数
                gripper: 夹爪开合度 0.0(全开)~1.0(全闭)
        """
        with self._lock:
            if self._latest_matrix is not None:
                return self._latest_matrix.copy(), self._latest_timestamp, self._recv_count, self._gripper_value, self._clutch_active
            return None, 0.0, 0, 0.0, False

    def close(self):
        self._running = False
        self.sock.close()


if __name__ == "__main__":
    # 独立测试：打印接收到的位姿
    receiver = iPhoneVIOReceiver(port=5005)
    print("等待 iPhone 数据... (Ctrl+C 退出)")
    last_count = 0
    try:
        while True:
            T, ts, count = receiver.get_pose()
            if count > last_count:
                pos = T[:3, 3]
                # 检查旋转矩阵正交性
                R = T[:3, :3]
                det = np.linalg.det(R)
                print(f"[{count:5d}] pos=[{pos[0]:+.3f} {pos[1]:+.3f} {pos[2]:+.3f}]  "
                      f"det(R)={det:.4f}  dt={time.time()-ts:.3f}s")
                last_count = count
            time.sleep(0.05)
    except KeyboardInterrupt:
        print("\n退出")
    finally:
        receiver.close()
