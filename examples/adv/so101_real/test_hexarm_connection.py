#!/usr/bin/env python3
"""最小测试：诊断 HexArm 命令是否被执行。"""

import sys, os, time, argparse
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')))

from hex_zmq_servers import HexRate, HexRobotHexarmClient, HEXARM_URDF_PATH_DICT
from hex_robo_utils import HexDynUtil as DynUtil

REMOTE_IP = "10.102.211.210"
LOCAL_IP = "127.0.0.1"

parser = argparse.ArgumentParser()
group = parser.add_mutually_exclusive_group()
group.add_argument("--remote", action="store_true")
group.add_argument("--local", action="store_true")
args = parser.parse_args()

hexarm_ip = REMOTE_IP if args.remote else LOCAL_IP
print(f"连接 HexArm: {hexarm_ip}:12345")

# HexRobotHexarmClient 构造函数里已经调用 _wait_for_working()
# 它会: 等待连接 → seq_clear → 启动后台线程
client = HexRobotHexarmClient(net_config={
    "ip": hexarm_ip, "port": 12345,
    "client_timeout_ms": 200,
})
print("已连接!")

# 诊断 1: 读状态
print("\n--- 诊断 1: 读状态 ---")
time.sleep(0.5)
for i in range(5):
    hdr, states = client.get_states()
    if hdr is not None and states is not None:
        print(f"  [OK] states shape={states.shape}, q={np.degrees(states[:, 0]).round(1)}")
        break
    print(f"  尝试 {i+1}/5: get_states 返回 None")
    time.sleep(0.5)
else:
    print("  [FAIL] 无法读取状态!")

# 诊断 2: 直接用 request() 发 set_cmds 看返回值
print("\n--- 诊断 2: 直接 request set_cmds ---")
home = np.array([0, -0.785, 2.2, 0.5, 0, 0, 0.0])
try:
    resp_hdr, resp_buf = client.request(
        {"cmd": "set_cmds", "ts": {"s": 0, "ns": 0}, "args": 0},
        home
    )
    print(f"  response: {resp_hdr}")
except Exception as e:
    print(f"  exception: {e}")

# 诊断 3: 连续发命令，检查 _cmds_seq 是否递增
print("\n--- 诊断 3: 发 100 条命令，检查 seq ---")
seq_before = client._cmds_seq
for _ in range(100):
    client.set_cmds(home)
    time.sleep(0.01)
seq_after = client._cmds_seq
print(f"  _cmds_seq: {seq_before} → {seq_after} (delta={seq_after - seq_before})")
if seq_after == seq_before:
    print("  [FAIL] seq 没增加，说明命令没被后台线程发送!")
    print(f"  _recv_flag = {client._recv_flag}")
    print(f"  _recv_thread alive = {client._recv_thread.is_alive()}")
else:
    print(f"  [OK] seq 增加了 {seq_after - seq_before}，命令正在发送")

# 诊断 4: 持续发 3 秒看是否动
print("\n--- 诊断 4: 持续发 home 命令 3 秒 ---")
dyn_util = DynUtil(HEXARM_URDF_PATH_DICT["archer_l6y_gp100"], "link_6")

hdr, states_before = client.get_states()
if states_before is not None:
    q_before = states_before[:, 0].copy()
    print(f"  发之前: {np.degrees(q_before).round(1)}°")

last_tau = np.zeros(7)
for _ in range(1500):
    hdr, states = client.get_states()
    if states is not None:
        arm_q = states[:, 0][:-1]
        arm_dq = states[:, 1][:-1]
        _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
        last_tau[:-1] = c_mat @ arm_dq + g_vec
    cmds = np.concatenate((home.reshape(-1, 1), last_tau.reshape(-1, 1)), axis=1)
    client.set_cmds(cmds)
    time.sleep(0.002)

time.sleep(0.5)
hdr, states_after = client.get_states()
if states_after is not None:
    q_after = states_after[:, 0].copy()
    print(f"  发之后: {np.degrees(q_after).round(1)}°")
    diff = np.degrees(q_after - q_before) if q_before is not None else None
    if diff is not None:
        print(f"  变化量: {diff.round(1)}°")
        if np.max(np.abs(diff[:6])) > 0.5:
            print("  ✓ Robot 动了!")
        else:
            print("  ✗ Robot 没动。问题在 server 到硬件之间。")

print("\nseq 最终值:", client._cmds_seq)
