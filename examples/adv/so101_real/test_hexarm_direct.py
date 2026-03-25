#!/usr/bin/env python3
"""绕过 HexRobotHexarmClient，直接用 ZMQ socket 发命令"""

import sys, os, time, argparse, zmq
import numpy as np

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..')))
from hex_zmq_servers.zmq_base import hex_zmq_ts_now

REMOTE_IP = "10.102.211.210"
LOCAL_IP = "127.0.0.1"

parser = argparse.ArgumentParser()
group = parser.add_mutually_exclusive_group()
group.add_argument("--remote", action="store_true")
group.add_argument("--local", action="store_true")
args = parser.parse_args()

ip = REMOTE_IP if args.remote else LOCAL_IP
port = 12345

import json

ctx = zmq.Context()
sock = ctx.socket(zmq.REQ)
sock.setsockopt(zmq.RCVTIMEO, 2000)
sock.setsockopt(zmq.SNDTIMEO, 2000)
sock.connect(f"tcp://{ip}:{port}")
print(f"连接 {ip}:{port}")

def send_recv(hdr, buf=None):
    """发送请求，返回响应"""
    hdr_bytes = json.dumps(hdr).encode()
    if buf is not None:
        buf_bytes = np.ascontiguousarray(buf).tobytes()
        sock.send_multipart([hdr_bytes, buf_bytes])
    else:
        sock.send_multipart([hdr_bytes, b''])
    parts = sock.recv_multipart()
    resp_hdr = json.loads(parts[0])
    resp_buf = np.frombuffer(parts[1], dtype=resp_hdr.get('dtype', 'uint8')) if len(parts) > 1 and len(parts[1]) > 0 else None
    if resp_buf is not None and 'shape' in resp_hdr and resp_hdr['shape']:
        try:
            resp_buf = resp_buf.reshape(resp_hdr['shape'])
        except:
            pass
    return resp_hdr, resp_buf

# 1. is_working
print("\n--- is_working ---")
hdr, _ = send_recv({"cmd": "is_working"})
print(f"  {hdr}")

# 2. seq_clear
print("\n--- seq_clear ---")
hdr, _ = send_recv({"cmd": "seq_clear"})
print(f"  {hdr}")

# 3. get_states
print("\n--- get_states ---")
hdr, buf = send_recv({"cmd": "get_states", "args": 0})
print(f"  cmd={hdr['cmd']}")
if buf is not None:
    states = buf.reshape(hdr['shape']) if 'shape' in hdr else buf
    print(f"  shape={states.shape}, q={np.degrees(states[:, 0]).round(1)}")
    q_before = states[:, 0].copy()

# 4. set_cmds — 连续发
print("\n--- set_cmds x 500 ---")
home = np.array([0, -0.785, 2.2, 0.5, 0, 0, 0.0], dtype=np.float64)
success_count = 0
fail_count = 0
for seq in range(500):
    ts = hex_zmq_ts_now()
    hdr, _ = send_recv(
        {"cmd": "set_cmds", "ts": ts, "args": seq,
         "dtype": str(home.dtype), "shape": list(home.shape)},
        home
    )
    if hdr.get("cmd") == "set_cmds_ok":
        success_count += 1
    else:
        fail_count += 1
    time.sleep(0.004)  # ~250Hz

print(f"  成功: {success_count}, 失败: {fail_count}")

# 5. 再读状态
time.sleep(1)
print("\n--- get_states (after) ---")
for seq in range(q_before.shape[0] if q_before is not None else 1, q_before.shape[0] + 5 if q_before is not None else 5):
    hdr, buf = send_recv({"cmd": "get_states", "args": seq})
    if hdr.get("cmd") == "get_states_ok" and buf is not None:
        states = buf.reshape(hdr['shape']) if 'shape' in hdr else buf
        q_after = states[:, 0]
        print(f"  q={np.degrees(q_after).round(1)}")
        diff = np.degrees(q_after - q_before)
        print(f"  变化: {diff.round(1)}")
        if np.max(np.abs(diff[:6])) > 0.5:
            print("  ✓ 动了!")
        else:
            print("  ✗ 没动")
        break
else:
    print("  无法读取")

sock.close()
ctx.term()
