#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Lock-free shared-memory IPC between the main arm-server process
# (robot_hexarm: heavy pinocchio/FK/PUB/ZMQ) and the isolated
# device-I/O process (hexarm_device_io: HexDeviceApi _periodic + KCP).
#
# WHY: at control_hz=1000 the SDK's _periodic watchdog-feed thread must
# emit a DOWN frame every 1ms. In one process its GIL is stolen by the
# per-tick pinocchio-gravity / FK / zmq compute -> >300ms starvation ->
# firmware PscApiCommunicationTimeout park. Splitting the two into
# separate processes gives _periodic its own GIL/core. This module is the
# bridge: two fixed-size, single-writer/single-reader SEQLOCK slots so a
# read never blocks the 1ms sender and never sees a torn frame.
#
# SEQLOCK contract (per slot, one writer / one reader):
#   writer: seq -> odd (publish start); write payload; seq -> even (+2, done)
#   reader: read seq0; if odd, retry; read payload; read seq1;
#           if seq1 != seq0, retry. Bounded retries, then use last good.
# Payloads are fixed-size float64 arrays -> a stale read is at worst one
# tick old, never corrupt. No mutex, no syscall on the hot path.
################################################################

import numpy as np
from multiprocessing import shared_memory

# firefly_y6 arm (6-7 motors) + gr100 gripper (1). 8 covers both with margin;
# the real dof count travels in the header so the reader slices exactly.
MAX_DOF = 8

# ---- STATE slot layout (device-io -> main): measured motor state ----------
# The hex_device timestamp is a dict {'s': int, 'ns': int} (NOT a float), and the
# work_loop compares it via hex_ts_delta_ms — so we carry s/ns as two fields and
# rebuild the dict on the reader side (a float() would TypeError).
#   [0]      seq (seqlock; float64 holds the int exactly for our range)
#   [1]      ts.s   (device timestamp seconds)
#   [2]      ts.ns  (device timestamp nanoseconds)
#   [3]      n_dof (valid motor count)
#   [4]      valid (1.0 once the first real frame is written, else 0.0)
#   [5                :5+MAX_DOF ]   pos
#   [5+  MAX_DOF      :5+2*MAX_DOF]  vel
#   [5+2*MAX_DOF      :5+3*MAX_DOF]  eff
_STATE_HDR = 5
STATE_LEN = _STATE_HDR + 3 * MAX_DOF

# ---- CMD slot layout (main -> device-io): FULLY-RESOLVED command ----------
# The main process does ALL command building (gravity feedforward, MIT-safety
# effort/slew clamp, reference slew, joint limits, gripper-mode decision) and
# writes only numbers here; device-io replays them onto the SDK with zero
# pinocchio. Arm goes out as a 5-col MIT command; the gripper carries a small
# mode enum so the 3 gripper behaviors (position / limp-torque / gated-position)
# survive the split.
#   [0]      seq (seqlock)
#   [1]      cmd_seq (monotone command id; device-io re-sends last on a gap)
#   [2]      arm_dof
#   [3]      grip_dof
#   [4]      grip_mode  (0 POSITION | 1 TORQUE-limp | 2 gated POSITION)
#   [5]      grip_gate  (set_pos_torque value for grip_mode==2; <0 = leave)
#   [6                :6+  MAX_DOF]  arm_pos   (target position, rad)
#   [6+  MAX_DOF      :6+2*MAX_DOF]  arm_vel
#   [6+2*MAX_DOF      :6+3*MAX_DOF]  arm_tor   (feedforward torque incl. gravity)
#   [6+3*MAX_DOF      :6+4*MAX_DOF]  arm_kp
#   [6+4*MAX_DOF      :6+5*MAX_DOF]  arm_kd
#   [6+5*MAX_DOF      :6+6*MAX_DOF]  grip_val  (position rad, or ignored for limp)
_CMD_HDR = 6
CMD_LEN = _CMD_HDR + 6 * MAX_DOF

_SEQLOCK_RETRIES = 8


def _create(name: str, length: int) -> shared_memory.SharedMemory:
    size = length * np.dtype(np.float64).itemsize
    try:
        shm = shared_memory.SharedMemory(name=name, create=True, size=size)
    except FileExistsError:
        # Stale segment from a crashed prior run — reclaim it.
        old = shared_memory.SharedMemory(name=name)
        old.close()
        old.unlink()
        shm = shared_memory.SharedMemory(name=name, create=True, size=size)
    np.ndarray(length, dtype=np.float64, buffer=shm.buf)[:] = 0.0
    return shm


class ShmemSlot:
    """One seqlock slot. Create in ONE process (owner=True), attach in the other."""

    def __init__(self, name: str, length: int, owner: bool):
        self._length = length
        if owner:
            self._shm = _create(name, length)
        else:
            self._shm = shared_memory.SharedMemory(name=name)
        self._owner = owner
        self._buf = np.ndarray(length, dtype=np.float64, buffer=self._shm.buf)

    # ---- writer side ------------------------------------------------------
    def write(self, payload: np.ndarray) -> None:
        """payload: float64 array of length (self._length - 1); index 0 is the seq."""
        b = self._buf
        seq = b[0] + 1.0            # -> odd: publish in progress
        b[0] = seq
        b[1:] = payload             # numpy assignment; ordered after the seq store
        b[0] = seq + 1.0            # -> even: done

    # ---- reader side ------------------------------------------------------
    def read(self) -> np.ndarray | None:
        """Return a consistent copy of the payload (length-1), or None if never written."""
        b = self._buf
        for _ in range(_SEQLOCK_RETRIES):
            s0 = b[0]
            if s0 == 0.0:
                return None         # never written
            if int(s0) % 2 == 1:
                continue            # writer mid-update (odd)
            payload = b[1:].copy()
            s1 = b[0]
            if s0 == s1:
                return payload
        return b[1:].copy()         # torn under sustained contention: last-effort copy

    def peek_seq(self) -> float:
        """Cheap seqlock-free read of the first payload float (buf[1]). For the CMD
        slot that is cmd_seq — lets the reader skip the full copy+unpack when the
        command is unchanged. Monotone cmd_seq makes a torn read safe: the caller
        re-reads under the seqlock (read() + cmd_seq re-check) before acting."""
        return float(self._buf[1])

    def close(self):
        try:
            self._shm.close()
            if self._owner:
                self._shm.unlink()
        except Exception:
            pass


# ---- typed helpers on top of the raw slots --------------------------------

def pack_state(ts, pos, vel, eff) -> np.ndarray:
    """ts: hex_device timestamp dict {'s': int, 'ns': int}."""
    pos = np.asarray(pos, dtype=np.float64).ravel()
    vel = np.asarray(vel, dtype=np.float64).ravel()
    eff = np.asarray(eff, dtype=np.float64).ravel()
    n = pos.shape[0]
    p = np.zeros(STATE_LEN - 1, dtype=np.float64)   # -1: seq lives in slot[0]
    p[0] = float(ts.get('s', 0)) if isinstance(ts, dict) else 0.0
    p[1] = float(ts.get('ns', 0)) if isinstance(ts, dict) else 0.0
    p[2] = float(n)
    p[3] = 1.0                                       # valid
    p[4:4 + n] = pos
    p[4 + MAX_DOF:4 + MAX_DOF + n] = vel
    p[4 + 2 * MAX_DOF:4 + 2 * MAX_DOF + n] = eff
    return p


def unpack_state(payload: np.ndarray):
    """Return (ts_dict {'s','ns'}, state[n,3] pos/vel/eff) or None if not valid."""
    if payload is None or payload[3] < 0.5:
        return None
    ts = {'s': int(payload[0]), 'ns': int(payload[1])}
    n = int(payload[2])
    pos = payload[4:4 + n]
    vel = payload[4 + MAX_DOF:4 + MAX_DOF + n]
    eff = payload[4 + 2 * MAX_DOF:4 + 2 * MAX_DOF + n]
    return ts, np.array([pos, vel, eff]).T.copy()


def pack_cmd(cmd_seq, arm_pos, arm_vel, arm_tor, arm_kp, arm_kd,
             grip_mode, grip_gate, grip_val) -> np.ndarray:
    def a(x): return np.asarray(x, dtype=np.float64).ravel()
    arm_pos, arm_vel, arm_tor = a(arm_pos), a(arm_vel), a(arm_tor)
    arm_kp, arm_kd = a(arm_kp), a(arm_kd)
    grip_val = a(grip_val) if grip_val is not None else np.zeros(0)
    na, ng = arm_pos.shape[0], grip_val.shape[0]
    p = np.zeros(CMD_LEN - 1, dtype=np.float64)
    p[0] = float(cmd_seq)
    p[1] = float(na)
    p[2] = float(ng)
    p[3] = float(grip_mode)
    p[4] = float(grip_gate)
    base = 5
    for i, arr in enumerate((arm_pos, arm_vel, arm_tor, arm_kp, arm_kd)):
        p[base + i * MAX_DOF: base + i * MAX_DOF + na] = arr
    p[base + 5 * MAX_DOF: base + 5 * MAX_DOF + ng] = grip_val
    return p


def unpack_cmd(payload: np.ndarray):
    """Return dict of the resolved command, or None if not written yet."""
    if payload is None:
        return None
    cmd_seq = int(payload[0])
    na, ng = int(payload[1]), int(payload[2])
    grip_mode, grip_gate = int(payload[3]), float(payload[4])
    base = 5
    def col(i, n): return payload[base + i * MAX_DOF: base + i * MAX_DOF + n].copy()
    return {
        "cmd_seq": cmd_seq,
        "arm_pos": col(0, na), "arm_vel": col(1, na), "arm_tor": col(2, na),
        "arm_kp": col(3, na), "arm_kd": col(4, na),
        "grip_mode": grip_mode, "grip_gate": grip_gate,
        "grip_val": payload[base + 5 * MAX_DOF: base + 5 * MAX_DOF + ng].copy(),
    }
