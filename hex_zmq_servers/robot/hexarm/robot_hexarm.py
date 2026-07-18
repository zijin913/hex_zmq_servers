#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-14
################################################################

import os
import time
import threading
import numpy as np
from collections import deque

from ..robot_base import HexRobotBase
from ..mit_control import MitArmSafety, idle_gone_stale, build_safe_hold_cmd
from ...hex_launch import hex_log, HEX_LOG_LEVEL

from hex_robo_utils import (
    HexRate,
    hex_ts_delta_ms,
    hex_ts_now,
)
from hex_device import HexDeviceApi, Arm, Hands
from hex_device.motor_base import CommandType

# #6: SDK-isolation process — the HexDeviceApi (_periodic watchdog-feed + KCP)
# now runs in a SEPARATE process so its 1ms send never loses the GIL to this
# process's pinocchio/FK/PUB compute (the control_hz=1000 park root cause).
import multiprocessing as mp
from . import hexarm_shmem as SHM
from .hexarm_device_io import run_device_io, GRIP_POSITION, GRIP_LIMP, GRIP_GATED

# spawn context: the child re-imports fresh (no inherited pinocchio/numpy/SDK
# state) and the SDK is spawn-safe (verified on real HW).
_MP = mp.get_context("spawn")

ROBOT_CONFIG = {
    "device_ip": "172.18.8.161",
    "device_port": 8439,
    "control_hz": 250,
    "arm_type": "archer_y6",
    "mit_kp": [200.0, 200.0, 200.0, 75.0, 15.0, 15.0, 20.0],
    "mit_kd": [12.5, 12.5, 12.5, 6.0, 0.31, 0.31, 1.0],
    "sens_ts": True,
}

HEX_DEVICE_TYPE_DICT = {
    "archer_y6": 25,
    "archer_d6y": 16,
    "archer_l6y": 17,
    "firefly_y6": 27,
    "firefly_y6_h1": 27,  # 实机 launcher 配置(left/right_arm_cfg.json)用此别名
    "hello": 26,
}


class HexRobotHexarm(HexRobotBase):

    def __init__(
        self,
        robot_config: dict = ROBOT_CONFIG,
        realtime_mode: bool = False,
    ):
        HexRobotBase.__init__(self, realtime_mode)

        try:
            device_ip = robot_config["device_ip"]
            device_port = robot_config["device_port"]
            control_hz = robot_config["control_hz"]
            arm_type = HEX_DEVICE_TYPE_DICT[robot_config["arm_type"]]
            self.__sens_ts = robot_config["sens_ts"]
        except KeyError as ke:
            missing_key = ke.args[0]
            raise ValueError(
                f"robot_config is not valid, missing key: {missing_key}")

        self.__mit_kp = robot_config.get(
            "mit_kp",
            [200.0, 200.0, 200.0, 75.0, 15.0, 15.0, 20.0],
        )
        self.__mit_kd = robot_config.get(
            "mit_kd",
            [12.5, 12.5, 12.5, 6.0, 0.31, 0.31, 1.0],
        )

        # Gripper (Hands device) compliance for zero-gravity hand-posing. The
        # gripper is POSITION-controlled (its MIT kp/kd/torque are ignored) and
        # its internal position servo is too stiff to backdrive by hand. When a
        # client requests compliance (gripper-joint kp≈0, as the zero-gravity cli
        # sends) we relax it per gripper_compliant_mode:
        #   "torque"   — command zero torque so the motor goes limp and the claws
        #                backdrive by hand (default; the only thing that actually
        #                frees a stiff position servo). Won't hold a pose while
        #                limp. Falls back to a position hold if the firmware
        #                ignores torque commands.
        #   "position" — keep POSITION control but widen the Hands torque gate
        #                (set_pos_torque -> gripper_compliant_torque) so the
        #                streamed current angle tracks the hand. Gentler, but can
        #                still feel stiff if the motor's position loop is hard.
        self.__gripper_compliant_mode = str(
            robot_config.get("gripper_compliant_mode", "torque")).lower()
        self.__gripper_compliant_torque = float(
            robot_config.get("gripper_compliant_torque", 50.0))
        self.__gripper_default_torque = float(
            robot_config.get("gripper_default_torque", 3.0))
        self.__gripper_gate = None  # last set_pos_torque value (lazy-applied)

        # Idle-hold keep-alive (opt-in). When the client command stream has a
        # gap (teleop clutch released, pause, jitter, or no client at all),
        # re-send the last command / hold the measured pose so the firmware's
        # API watchdog keeps getting commands. Without this the arm parks with
        # PscApiCommunicationTimeout, the SDK churns reconnects, and the native
        # hex_device layer can segfault and kill the server. Default off so
        # behavior is unchanged unless a config opts in.
        self.__idle_hold = bool(robot_config.get("idle_hold", False))
        self.__idle_hold_period_ms = 1000.0 / float(
            robot_config.get("idle_hold_hz", 200.0))

        # Client-death safe-stop. After this many ms with no FRESH client command,
        # stop re-sending the last command (which would pin the arm rigid at its
        # last kp — up to ~200 — against whatever it was touching) and instead
        # stream a gravity-compensated COMPLIANT hold of the measured pose (soft
        # float; gripper stays clamped so a grasped payload is not dropped). Must
        # exceed the longest legitimate command gap (teleop clutch / policy pause /
        # inter-chunk). None/<=0 keeps the legacy hold-last-forever behavior.
        _ihm = robot_config.get("idle_hold_max_ms", 2000.0)
        self.__idle_hold_max_ms = (float(_ihm) if _ihm is not None
                                   and float(_ihm) > 0 else None)
        self.__idle_safe_kp_scale = float(
            robot_config.get("idle_safe_kp_scale", 0.15))
        self.__idle_safe_kd_scale = float(
            robot_config.get("idle_safe_kd_scale", 0.5))
        self.__idle_safe_active = False   # engaged-safe-hold flag (log once)

        # High-rate state broadcaster (zmq.PUB, optional). When pub_port is set,
        # publish per-parameter topics "<side>.{pos,vel,eff,state}" on every fresh
        # tick for clients that want push-based, selectively-subscribable feedback at
        # the device loop rate (alongside the REQ/REP get_states path). NON-BLOCKING:
        # a slow/absent subscriber drops frames and never stalls this control loop.
        # #6 opt (方案2): the state PUB moved into the device-I/O process so the
        # state stream is RT-steady (not jittery like this non-RT process). This
        # process no longer creates/uses a StatePublisher — it just remembers
        # pub_port/side to hand to the child. Dropping the main-process publish
        # also removes the per-fresh-frame pinocchio-tau_ext + FK + 5x zmq that
        # used to run in work_loop here (extra jitter + CPU gone).
        self.__pub_side = str(robot_config.get("side", "arm"))
        self.__pub_port = robot_config.get("pub_port")
        self.__state_pub = None
        self.__ee_fk = None

        # work_loop spin rate. Post-#6 this loop only copies the freshest seqlock
        # STATE frame into states_queue + forwards fresh CMD-slot commands; the SDK
        # read/_periodic threads (whose in-process GIL starvation used to park the
        # arm) now live in the separate device-io process. State refreshes at only
        # <= device_io_state_hz, so 2000Hz was pure oversampling — default to
        # control_hz. (An explicit work_loop_hz in a cfg still overrides.)
        self.__work_loop_hz = float(
            robot_config.get("work_loop_hz", control_hz))

        # Gravity feedforward (opt-in). MIT commands here carry zero torque
        # feedforward, so position-only commands settle below target by g/kp and
        # a hold (target≈measured) droops. When enabled, add tau_g(q) computed
        # from the firefly_y6 + gr100 URDF (pinocchio) to the arm torque so the
        # arm holds its commanded pose. Benefits every client (teleop included).
        # gravity_comp_scale<1 leaves a controlled residual sag. Default off so
        # behavior is unchanged unless a config opts in.
        self.__gravity_comp = bool(robot_config.get("gravity_comp", False))
        self.__grav_scale = float(robot_config.get("gravity_comp_scale", 1.0))

        # <side>/wrench = ee_wrench_sign · J(q)^-T · tau_ext — quasi-static Cartesian
        # force estimate published on the state stream. Sign default +1 matches
        # tools/ee_wrench_check.py's raw solve; flip to -1 after the known-weight
        # calibration so a downward hung load reads -z (external wrench applied to the
        # robot). Only computed when a client is subscribed (see work_loop).
        self.__ee_wrench_sign = float(robot_config.get("ee_wrench_sign", 1.0))

        # Reference slew (anti-lunge) — IDENTICAL to the sim device so sim predicts
        # real. Cap how far the commanded arm target may lead the MEASURED angle
        # (rad); kp*(q_des-q) then stays inside the motor torque envelope, so a
        # far / stale / aggressive setpoint becomes a smooth bounded-speed pursuit
        # instead of saturating and lunging. Default 0.1 (the sim default) so the
        # real arm is protected even without an explicit cfg; <=0 disables. Only
        # binds on a far jump — dense policy/home/jog steps are well under it.
        _mpe = robot_config.get("max_pos_err", 0.1)
        self.__max_pos_err = float(_mpe) if _mpe and float(_mpe) > 0 else None

        # Control mode: "position" | "joint_impedance" | "torque". Decided purely by the
        # command column-count in __set_cmds (soda_os shapes the 5-col MIT command), so
        # the device needs no per-mode branch; the flag is kept for state/logging only.
        self.__control_mode = str(
            robot_config.get("control_mode", "position")).lower()

        # Shared MIT-arm safety (gravity feedforward + effort/slew clamp). Built from
        # the SAME config keys (incl. gravity_comp_scale_lowstiff) via from_config so
        # the sim and real devices read everything identically — the sim is a faithful
        # pre-flight test. (robot/mit_control.py)
        self.__safety = MitArmSafety.from_config(robot_config)

        # Latest measured arm position (cached by work_loop) — the max_pos_err guard
        # needs measured q, which __set_cmds doesn't otherwise receive.
        self.__last_q = None

        # Pinocchio model — needed for gravity feedforward.
        self.__pin = None
        self.__grav_model = None
        self.__grav_data = None
        if self.__gravity_comp:
            try:
                import pinocchio as pin
                urdf = os.path.join(os.path.dirname(__file__),
                                    "urdf", "firefly_y6", "gr100.urdf")
                self.__pin = pin
                self.__grav_model = pin.buildModelFromUrdf(urdf)
                self.__grav_data = self.__grav_model.createData()
                self.__grav_model.gravity.linear = np.array([0.0, 0.0, -9.81])
                hex_log(HEX_LOG_LEVEL["info"],
                        f"[hexarm] pinocchio ON (gravity_comp={self.__gravity_comp} "
                        f"scale={self.__grav_scale}, control_mode={self.__control_mode})")
            except Exception as e:
                print(f"\033[91m[hexarm] pinocchio init failed: {e}\033[0m")
                self.__gravity_comp = False

        # ---- #6: spawn the SDK-isolation device-I/O process --------------
        # The HexDeviceApi (arm + gripper) no longer lives in THIS process; it
        # runs in run_device_io on its own GIL so the 1ms _periodic watchdog
        # feed never loses the GIL to the pinocchio/FK/PUB compute below (the
        # control_hz=1000 park root cause). We talk to it over two seqlock
        # shmem slots (hot path) + a Queue (one-time dof/limits handshake) +
        # two Values (clear-fault request / fault-status readback). spawn (not
        # fork) — the SDK is spawn-safe (verified) and a fresh child avoids
        # inheriting this process's pinocchio/numpy state.
        _uid = f"{self.__pub_side}_{os.getpid()}"
        self.__state_name = f"hexarm_state_{_uid}"
        self.__cmd_name = f"hexarm_cmd_{_uid}"
        self.__state_slot = SHM.ShmemSlot(self.__state_name, SHM.STATE_LEN, owner=True)
        self.__cmd_slot = SHM.ShmemSlot(self.__cmd_name, SHM.CMD_LEN, owner=True)
        self.__cmd_seq = 0
        self.__io_cfg = {
            "device_ip": device_ip, "device_port": device_port,
            "control_hz": control_hz, "arm_type": arm_type,
            "gripper_max_position": robot_config.get("gripper_max_position"),
            "sens_ts": self.__sens_ts,
            # Poll loop governs CMD-pickup latency (commands arrive <=~250Hz) — it
            # is NOT the firmware watchdog feed (that is the SDK's own _periodic), so
            # lowering it never risks a park; lowering it FREES the shared GIL for
            # _periodic. 1.0x the report rate is plenty given the cheap cmd_seq peek.
            "device_io_poll_hz": float(robot_config.get(
                "device_io_poll_hz", max(2.0 * control_hz, 1000.0))),
            # STATE read + PUB run slower than the poll loop (CMD pickup stays at
            # poll cadence) — 500Hz still far exceeds every STATE consumer.
            "device_io_state_hz": float(robot_config.get(
                "device_io_state_hz", max(2.0 * control_hz, 1000.0))),
            # get_status_summary() is a heavy full SDK query — throttle it (latched
            # park indicator, a few tens of ms of detection latency is fine).
            "device_io_fault_hz": float(robot_config.get(
                "device_io_fault_hz", 20.0)),
            # #6 opt: state PUB runs in the device-io (RT-steady); pass it down.
            "pub_port": self.__pub_port,
            "side": self.__pub_side,
            "device_io_rt_prio": int(robot_config.get("device_io_rt_prio", 20)),
        }
        self.__io_stop = _MP.Event()
        self.__io_clear_req = _MP.Value('i', 0)
        self.__io_fault = _MP.Value('d', 0.0)
        self.__io_proc = None
        self.__io_lock = threading.Lock()   # serialize (re)spawn
        hs = self.__spawn_device_io()        # blocks on the dof/limits handshake

        # ---- dof / limits from the handshake (was queried off the SDK here) --
        arm_dofs = int(hs["arm_dofs"])
        self._dofs = [arm_dofs]
        self._limits = np.array(hs["arm_limits"]).reshape(-1, 3, 2)
        self.__motor_idx = {"robot_arm": np.arange(arm_dofs).tolist()}
        if hs.get("gripper_dofs"):
            gripper_dofs = int(hs["gripper_dofs"])
            self._dofs.append(gripper_dofs)
            gripper_limits = np.array(hs["gripper_limits"]).reshape(-1, 3, 2)
            self._limits = np.concatenate([self._limits, gripper_limits],
                                          axis=0)
            self.__motor_idx["robot_gripper"] = (np.arange(gripper_dofs) +
                                                 arm_dofs).tolist()

        # modify variables
        self._dofs = np.array(self._dofs)
        self._dofs_sum = self._dofs.sum()
        self._limits = np.ascontiguousarray(np.asarray(self._limits)).reshape(
            self._dofs_sum, 3, 2)
        self.__mit_kp = np.ascontiguousarray(np.asarray(self.__mit_kp))
        self.__mit_kd = np.ascontiguousarray(np.asarray(self.__mit_kd))
        if self.__mit_kp.shape[0] < self._dofs_sum or self.__mit_kd.shape[
                0] < self._dofs_sum:
            raise ValueError(
                "The length of mit_kp and mit_kd must be greater than or equal to the number of motors"
            )
        elif self.__mit_kp.shape[0] > self._dofs_sum or self.__mit_kd.shape[
                0] > self._dofs_sum:
            print(
                f"\033[33mThe length of mit_kp and mit_kd is greater than the number of motors\033[0m"
            )
            self.__mit_kp = self.__mit_kp[:self._dofs_sum]
            self.__mit_kd = self.__mit_kd[:self._dofs_sum]

        # Precompute the compliant safe-hold gains used on client death: arm joints
        # get a fraction of the MIT gains (soft float that resists sag; 0 = pure
        # zero-gravity float), the gripper keeps its full kp so it stays clamped.
        _arm_idx = self.__motor_idx["robot_arm"]
        self.__idle_safe_kp = self.__mit_kp[_arm_idx] * self.__idle_safe_kp_scale
        self.__idle_safe_kd = self.__mit_kd[_arm_idx] * self.__idle_safe_kd_scale
        _g = self.__motor_idx.get("robot_gripper")
        self.__idle_safe_grip_kp = float(self.__mit_kp[_g[0]]) if _g else 0.0
        self.__idle_safe_grip_kd = float(self.__mit_kd[_g[0]]) if _g else 0.0

        # start work loop
        self._working.set()

    def work_loop(self, hex_queues: list[deque | threading.Event]):
        states_queue = hex_queues[0]
        cmds_queue = hex_queues[1]
        stop_event = hex_queues[2]

        last_states_ts = hex_ts_now()
        states_count = 0
        last_cmds_seq = -1
        last_cmds = None       # most recent client command (idle-hold source)
        hold_pos = None        # latest measured pose (live)
        idle_target = None     # latched pose held while idle (fixed, not chased)
        last_send_ts = hex_ts_now()
        last_real_cmd_ts = hex_ts_now()  # last FRESH client command (safe-stop timer)
        last_io_check = hex_ts_now()     # #6: device-io liveness poll throttle
        rate = HexRate(self.__work_loop_hz)
        # --- optional hot-loop profiler (SODA_LOOP_PROFILE=1): per-stage SDK
        # call latency (read = get_simple_motor_status, write = __set_cmds ->
        # motor_command), dumped every 5 s OFF the hot path. Near-zero cost when
        # off (one bool test/tick). GATE 1: does motor_command enqueue (us) or
        # block on a controller ack (ms)?  No behaviour change; timing only.
        import os as _os, time as _time, threading as _threading
        _prof_on = bool(_os.environ.get("SODA_LOOP_PROFILE"))
        if _prof_on:
            _N = max(2000, int(self.__work_loop_hz) * 5)
            _rd = np.zeros(_N); _wr = np.zeros(_N); _lp = np.zeros(_N)
            _idx = {"r": 0, "w": 0, "l": 0}
            _pstop = _threading.Event()

            def _prof_dump():
                while not _pstop.wait(5.0):
                    def _s(a, n):
                        v = a[:min(n, _N)]; v = v[v > 0]
                        if v.size == 0:
                            return "n=0"
                        return (f"p50={np.percentile(v, 50) / 1e3:7.1f} "
                                f"p99={np.percentile(v, 99) / 1e3:7.1f} "
                                f"max={v.max() / 1e3:8.1f}us n={v.size}")
                    print(f"[PROF] read  {_s(_rd, _idx['r'])}\n"
                          f"[PROF] write {_s(_wr, _idx['w'])}\n"
                          f"[PROF] loop  {_s(_lp, _idx['l'])}", flush=True)
            _threading.Thread(target=_prof_dump, name="loop_prof",
                              daemon=True).start()
        # --- optional RT control path (SODA_RT_CONTROL=1): pin THIS control loop
        # (the now-light post-#6 loop) to an isolated core (left->4 / right->5, to
        # match isolcpus=6,7) + SCHED_FIFO-80, and force the PUB decouple on so
        # the serialize/send never blocks the 1 kHz loop. Best-effort + guarded:
        # any failure logs and falls back to normal scheduling — a physical arm
        # must never fail to start over RT setup. Requires the box provisioned by
        # scripts/rt/rt_setup.sh (isolcpus + rtprio). SODA_RT_CORE overrides.
        _rt_on = bool(_os.environ.get("SODA_RT_CONTROL"))
        if _rt_on:
            try:
                _rc = _os.environ.get("SODA_RT_CORE")
                _core = int(_rc) if _rc else (6 if self.__pub_side == "left" else 7)
                _os.sched_setaffinity(0, {_core})
                _os.sched_setscheduler(0, _os.SCHED_FIFO, _os.sched_param(80))
                print(f"[hexarm] RT control loop -> core {_core}, SCHED_FIFO-80")
            except Exception as e:
                print(f"[hexarm] RT setup failed ({e}); default scheduling")
        # --- PUB decouple (SODA_PUB_THREAD=1, or implied by SODA_RT_CONTROL): move
        # the ~0.5 ms state serialize/encode/send OFF this hot loop to a non-RT
        # publisher thread (latest-wins, coalescing). The hot loop only hands off
        # the latest frame (states.copy + notify, ~us).
        self.__pub_thread_on = ((bool(_os.environ.get("SODA_PUB_THREAD")) or _rt_on)
                                and self.__state_pub is not None)
        self.__pub_rt = _rt_on
        if self.__pub_thread_on:
            self.__pub_slot = None     # (tag, ts, states) — GIL-atomic latest-wins
            self.__pub_tag = 0         # monotonic control-cycle tag (seqlock)
            self.__pub_stop = _threading.Event()
            _threading.Thread(target=self.__pub_worker, name="pub_worker",
                              daemon=True).start()
        while self._working.is_set() and not stop_event.is_set():
            # #6: respawn the device-I/O process if the SDK crashed (a segfault
            # on churn killed the arm before). Throttled to ~5Hz; reattaches to
            # the same shmem slots so ZMQ clients don't see a gap.
            if hex_ts_delta_ms(hex_ts_now(), last_io_check) > 200.0:
                last_io_check = hex_ts_now()
                if self.__io_proc is not None and not self.__io_proc.is_alive():
                    print("\033[91m[hexarm] device-io died -> respawning\033[0m",
                          flush=True)
                    try:
                        self.__spawn_device_io()
                    except Exception as e:
                        print(f"\033[91m[hexarm] device-io respawn failed: {e}\033[0m",
                              flush=True)

            # states
            _t0 = _time.perf_counter_ns() if _prof_on else 0
            ts, states = self.__get_states()
            if _prof_on:
                _rd[_idx["r"] % _N] = _time.perf_counter_ns() - _t0
                _idx["r"] += 1
            if states is not None:
                hold_pos = states[:, 0]
                # Cache measured arm q for the max_pos_err guard (which runs inside
                # __set_cmds and otherwise has no access to measured state).
                arm_ids = self.__motor_idx["robot_arm"]
                self.__last_q = states[arm_ids, 0]
                if hex_ts_delta_ms(ts, last_states_ts) > 1e-6:
                    last_states_ts = ts
                    states_queue.append((ts, states_count, states))
                    states_count = (states_count + 1) % self._max_seq_num
                    # (#6: the high-rate state PUB now lives in the device-io process.
                    # The old per-fresh-frame pinocchio tau_ext + FK + publish that
                    # used to run HERE — the control_hz=1000 GIL-starvation park cause
                    # — is gone; __state_pub/__ee_fk are permanently None.)

            # cmds — sole consumer + append-only writers under the GIL, so a
            # truthiness guard is race-free and avoids raising/catching IndexError
            # on the ~common empty-queue iteration.
            cmds_pack = None
            if cmds_queue:
                cmds_pack = cmds_queue[-1] if self._realtime_mode else cmds_queue.popleft()
            sent = False
            if cmds_pack is not None:
                ts, seq, cmds = cmds_pack
                if seq != last_cmds_seq:
                    last_cmds_seq = seq
                    last_cmds = cmds
                    idle_target = None   # active stream: re-latch fresh on next idle
                    last_real_cmd_ts = hex_ts_now()  # client alive: reset safe-stop timer
                    if self.__idle_safe_active:      # leaving safe-hold on a fresh cmd
                        self.__idle_safe_active = False
                        hex_log(HEX_LOG_LEVEL["info"],
                                "[hexarm] client command resumed -> exit safe-hold")
                    # 命令时间戳新鲜度校验已移除:始终下发(see fork history)
                    # Never let a malformed command kill the control loop / server.
                    try:
                        _tw = _time.perf_counter_ns() if _prof_on else 0
                        self.__set_cmds(cmds)
                        if _prof_on:
                            _wr[_idx["w"] % _N] = _time.perf_counter_ns() - _tw
                            _idx["w"] += 1
                    except Exception as e:
                        print(f"\033[91m[hexarm] set_cmds error: {e}\033[0m")
                    last_send_ts = hex_ts_now()
                    sent = True

            # idle-hold keep-alive — feed the firmware's API watchdog when the
            # client command stream has a gap, so the arm holds position instead
            # of parking (PscApiCommunicationTimeout) and churning the SDK into a
            # native crash. Throttled to idle_hold_hz.
            #
            # Short gap (< idle_hold_max_ms): re-send the last real command if
            # there was one; otherwise hold a *latched* pose captured once when
            # the arm went idle. (A fixed target builds real kp*(target-measured)
            # torque and holds; feeding the LIVE measured pose with no gravity FF
            # would droop — but see the safe-hold below, which carries gravity FF.)
            #
            # Stale (>= idle_hold_max_ms, i.e. the client is presumed DEAD): drop
            # to a gravity-compensated COMPLIANT hold of the measured pose instead
            # of pinning the arm rigid at its last kp forever — safe-stop on client
            # death. The gripper stays clamped so a grasped payload is not dropped.
            if self.__idle_hold and not sent and hex_ts_delta_ms(
                    hex_ts_now(), last_send_ts) >= self.__idle_hold_period_ms:
                if idle_gone_stale(
                        hex_ts_delta_ms(hex_ts_now(), last_real_cmd_ts),
                        self.__idle_hold_max_ms) and hold_pos is not None:
                    hold = build_safe_hold_cmd(
                        hold_pos,
                        self.__motor_idx["robot_arm"],
                        self.__motor_idx.get("robot_gripper"),
                        self.__idle_safe_kp, self.__idle_safe_kd,
                        self.__idle_safe_grip_kp, self.__idle_safe_grip_kd)
                    if not self.__idle_safe_active:
                        self.__idle_safe_active = True
                        hex_log(HEX_LOG_LEVEL["info"],
                                "[hexarm] client idle > idle_hold_max_ms -> "
                                "compliant safe-hold (gripper stays clamped)")
                elif last_cmds is not None:
                    hold = last_cmds
                else:
                    if idle_target is None and hold_pos is not None:
                        idle_target = hold_pos.copy()   # latch once
                    hold = idle_target
                if hold is not None:
                    try:
                        _tw = _time.perf_counter_ns() if _prof_on else 0
                        self.__set_cmds(hold)
                        if _prof_on:
                            _wr[_idx["w"] % _N] = _time.perf_counter_ns() - _tw
                            _idx["w"] += 1
                    except Exception as e:
                        print(f"\033[91m[hexarm] idle-hold set_cmds error: {e}\033[0m")
                    last_send_ts = hex_ts_now()

            # sleep
            if _prof_on:
                _lp[_idx["l"] % _N] = _time.perf_counter_ns() - _t0
                _idx["l"] += 1
            rate.sleep()

        # stop the PUB worker before teardown (timer loop exits within one tick)
        if self.__pub_thread_on:
            self.__pub_stop.set()
        # close
        self.close()

    def __get_states(self) -> tuple[float | None, np.ndarray | None]:
        # #6: measured state now arrives from the device-I/O process via the
        # STATE seqlock slot. The arm/gripper timestamp-sync and the sens_ts
        # choice already happened there, so this is just a lock-free read of the
        # freshest consistent frame (or None until the first frame lands).
        got = SHM.unpack_state(self.__state_slot.read())
        if got is None:
            return None, None
        return got  # (ts, state[n,3] pos/vel/eff)

    def __set_cmds(self, cmds: np.ndarray) -> bool:
        # cmds: (n)
        # [pos_0, ..., pos_n]
        # cmds: (n, 2)
        # [[pos_0, tor_0], ..., [pos_n, tor_n]]
        # cmds: (n, 5)
        # [[pos_0, vel_0, tor_0, kp_0, kd_0], ..., [pos_n, vel_n, tor_n, kp_n, kd_n]]
        if cmds.shape[0] < self._dofs_sum:
            print(
                "\033[91mThe length of joint_angles must be greater than or equal to the number of motors\033[0m"
            )
            return False
        elif cmds.shape[0] > self._dofs_sum:
            print(
                f"\033[33mThe length of joint_angles is greater than the number of motors\033[0m"
            )
            cmds = cmds[:self._dofs_sum]

        cmd_pos = None
        tar_vel = np.zeros(self._dofs_sum)
        cmd_tor = np.zeros(self._dofs_sum)
        cmd_kp = self.__mit_kp.copy()
        cmd_kd = self.__mit_kd.copy()
        if len(cmds.shape) == 1:
            cmd_pos = cmds
        elif len(cmds.shape) == 2:
            if cmds.shape[1] == 2:
                cmd_pos = cmds[:, 0]
                cmd_tor = cmds[:, 1]
            elif cmds.shape[1] == 5:
                cmd_pos = cmds[:, 0]
                tar_vel = cmds[:, 1]
                cmd_tor = cmds[:, 2]
                cmd_kp = cmds[:, 3]
                cmd_kd = cmds[:, 4]
            else:
                raise ValueError(f"The shape of cmds is invalid: {cmds.shape}")
        else:
            raise ValueError(f"The shape of cmds is invalid: {cmds.shape}")

        # arm
        arm_cmd_pos = cmd_pos[self.__motor_idx["robot_arm"]]
        # Reference slew (anti-lunge), IDENTICAL to the sim device: keep the
        # commanded arm target within max_pos_err of the MEASURED angle so
        # kp*(q_des-q) stays inside the motor torque envelope — a far/stale/
        # aggressive setpoint pursues smoothly instead of saturating and lunging.
        # Applied before the joint-limit clamp (matches the sim ordering).
        if self.__max_pos_err is not None and self.__last_q is not None:
            q_meas = np.asarray(self.__last_q, dtype=np.float64)
            arm_cmd_pos = np.asarray(arm_cmd_pos, dtype=np.float64)
            if q_meas.shape == arm_cmd_pos.shape:
                arm_cmd_pos = q_meas + np.clip(arm_cmd_pos - q_meas,
                                               -self.__max_pos_err,
                                               self.__max_pos_err)
        arm_tar_pos = self._apply_pos_limits(
            arm_cmd_pos,
            self._limits[self.__motor_idx["robot_arm"], 0, 0],
            self._limits[self.__motor_idx["robot_arm"], 0, 1],
        )
        # Gravity feedforward (zero-stiffness-safe: measured pose + unity scale when
        # kp≈0) + effort/slew clamp, via the SHARED MitArmSafety so the sim device
        # does exactly the same thing. No-op for gravity unless gravity_comp is on.
        arm_tor = self.__safety.apply(
            cmd_tor[self.__motor_idx["robot_arm"]],
            arm_tar_pos, self.__last_q,
            cmd_kp[self.__motor_idx["robot_arm"]], self.__gravity_fn)
        # #6: instead of construct_mit_command + motor_command HERE, hand the
        # FULLY-RESOLVED MIT numbers to the device-I/O process via the CMD slot;
        # it replays them onto the SDK on its own GIL (pinocchio-free). The
        # gripper's behaviors (POSITION honored by Hands; MIT/torque ignored) are
        # decided here and carried as a mode enum + set_pos_torque gate so the
        # device side needs no branch logic and no state:
        #   LIMP  — compliant + "torque" mode: zero-torque so the claws backdrive
        #   GATED — widen the Hands torque gate (compliant_torque / default), then
        #           POSITION-control the streamed angle.
        _arm = self.__motor_idx["robot_arm"]
        grip_mode = GRIP_POSITION
        grip_gate = -1.0
        grip_val = np.zeros(0)
        g_idx = self.__motor_idx.get("robot_gripper")
        if g_idx is not None:
            grip_val = np.asarray(cmd_pos)[g_idx]
            want_compliant = bool(np.all(np.asarray(cmd_kp)[g_idx] <= 1e-6))
            if want_compliant and self.__gripper_compliant_mode == "torque":
                grip_mode = GRIP_LIMP
            else:
                grip_mode = GRIP_GATED
                grip_gate = (self.__gripper_compliant_torque if want_compliant
                             else self.__gripper_default_torque)

        self.__cmd_seq = (self.__cmd_seq + 1) % 2_000_000_000
        self.__cmd_slot.write(SHM.pack_cmd(
            self.__cmd_seq,
            arm_tar_pos, tar_vel[_arm], arm_tor,
            cmd_kp[_arm], cmd_kd[_arm],
            grip_mode, grip_gate, grip_val))
        return True

    # ==================== control mode + safety + Cartesian impedance ==========

    def set_control_mode(self, mode: str) -> bool:
        """Runtime control-mode switch (one-shot ZMQ cmd). The modes only change how
        soda_os shapes the streamed command; the device command path is identical, so
        this just records the mode and resets the torque-slew history across the switch."""
        mode = str(mode).lower()
        if mode not in ("position", "joint_impedance", "torque"):
            print(f"\033[91m[hexarm] unknown control_mode {mode!r}\033[0m")
            return False
        self.__control_mode = mode
        self.__safety._last_tau.clear()  # reset slew history across a mode change
        hex_log(HEX_LOG_LEVEL["info"], f"[hexarm] control_mode -> {mode}")
        return True

    def clear_fault(self) -> bool:
        """Clear a latched parking-stop / fault and re-enter MIT so the arm resumes
        WITHOUT a physical power-cycle. Best-effort against the closed hex_device SDK
        (getattr-guarded); the streamed MIT commands then re-establish control."""
        # #6: the SDK lives in the device-I/O process, so forward the request:
        # bump the shared counter; the child does clear_parking_stop + enable_mit.
        try:
            with self.__io_clear_req.get_lock():
                self.__io_clear_req.value += 1
            self.__idle_safe_active = False   # let the work loop command again
            hex_log(HEX_LOG_LEVEL["info"],
                    "[hexarm] clear_fault -> requested device-io")
            return True
        except Exception as e:
            print(f"\033[91m[hexarm] clear_fault error: {e}\033[0m")
            return False

    def get_fault(self) -> np.ndarray:
        """Fault descriptor for the ZMQ buffer: float64 [active(0/1),
        remotely_clearable(0/1)]. #6: the device-I/O process writes the active
        flag from the SDK status each tick into a shared Value; a parking stop
        is remotely clearable."""
        try:
            active = float(self.__io_fault.value)
        except Exception:
            active = 0.0
        return np.array([active, 1.0], dtype=np.float64)

    def __gravity_fn(self, q: np.ndarray) -> np.ndarray:
        """Generalized gravity at q — the pinocchio closure handed to MitArmSafety."""
        return self.__pin.computeGeneralizedGravity(
            self.__grav_model, self.__grav_data, np.asarray(q, dtype=np.float64))

    def __spawn_device_io(self):
        """(Re)spawn the device-I/O process and block on its dof/limits handshake.
        Respawns reattach to the SAME shmem slots (this process stays the owner),
        so the ZMQ clients see an uninterrupted state/command path across an SDK
        crash. Returns the handshake dict."""
        with self.__io_lock:
            # Put the TIMING-CRITICAL device-io (SDK _periodic/KCP watchdog-feed) on
            # cores 6/7 -- the HT siblings of the arm-compute cores 2/3, the exact
            # placement that gave the single-process 496Hz -- and leave the now-light
            # control loop on 4/5. Cores 4/5 are HT siblings of the busy cameras on
            # 0/1, so the device-io must NOT go there (that measured ~387 vs ~416
            # floating). Override per-side with SODA_RT_DEVICE_CORE="<left>,<right>".
            if os.environ.get("SODA_RT_CONTROL"):
                _dc = os.environ.get("SODA_RT_DEVICE_CORE", "")
                _cs = [int(x) for x in _dc.split(",") if x.strip()]
                self.__io_cfg["device_io_cpu"] = (
                    (_cs[0] if self.__pub_side == "left" else _cs[-1]) if _cs
                    else (4 if self.__pub_side == "left" else 5))
            init_q = _MP.Queue()
            self.__io_proc = _MP.Process(
                target=run_device_io,
                args=(self.__io_cfg, self.__state_name, self.__cmd_name,
                      self.__io_stop, init_q, self.__io_clear_req, self.__io_fault),
                daemon=True,
            )
            self.__io_proc.start()
            try:
                hs = init_q.get(timeout=60.0)
            except Exception:
                raise RuntimeError(
                    "device-io handshake timed out (arm not reachable?)")
            if isinstance(hs, dict) and "err" in hs:
                raise RuntimeError(f"device-io init failed: {hs['err']}")
            hex_log(HEX_LOG_LEVEL["info"],
                    f"[hexarm] device-io up (pid={self.__io_proc.pid})")
            return hs

    def close(self):
        if not self._working.is_set():
            return
        self._working.clear()
        # #6: stop the device-I/O process, then release the shmem slots (owner).
        try:
            self.__io_stop.set()
            if self.__io_proc is not None:
                self.__io_proc.join(timeout=5.0)
                if self.__io_proc.is_alive():
                    self.__io_proc.terminate()
        except Exception:
            pass
        try:
            self.__state_slot.close()
            self.__cmd_slot.close()
        except Exception:
            pass
        hex_log(HEX_LOG_LEVEL["info"], "HexRobotHexarm closed")
