#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Isolated device-I/O process for the HexFellow arm (#6).
#
# Runs HexDeviceApi (arm + gripper) — i.e. the SDK's _periodic watchdog-feed
# coroutine + the KCP send/recv threads — in a process with NOTHING ELSE on its
# GIL. No pinocchio, no FK, no state-PUB, no ZMQ-REP here. That is the whole
# point: at control_hz=1000 the _periodic sender must emit a DOWN frame every
# 1ms; giving it a free GIL/core is what keeps it under the firmware's ~300ms
# PscApiCommunicationTimeout window under load.
#
# IPC with the main robot_hexarm process (two seqlock shmem slots):
#   STATE slot (this proc WRITES): measured motor state (get_simple_motor_status)
#   CMD   slot (this proc READS) : the FULLY-RESOLVED command. The MAIN process
#     already did gravity feedforward + MIT effort/slew clamp + reference slew +
#     joint limits + the gripper-mode decision; here we only replay numbers onto
#     the SDK. motor_command() just stashes _target_command (cheap); the SDK's
#     own _periodic thread does the timed DOWN send + idle re-send.
#
# Crash isolation: if the closed SDK segfaults on a churn (the failure that
# killed the right arm), only THIS process dies. The parent detects the exit
# and respawns it — the heavy main process (and its ZMQ clients) survive.
################################################################

import os
import time
import numpy as np

from hex_device import HexDeviceApi
from hex_device.motor_base import CommandType
from hex_robo_utils import hex_ts_now, hex_ts_delta_ms

from . import hexarm_shmem as H

# grip_mode enum (must match robot_hexarm's writer)
GRIP_POSITION = 0   # normal position control
GRIP_LIMP = 1       # zero-torque limp (compliant hand-posing)
GRIP_GATED = 2      # widen set_pos_torque gate, then position


def _concat(a, g, key):
    return np.concatenate([a[key], g[key]]) if g is not None else np.asarray(a[key])


def _pin_affinity(cpu: int) -> int:
    """Pin ALL threads of THIS process (incl. the SDK's _periodic watchdog-feed +
    KCP send/recv) to a single isolated core. The isolated core (isolcpus) has no
    other runnable work, so those threads never contend the RealSense cameras +
    main compute on the general cores. Best-effort; returns the count pinned."""
    n = 0
    try:
        for tid in os.listdir("/proc/self/task"):
            try:
                os.sched_setaffinity(int(tid), {int(cpu)})
                n += 1
            except (PermissionError, OSError):
                pass
    except Exception:
        pass
    return n


def _set_rt_priority(prio: int) -> int:
    """Best-effort SCHED_FIFO on ALL threads of THIS process — including the SDK's
    _periodic watchdog-feed + KCP send/recv threads, which live here now (#6).

    RT priority lets the device-io win CPU against the RealSense cameras + the two
    main arm-server processes + backend + teleop when the box is saturated, so the
    1ms _periodic deadline is met and the firmware watchdog never times out. This
    only helps because #6 moved the SDK into its own process (RT is oblivious to
    the intra-process GIL). Needs CAP_SYS_NICE / root / RLIMIT_RTPRIO; returns 0
    (no-op, no crash) without the privilege. The loop's time.sleep() yields the
    CPU between ticks, so a moderate FIFO prio won't starve the system.
    """
    if prio <= 0:
        return 0
    param = os.sched_param(prio)
    n = 0
    try:
        for tid in os.listdir("/proc/self/task"):
            try:
                os.sched_setscheduler(int(tid), os.SCHED_FIFO, param)
                n += 1
            except (PermissionError, OSError):
                pass
    except Exception:
        pass
    return n


def run_device_io(cfg: dict, state_name: str, cmd_name: str, stop_flag,
                  init_q=None, clear_req=None, fault_active=None) -> None:
    """Body of the device-I/O process.

    stop_flag    : multiprocessing.Event — set by parent to shut down.
    init_q       : multiprocessing.Queue — child puts the one-time dof/limits
                   handshake so the parent can build its dof/limit tables
                   (the SDK now lives only here). Sends {"err": ...} on failure.
    clear_req    : multiprocessing.Value('i') — parent increments to request a
                   clear_parking_stop + enable_mit (SAFETY-2 clear-fault API).
    fault_active : multiprocessing.Value('d') — child writes 1.0/0.0 each tick
                   from the SDK status so the parent's get_fault() can read it.
    """
    # ---- open the SDK (moved out of robot_hexarm.__init__) ----------------
    api = HexDeviceApi(
        ws_url=f"ws://{cfg['device_ip']}:{cfg['device_port']}",
        control_hz=cfg['control_hz'],
    )
    arm_type = cfg['arm_type']  # already the int SDK type id
    while api.find_device_by_robot_type(arm_type) is None and not stop_flag.is_set():
        print("\033[33m[device_io] Arm not found\033[0m", flush=True)
        time.sleep(1)
    if stop_flag.is_set():
        return
    arm = api.find_device_by_robot_type(arm_type)
    arm.start()
    gripper = api.find_optional_device_by_id(1)
    if gripper is not None and cfg.get('gripper_max_position') is not None:
        try:
            gripper._hands_limit[1] = float(cfg['gripper_max_position'])
        except (AttributeError, IndexError):
            pass

    # ---- one-time handshake: send dof counts + joint limits to the parent --
    # (arm.get_joint_limits() is a hardware query only available here now.)
    if init_q is not None:
        try:
            arm_dofs = len(arm)
            hs = {"arm_dofs": arm_dofs,
                  "arm_limits": np.asarray(arm.get_joint_limits()).tolist(),
                  "gripper_dofs": 0, "gripper_limits": None}
            if gripper is not None:
                hs["gripper_dofs"] = len(gripper)
                hs["gripper_limits"] = np.asarray(
                    gripper.get_joint_limits()).tolist()
            init_q.put(hs)
        except Exception as e:
            init_q.put({"err": repr(e)})
            return

    # attach to the shmem slots the MAIN process created (owner=False)
    state_slot = H.ShmemSlot(state_name, H.STATE_LEN, owner=False)
    cmd_slot = H.ShmemSlot(cmd_name, H.CMD_LEN, owner=False)
    print(f"\033[36m[device_io] up: arm+gripper, control_hz={cfg['control_hz']}, "
          f"pid feeds firmware watchdog on its own GIL\033[0m", flush=True)

    # #6 + RT (方案2): SCHED_FIFO so this process (and its SDK _periodic/KCP
    # threads) outruns the cameras + main arm processes + teleop under full-stack
    # saturation, meeting the 1ms watchdog-feed deadline. Best-effort.
    rt_prio = int(cfg.get('device_io_rt_prio', 20))
    _n_rt = _set_rt_priority(rt_prio)
    # #6 + RT: pin all device-io threads to a dedicated isolated core (device_io_cpu,
    # left->4 / right->5, disjoint from the control loop on 6/7) so the SDK never
    # contends the cameras on the general cores -- that cross-core contention had
    # throttled the firmware report rate to ~418Hz. Best-effort.
    _cpu_pin = cfg.get('device_io_cpu')
    if _cpu_pin is not None:
        _n_pin = _pin_affinity(int(_cpu_pin))
        if _n_pin > 0:
            print("[device_io] pinned %d threads -> isolated core %d"
                  % (_n_pin, int(_cpu_pin)), flush=True)
    if _n_rt > 0:
        print(f"\033[36m[device_io] RT SCHED_FIFO prio={rt_prio} on {_n_rt} "
              f"threads\033[0m", flush=True)
    elif rt_prio > 0:
        print(f"\033[33m[device_io] RT priority NOT granted (need CAP_SYS_NICE / "
              f"RLIMIT_RTPRIO / sudo) — normal prio\033[0m", flush=True)

    # #6 opt (方案2): the state PUB now lives HERE (the RT process) so the state
    # stream is RT-steady instead of jittery like the non-RT main process. Raw
    # pos/vel/eff only — tau_ext/ee (pinocchio) stay OUT to keep this process
    # pinocchio-free. Bonus: this also removes the per-fresh-frame pinocchio +
    # FK + 5x zmq that used to run in the main work_loop.
    pub = None
    pub_side = str(cfg.get('side', 'arm'))
    if cfg.get('pub_port'):
        try:
            from ..state_pub import StatePublisher
            pub = StatePublisher(int(cfg['pub_port']))
            print(f"\033[36m[device_io] state PUB on :{cfg['pub_port']} "
                  f"(RT-steady, raw pos/vel/eff)\033[0m", flush=True)
        except Exception as e:
            print(f"\033[33m[device_io] state PUB disabled: {e}\033[0m", flush=True)

    last_cmd_seq = -1
    last_gate = None
    last_clear = int(clear_req.value) if clear_req is not None else 0
    last_rt = time.perf_counter()
    last_fault = 0.0
    last_state = 0.0
    sens_ts = bool(cfg.get('sens_ts', True))
    # arm/gripper timestamp pairing tolerance (mirrors robot_hexarm c97f5ac): the old exact
    # <1ns match dropped a whole cycle on any misalignment -> ~410Hz cap with a gripper.
    ts_pair_tol = float(cfg.get('ts_pair_tol_ms', 1.5 * 1000.0 / cfg['control_hz']))
    # Tight but not busy: poll a bit faster than the report rate so we never
    # miss a fresh state frame or a fresh command, without burning a core.
    # NOTE: poll at 1x control_hz, NOT 2x. Polling faster spins this loop's GIL and
    # STARVES the SDK's own KCP-recv thread (same process/GIL) -> fewer frames surface
    # -> ~380-416Hz. Mirrors the single-process work_loop_hz=control_hz fix (470->496).
    poll_hz = float(cfg.get('device_io_poll_hz', 1.0 * cfg['control_hz']))
    period = 1.0 / max(poll_hz, 1.0)
    # get_status_summary() is a FULL SDK status query — far heavier than the
    # pos/vel/eff read. It is a latched park indicator, so throttle it well below
    # the poll rate (default 20 Hz). NOT the 1ms watchdog path.
    fault_period = 1.0 / max(float(cfg.get('device_io_fault_hz', 20.0)), 1.0)
    # STATE read + PUB may run slower than the poll loop: CMD pickup + fault stay
    # at poll cadence, but the 2x get_simple_motor_status + pack + publish only
    # need to keep up with the fastest STATE consumer (teleop/policy/telemetry,
    # all <= ~250 Hz). Default = poll_hz (BC no-op); set lower to save CPU.
    state_period = 1.0 / max(float(cfg.get('device_io_state_hz', poll_hz)), 1.0)

    # #6 diag: localize where the firmware's ~900 frame/s/arm collapses to ~330 pub/s.
    # Every ~2s, report the rate at each stage: loop iters, valid SDK reads, DISTINCT
    # arm/gripper timestamps (new frames the SDK surfaces), and sync-gate passes
    # (= publishes). arm_new >> pub means the arm+gripper sync gate is the limiter
    # (host-side, fixable); arm_new ~= pub ~= 330 means the SDK/firmware only yields
    # ~330 distinct states. Pure counting, no control-path change; device_io_diag=false disables.
    _diag = bool(cfg.get('device_io_diag', False))
    _d_loop = _d_read = _d_aok = _d_gok = _d_arm = _d_grip = _d_pub = 0
    _d_drainsum = _d_drainn = 0
    _d_prev_arm = _d_prev_grip = None
    _d_mindelta = 1e9  # min ms between consecutive DISTINCT firmware arm timestamps:
    # ~1ms => firmware produces >=1000Hz (we/link drop, host-fixable); ~2.4ms => firmware is 411Hz (vendor)
    _d_last = time.perf_counter()

    while not stop_flag.is_set():
        t0 = time.perf_counter()
        if _diag:
            _d_loop += 1

        # ---- STATE: read SDK, write STATE slot (throttled to device_io_state_hz) --
        if time.perf_counter() - last_state >= state_period:
            last_state = time.perf_counter()
            try:
                # #6 fix: DRAIN the deque to the freshest state each iteration. The SDK
                # pushes ~1000/s, but bursty KCP (10ms batches) + maxlen-10 deque + a
                # single pop surfaced only ~410 (older frames overflow-dropped). Draining
                # the whole buffered batch -> freshest state, higher effective rate.
                a = arm.get_simple_motor_status()
                _drain = 0
                while True:
                    _x = arm.get_simple_motor_status()
                    if _x is None:
                        break
                    a = _x
                    _drain += 1
                g = gripper.get_simple_motor_status() if gripper is not None else None
                if gripper is not None:
                    while True:
                        _y = gripper.get_simple_motor_status()
                        if _y is None:
                            break
                        g = _y
                if _diag:
                    _d_drainsum += _drain
                    _d_drainn += 1
                if _diag:
                    if a is not None:
                        _d_aok += 1
                        _ak = (a['ts'].get('s'), a['ts'].get('ns'))
                        if _ak != _d_prev_arm:
                            _d_arm += 1
                            if _d_prev_arm is not None:
                                _dms = ((_ak[0] - _d_prev_arm[0]) * 1000.0
                                        + (_ak[1] - _d_prev_arm[1]) / 1e6)
                                if 0.0 < _dms < _d_mindelta:
                                    _d_mindelta = _dms
                            _d_prev_arm = _ak
                    if g is not None:
                        _d_gok += 1
                        _gk = (g['ts'].get('s'), g['ts'].get('ns'))
                        if _gk != _d_prev_grip:
                            _d_grip += 1
                            _d_prev_grip = _gk
                if a is not None and (gripper is None or g is not None):
                    # match robot_hexarm.__get_states timestamp-sync semantics
                    a_ts = a['ts']
                    g_ts = g['ts'] if g is not None else a_ts
                    if _diag:
                        _d_read += 1
                    if abs(hex_ts_delta_ms(a_ts, g_ts)) < ts_pair_tol:
                        if _diag:
                            _d_pub += 1
                        pos = _concat(a, g, 'pos')
                        vel = _concat(a, g, 'vel')
                        eff = _concat(a, g, 'eff')
                        ts = a_ts if sens_ts else hex_ts_now()  # hex-ts dict
                        state_slot.write(H.pack_state(ts, pos, vel, eff))
                        if pub is not None:
                            # RT-steady raw state stream (no tau_ext/ee → no pinocchio)
                            pub.publish(pub_side, a_ts, pos, vel, eff)
            except Exception:
                pass

        # ---- CMD: read slot, replay onto SDK (no pinocchio) --------------
        # Cheap seqlock-free peek of just cmd_seq; only do the full copy + 7-array
        # unpack when the command actually changed (~75-95% of iterations skip).
        # Monotone cmd_seq + the full seqlock read()+re-check below make a torn
        # single-float peek safe (worst case: one redundant full read).
        try:
            if cmd_slot.peek_seq() != last_cmd_seq:
                c = H.unpack_cmd(cmd_slot.read())
            else:
                c = None
            if c is not None and c['cmd_seq'] != last_cmd_seq:
                last_cmd_seq = c['cmd_seq']
                arm_cmd = arm.construct_mit_command(
                    c['arm_pos'], c['arm_vel'], c['arm_tor'], c['arm_kp'], c['arm_kd'])
                arm.motor_command(CommandType.MIT, arm_cmd)
                if gripper is not None and c['grip_val'].size:
                    mode = c['grip_mode']
                    if mode == GRIP_LIMP:
                        gripper.motor_command(CommandType.TORQUE,
                                              [0.0] * c['grip_val'].size)
                        last_gate = None
                    else:
                        if mode == GRIP_GATED and c['grip_gate'] >= 0 \
                                and c['grip_gate'] != last_gate:
                            gripper.set_pos_torque(c['grip_gate'])
                            last_gate = c['grip_gate']
                        gripper.motor_command(CommandType.POSITION, c['grip_val'])
            # NOTE: when no fresh cmd arrives, we do NOT re-send here — the SDK's
            # own _periodic re-sends the last _target_command to keep the firmware
            # watchdog fed. The MAIN process's safe-hold logic writes a *different*
            # (compliant) command into the CMD slot on client death; that flows
            # through the fresh-cmd path above.
        except Exception as e:
            print(f"\033[91m[device_io] cmd replay error: {e}\033[0m", flush=True)

        # ---- SAFETY-2: clear-fault request + fault-status readback -------
        # Low-rate control channel; not on the 1ms hot path.
        try:
            if clear_req is not None and int(clear_req.value) != last_clear:
                last_clear = int(clear_req.value)
                if hasattr(arm, "clear_parking_stop"):
                    arm.clear_parking_stop()
                if hasattr(arm, "enable_mit"):
                    arm.enable_mit()
                print("\033[36m[device_io] clear_parking_stop + enable_mit\033[0m",
                      flush=True)
            # get_status_summary() is a full SDK query — throttle to device_io_fault_hz
            # (default 20 Hz). fault_active is a latched park indicator; a few tens of
            # ms detection latency is immaterial and this halves the poll-loop cost.
            if (fault_active is not None and hasattr(arm, "get_status_summary")
                    and time.perf_counter() - last_fault >= fault_period):
                last_fault = time.perf_counter()
                s = arm.get_status_summary() or {}
                fault_active.value = 1.0 if s.get("parking_stop_detail") else 0.0
        except Exception:
            pass

        # Re-apply RT to any SDK threads spawned late (KCP reconnect etc.). Cheap;
        # throttled to ~0.5Hz.
        if rt_prio > 0 and (time.perf_counter() - last_rt) > 2.0:
            last_rt = time.perf_counter()
            _set_rt_priority(rt_prio)

        # #6 diag report (every ~2s), per arm side
        if _diag and (time.perf_counter() - _d_last) >= 2.0:
            _e = time.perf_counter() - _d_last
            print(f"\033[35m[device_io diag {pub_side}] loop={_d_loop/_e:.0f} "
                  f"aok={_d_aok/_e:.0f} gok={_d_gok/_e:.0f} arm_new={_d_arm/_e:.0f} "
                  f"grip_new={_d_grip/_e:.0f} pub={_d_pub/_e:.0f} Hz "
                  f"min_dt={_d_mindelta:.2f}ms drain={_d_drainsum / max(_d_drainn, 1):.1f}\033[0m", flush=True)
            _d_loop = _d_read = _d_aok = _d_gok = _d_arm = _d_grip = _d_pub = 0
            _d_drainsum = _d_drainn = 0
            _d_mindelta = 1e9
            _d_last = time.perf_counter()

        dt = time.perf_counter() - t0
        if dt < period:
            time.sleep(period - dt)

    # ---- shutdown --------------------------------------------------------
    try:
        arm.stop()
        api.close()
    except Exception:
        pass
    if pub is not None:
        try:
            pub.close()
        except Exception:
            pass
    state_slot.close()
    cmd_slot.close()
    print("\033[36m[device_io] closed\033[0m", flush=True)

# Entrypoint is run_device_io, launched via multiprocessing.Process(target=...)
# from robot_hexarm.__spawn_device_io (spawn context). Not runnable standalone
# (relative import + shared Event/Value/Queue come from the parent).
