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
import threading
import gc
from collections import deque
import numpy as np

from hex_device import HexDeviceApi
from hex_device.motor_base import CommandType
from hex_robo_utils import hex_ts_now, hex_ts_delta_ms

from . import hexarm_shmem as H

# grip_mode enum (must match robot_hexarm's writer)
GRIP_POSITION = 0   # normal position control
GRIP_LIMP = 1       # zero-torque limp (compliant hand-posing)
GRIP_GATED = 2      # widen set_pos_torque gate, then position
GRIP_HOLD = 3       # force-controlled grasp: hold at a set TORQUE (no position stall)


def _concat(a, g, key):
    return np.concatenate([a[key], g[key]]) if g is not None else np.asarray(a[key])


def _isolated_cpus() -> set:
    """CPUs the kernel was booted with isolcpus= (empty set if none / unreadable).

    Used to move the PUB worker onto HOUSEKEEPING cores specifically, rather
    than merely widening its mask to every CPU -- see _pub_worker.
    """
    try:
        raw = open("/sys/devices/system/cpu/isolated").read().strip()
    except Exception:
        return set()
    out = set()
    for part in raw.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            lo, hi = part.split("-", 1)
            out.update(range(int(lo), int(hi) + 1))
        else:
            out.add(int(part))
    return out


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

    # decoupled state-PUB handoff: read loop writes latest (tag, ts, pos, vel, eff);
    # _pub_worker below oversamples + sends it off the read hot path.
    # Read loop -> pub worker handoff. This was a single latest-wins slot, which
    # silently coalesced: any state produced between two worker samples was
    # overwritten and never published. Measured on soda-can that cost 1-9
    # states/s out of 500 -- the entire gap between what the firmware produced
    # and what a subscriber received (ZMQ refused 0, the ts-pair gate dropped 0).
    #
    # The slot only worked if the worker sampled strictly faster than the source
    # produced, and its rate is max(2x control_hz, 1000) = exactly 2x at
    # control_hz=500 -- no margin, so any wakeup jitter lost a frame. A BOUNDED
    # queue removes that requirement entirely: the worker drains whatever
    # accumulated, so being late costs latency, not data.
    #
    # Still bounded, so the original safety property holds -- under sustained
    # overload it drops rather than growing without limit or blocking the read
    # loop. maxlen 8 is ~16 ms of backlog at 500 Hz. deque append/popleft are
    # atomic under the GIL, so no lock is needed on this path.
    _PUB_Q_MAX = 8
    _pub_q = deque(maxlen=_PUB_Q_MAX)
    _pub_tag = [0]
    # Accounting for the last unmeasured hop. _d_pub (read loop) counts states
    # OFFERED to the slot; the slot is latest-wins, so a state the pub worker
    # never samples is silently overwritten and never reaches ZMQ. Until now
    # nothing counted that: _d_pub said "offered", drop= said "ZMQ refused",
    # and the difference between them had no name.
    #   _pub_sent    -> publish() calls actually made
    #   _pub_coalesced -> states overwritten before the worker sampled them,
    #                     measured directly as the tag gap (the tag is a
    #                     monotonic counter, so a jump of N skipped N-1 states)
    _pub_sent = [0]
    _pub_coalesced = [0]

    last_cmd_seq = -1
    last_gate = None
    last_clear = int(clear_req.value) if clear_req is not None else 0
    last_state = 0.0
    sens_ts = bool(cfg.get('sens_ts', True))
    # arm/gripper timestamp pairing tolerance (mirrors robot_hexarm c97f5ac): the old exact
    # <1ns match dropped a whole cycle on any misalignment -> ~410Hz cap with a gripper.
    ts_pair_tol = float(cfg.get('ts_pair_tol_ms', 1.5 * 1000.0 / cfg['control_hz']))
    # Tight but not busy: poll a bit faster than the report rate so we never
    # miss a fresh state frame or a fresh command, without burning a core.
    # Poll at 2x control_hz so the single-pop read stays ahead of the firmware's
    # ~500/s delivery (SDK deque stays empty, no aliasing). The state PUB is DECOUPLED
    # to _pub_worker (below), so this loop neither serializes/sends nor drains inline.
    poll_hz = float(cfg.get('device_io_poll_hz', 2.0 * cfg['control_hz']))
    period = 1.0 / max(poll_hz, 1.0)
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
    # Env fallback. In REAL mode the launcher re-renders launchers/configs/*.json
    # from site.yaml on every start (_launch_servers_impl: "site.yaml is
    # authoritative"), so hand-editing the rendered config -- which is what
    # state-rate-measurement.md tells you to do -- is silently reverted before
    # launch. site.yaml itself has no device_io_diag field and _build_arm_cfg has
    # no passthrough, so there is no config route at all in real mode. The env var
    # gives one, without adding a diagnostic-only knob to the site schema.
    _diag = (bool(cfg.get('device_io_diag', False))
             or os.environ.get('SODA_DEVICE_IO_DIAG', '').strip().lower()
             not in ('', '0', 'false', 'no', 'off'))
    _d_loop = _d_read = _d_aok = _d_gok = _d_arm = _d_grip = _d_pub = 0
    _d_drainsum = _d_drainn = 0
    _d_prev_arm = _d_prev_grip = None
    _d_mindelta = 1e9  # min ms between consecutive DISTINCT firmware arm timestamps:
    # ~1ms => firmware produces >=1000Hz (we/link drop, host-fixable); ~2.4ms => firmware is 411Hz (vendor)
    _d_last = time.perf_counter()

    # ---- decoupled state PUB (mirrors rt-nic __pub_worker) --------------------
    # Serialize + ZMQ-send the latest state OFF the read hot path so the read loop
    # runs at full firmware rate. Oversample at max(2x control, 1kHz) to emit every
    # frame regardless of phase. Non-RT and moved onto the housekeeping cores so it
    # never contends the FF read loop / SDK _periodic on the isolated core -- and so
    # its ZMQ sends run where interrupts and softirqs are actually serviced.
    def _pub_worker():
        # Move to the HOUSEKEEPING cores, not merely to "every core".
        #
        # This previously did sched_setaffinity(0, set(range(os.cpu_count()))),
        # which on an 8-CPU box sets the mask to 0-7 -- i.e. it INCLUDES the
        # isolated cores. The thread inherits the process pin (the isolated
        # device-io core) and isolcpus takes those CPUs out of load balancing,
        # so nothing ever migrates it out: the "unpin" widened the mask but
        # left the thread exactly where it was. Measured on soda-can, 20
        # samples of both arms' PUB threads landed 0 times on a housekeeping
        # core; they sat on the SCHED_FIFO-80 control-loop cores, where a
        # normal-priority 1 kHz publisher is preempted unconditionally, and
        # where irqaffinity= has steered device IRQs away.
        #
        # Subtracting the isolated set makes the placement match the intent
        # that 8377b9a established: network I/O belongs on housekeeping cores.
        # Falls back to the full mask if isolcpus is empty or unreadable, which
        # reproduces the old behaviour on a non-isolated box.
        try:
            _all = set(range(os.cpu_count()))
            _hk = _all - _isolated_cpus()
            os.sched_setaffinity(0, _hk or _all)
        except Exception:
            pass
        try:
            print("\033[36m[device_io] pub worker on cpus %s (isolated %s)\033[0m"
                  % (sorted(os.sched_getaffinity(0)), sorted(_isolated_cpus()) or "none"),
                  flush=True)
        except Exception:
            pass
        _ph = float(cfg.get('device_io_pub_hz', max(2.0 * cfg['control_hz'], 1000.0)))
        _per = 1.0 / max(_ph, 1.0)
        while not stop_flag.is_set():
            _t0 = time.perf_counter()
            # Drain everything queued since the last wakeup, not just the newest.
            while pub is not None:
                try:
                    _sl = _pub_q.popleft()
                except IndexError:
                    break
                _pub_sent[0] += 1
                try:
                    # _sl[5] = receipt host_ts captured in the read loop before this
                    # queue; publish it verbatim rather than stamping send-time here.
                    pub.publish(pub_side, _sl[1], _sl[2], _sl[3], _sl[4],
                                host_ts=_sl[5])
                except Exception:
                    pass
            _dt = time.perf_counter() - _t0
            if _dt < _per:
                time.sleep(_per - _dt)
    if pub is not None:
        threading.Thread(target=_pub_worker, name='devio_pub', daemon=True).start()

    # ---- fault readback OFF the read loop ------------------------------------
    # get_status_summary() is a heavy full SDK query; polling it in the read loop
    # (even at 20Hz) stalled it ~2-3ms each time and the drain-to-freshest then
    # discarded the frames buffered during the stall -> ~7% loss (~465 vs 500Hz).
    # rt-nic never queried status in its control loop. Run it in its own thread.
    def _fault_worker():
        if fault_active is None or not hasattr(arm, 'get_status_summary'):
            return
        _fper = 1.0 / max(float(cfg.get('device_io_fault_hz', 20.0)), 1.0)
        while not stop_flag.is_set():
            _t0 = time.perf_counter()
            try:
                _s = arm.get_status_summary() or {}
                fault_active.value = 1.0 if _s.get('parking_stop_detail') else 0.0
            except Exception:
                pass
            _dt = time.perf_counter() - _t0
            if _dt < _fper:
                time.sleep(_fper - _dt)
    threading.Thread(target=_fault_worker, name='devio_fault', daemon=True).start()

    # ---- GC polish: kill the periodic gen2-collection freezes on the read loop ---
    # The loop allocates per tick; automatic GC then stops-the-world 5-15ms at
    # unpredictable times (the ~12ms max). Freeze the startup heap (never rescanned),
    # disable AUTO gc, and do a rare manual collect off-path. Per-loop objects are
    # refcounted (no cycles), so nothing accumulates between collects.
    gc.collect()
    try:
        gc.freeze()
    except Exception:
        pass
    gc.disable()

    def _housekeeping():
        # off the read hot path: rare manual GC + re-assert RT on late SDK threads
        # (KCP reconnect), only when the thread count actually changes.
        _ntask = [len(os.listdir('/proc/self/task'))]
        while not stop_flag.is_set():
            time.sleep(10.0)
            try:
                n = len(os.listdir('/proc/self/task'))
                if rt_prio > 0 and n != _ntask[0]:
                    _ntask[0] = n
                    _set_rt_priority(rt_prio)
                gc.collect()
            except Exception:
                pass
    threading.Thread(target=_housekeeping, name='devio_hk', daemon=True).start()

    while not stop_flag.is_set():
        t0 = time.perf_counter()
        if _diag:
            _d_loop += 1

        # ---- STATE: read SDK, write STATE slot (throttled to device_io_state_hz) --
        if time.perf_counter() - last_state >= state_period:
            last_state = time.perf_counter()
            try:
                # single-pop (mirrors rt-nic). With the oversampled poll (> firmware
                # rate) the SDK deque stays empty (diag drain=0.0), so the while-drain
                # only burned an extra get() per side AND pushed the loop work past the
                # poll period so it NEVER slept -> it hogged the GIL from the SDK's
                # _periodic DOWN-send thread -> ~470 sends/s -> firmware 470. One pop
                # each; the loop now sleeps and _periodic gets the GIL to send ~500.
                a = arm.get_simple_motor_status()
                # RECEIPT stamp: the first instant this process has the frame, taken
                # BEFORE the read-loop -> pub-worker queue. Published verbatim (the pub
                # worker does NOT re-stamp), so host_ts is free of queue-depth/drain
                # jitter -- its only error is up to one read-loop period (~1 ms). Uses
                # time.monotonic() to match the camera's host_ts clock exactly, so arm
                # and camera stamps are cross-comparable on one NUC.
                _recv_host_ts = time.monotonic()
                _drain = 0
                g = gripper.get_simple_motor_status() if gripper is not None else None
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
                            # hand off latest-wins to _pub_worker (decoupled); do NOT
                            # serialize/send inline -- that made read+pub serial (~2ms/
                            # iter -> ~416Hz) and dropped frames during the send window.
                            _pub_tag[0] += 1
                            # deque(maxlen) discards the OLDEST on overflow, so
                            # count that here -- it is the only remaining way a
                            # state can be lost before reaching ZMQ.
                            if len(_pub_q) == _PUB_Q_MAX:
                                _pub_coalesced[0] += 1
                            _pub_q.append((_pub_tag[0], a_ts, pos, vel, eff,
                                           _recv_host_ts))
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
                    elif mode == GRIP_HOLD:
                        # force-controlled grasp: hold at grip_gate TORQUE (no
                        # position stall -> no overheat; eff tracks torque).
                        gripper.motor_command(CommandType.TORQUE, [c['grip_gate']] * c['grip_val'].size)
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
            # (fault readback moved OFF this loop to _fault_worker — get_status_summary
            # is a heavy full SDK query that stalled the read here ~2-3ms every 50ms,
            # and the drain-to-freshest then discarded the buffered frames -> ~7% loss.)
        except Exception:
            pass

        # (RT re-apply moved OFF the read loop to the _housekeeping thread — the
        # in-loop /proc/self/task scan every 2s was itself a periodic stall.)

        # #6 diag report (every ~2s), per arm side
        if _diag and (time.perf_counter() - _d_last) >= 2.0:
            _e = time.perf_counter() - _d_last
            # drop = frames the PUB socket refused on a full SNDHWM. NOTE the unit:
            # `pub` counts publish() CALLS, each of which emits 4 topic frames
            # (pos/vel/eff/joint_states), so a drop rate of ~4N/s corresponds to N
            # lost states/s. drop>0 means the shortfall a subscriber sees is OURS
            # (raise SNDHWM); drop==0 means we sent everything and the loss is
            # downstream, in transport or in the subscriber.
            _d_drop = pub.take_dropped() if pub is not None else 0
            print(f"\033[35m[device_io diag {pub_side}] loop={_d_loop/_e:.0f} "
                  f"aok={_d_aok/_e:.0f} gok={_d_gok/_e:.0f} arm_new={_d_arm/_e:.0f} "
                  f"grip_new={_d_grip/_e:.0f} pub={_d_pub/_e:.0f} Hz "
                  f"sent={_pub_sent[0]/_e:.0f} coal={_pub_coalesced[0]/_e:.0f} "
                  f"drop={_d_drop/_e:.0f}/s "
                  f"min_dt={_d_mindelta:.2f}ms drain={_d_drainsum / max(_d_drainn, 1):.1f}\033[0m", flush=True)
            _pub_sent[0] = _pub_coalesced[0] = 0
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
