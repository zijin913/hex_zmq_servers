#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-25
################################################################

import argparse, json, time
import numpy as np
from hex_zmq_servers import (
    HexRate,
    HEX_LOG_LEVEL,
    hex_log,
    HexRobotHexarmClient,
)
from hex_robo_utils import HexDynUtil as DynUtil


def wait_client_working(client, timeout: float = 5.0) -> bool:
    for _ in range(int(timeout * 10)):
        if client.is_working():
            if hasattr(client, "seq_clear"):
                client.seq_clear()
            return True
        else:
            time.sleep(0.1)
    return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    try:
        model_path = cfg["model_path"]
        last_link = cfg["last_link"]
        use_gripper = cfg["use_gripper"]
        hexarm_net_cfg = cfg["hexarm_net_cfg"]
    except KeyError as ke:
        missing_key = ke.args[0]
        raise ValueError(f"cfg is not valid, missing key: {missing_key}")

    # Optional stiffness scales — let the user trade "feels free" against
    # "drifts under bad gravity model". Defaults to ~5% of the server's
    # configured MIT kp/kd, which on firefly_y6 is loose enough to push
    # by hand while still providing a soft "memory" if comp is off.
    #
    # Set kp_scale / kd_scale = 0 for pure-torque (no spring/damper) →
    # truly floating but will drift if gravity model is slightly wrong.
    kp_scale = float(cfg.get("kp_scale", 0.05))
    kd_scale = float(cfg.get("kd_scale", 0.05))
    # Gripper joint (last joint when use_gripper) carries NO gravity load — its
    # comp torque is 0 — so the arm's spring just makes it hard to open/close by
    # hand in zero-g. Scale it separately; default free (kp=0) with light damping
    # so it can be posed by hand without flopping. Raise gripper_kp_scale for a
    # soft "memory" spring, gripper_kd_scale for more drag.
    gripper_kp_scale = float(cfg.get("gripper_kp_scale", 0.0))
    gripper_kd_scale = float(cfg.get("gripper_kd_scale", kd_scale))
    # Per-arm overrides for server-side defaults (must match
    # launchers/configs/{left,right}_arm_cfg.json mit_kp/mit_kd).
    default_kp = np.array(cfg.get("mit_kp",
        [200.0, 200.0, 200.0, 75.0, 15.0, 15.0, 20.0]))
    default_kd = np.array(cfg.get("mit_kd",
        [12.5, 12.5, 12.5, 6.0, 0.31, 0.31, 1.0]))

    hexarm_client = HexRobotHexarmClient(net_config=hexarm_net_cfg)
    dyn_util = DynUtil(model_path, last_link)

    # wait servers to work
    if not wait_client_working(hexarm_client):
        hex_log(HEX_LOG_LEVEL["err"], "hexarm server is not working")
        return

    # Control rate is configurable. 500Hz gives the smoothest comp but is
    # the heaviest on the host (dynamics solve every cycle, ×2 arms). The
    # arm backend's safety watchdog only needs a command every ~300ms, so
    # anything ≥100Hz is safe; drop to ~200Hz if the host can't sustain
    # 500Hz and is tripping PscApiCommunicationTimeout parking stops.
    control_hz = float(cfg.get("control_hz", 500))

    print(f"[zerog] kp_scale={kp_scale}, kd_scale={kd_scale}, "
          f"control_hz={control_hz}")

    # work loop
    rate = HexRate(control_hz)
    last_cmds = None  # most recent command, re-sent on a state read miss
    while True:
        # hexarm
        hexarm_states_hdr, hexarm_states = hexarm_client.get_states()
        if hexarm_states_hdr is not None:
            cur_q = hexarm_states[:, 0]
            cur_dq = hexarm_states[:, 1]
            arm_q = cur_q[:-1] if use_gripper else cur_q
            arm_dq = cur_dq[:-1] if use_gripper else cur_dq
            _, c_mat, g_vec, _, _ = dyn_util.dynamic_params(arm_q, arm_dq)
            tau_comp = c_mat @ arm_dq + g_vec
            if use_gripper:
                tau_comp = np.concatenate((tau_comp, np.zeros(1)), axis=0)
            # 5-column MIT command: [pos, vel, tor, kp, kd]. We hand kp/kd
            # explicitly so the server's default stiffness is overridden
            # by our soft scale; pos = current pose so any residual
            # spring force is ~0 around the current pose.
            n = cur_q.shape[0]
            kp = default_kp[:n] * kp_scale
            kd = default_kd[:n] * kd_scale
            if use_gripper and n >= 1:
                # soften only the gripper joint (last entry)
                kp[-1] = default_kp[n - 1] * gripper_kp_scale
                kd[-1] = default_kd[n - 1] * gripper_kd_scale
            cmds = np.column_stack([
                cur_q,            # pos = where we are now
                np.zeros(n),      # tar_vel — doesn't matter at kd~0
                tau_comp,         # tor — gravity + Coriolis comp
                kp,
                kd,
            ])
            last_cmds = cmds
            hexarm_client.set_cmds(cmds)
        elif last_cmds is not None:
            # State read missed (ZMQ recv timeout). Keep feeding the arm
            # backend its last command so a single slow cycle doesn't open
            # a >300ms command gap and trip the watchdog → parking stop →
            # reconnect thrash. pos is held at the last pose, so the arm
            # stays soft-anchored rather than going limp.
            hexarm_client.set_cmds(last_cmds)

        rate.sleep()


if __name__ == '__main__':
    main()
