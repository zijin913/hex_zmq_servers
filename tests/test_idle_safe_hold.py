"""Unit tests for the client-death safe-hold logic (SAFETY-1, IF-X Pilot 0).

Pure numpy — no device / SDK needed, so it runs on SOMA. Validates the two
helpers robot_hexarm's work_loop uses to fail safe when a client dies:
  * idle_gone_stale  — when to drop to the compliant hold
  * build_safe_hold_cmd — the gravity-comp compliant command (arm soft, gripper clamped)

Run on SOMA:
    cd ~/Projects/soda-bimanual && python -m pytest \
        hex_zmq_servers/tests/test_idle_safe_hold.py -q
"""
import numpy as np
from hex_zmq_servers.robot.mit_control import idle_gone_stale, build_safe_hold_cmd


def test_idle_gone_stale_trips_after_timeout():
    # signature: idle_gone_stale(silent_ms, idle_hold_max_ms) — silent_ms is the
    # elapsed ms since the last FRESH command (hex_ts_delta_ms at the call site).
    assert idle_gone_stale(3000.0, 2000.0) is True      # 3s silent > 2s cap
    assert idle_gone_stale(1500.0, 2000.0) is False     # 1.5s < 2s: hold last
    assert idle_gone_stale(2000.0, 2000.0) is False     # exactly at cap: not yet
    assert idle_gone_stale(2000.1, 2000.0) is True


def test_idle_gone_stale_disabled():
    # 0 or None disables the safe-stop -> legacy hold-last-forever
    assert idle_gone_stale(1e9, 0) is False
    assert idle_gone_stale(1e9, None) is False


def test_safe_hold_holds_measured_pose_arm_soft_gripper_clamped():
    q = np.array([0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])   # 6 arm + 1 gripper
    arm_idx, grip_idx = list(range(6)), [6]
    cmd = build_safe_hold_cmd(
        q, arm_idx, grip_idx,
        arm_kp=np.full(6, 30.0), arm_kd=np.full(6, 6.0),
        gripper_kp=20.0, gripper_kd=1.0)

    assert cmd.shape == (7, 5)                     # (dofs, 5) MIT command
    assert np.allclose(cmd[:, 0], q)               # holds the MEASURED pose
    assert np.allclose(cmd[:, 1], 0.0)             # zero target velocity
    assert np.allclose(cmd[:, 2], 0.0)             # zero tau_ff (gravity added downstream)
    assert np.all(cmd[arm_idx, 3] == 30.0)         # arm: soft compliant kp
    assert np.all(cmd[arm_idx, 4] == 6.0)
    # gripper stays position-held (kp > 1e-6 -> __set_cmds treats it as clamped)
    assert cmd[6, 3] == 20.0 and cmd[6, 3] > 1e-6


def test_safe_hold_pure_float_still_clamps_gripper():
    q = np.zeros(7)
    cmd = build_safe_hold_cmd(q, list(range(6)), [6],
                              arm_kp=0.0, arm_kd=0.0,
                              gripper_kp=20.0, gripper_kd=1.0)
    assert np.all(cmd[:6, 3] == 0.0)               # pure zero-gravity float (kp=0)
    assert cmd[6, 3] == 20.0                        # gripper NOT released
