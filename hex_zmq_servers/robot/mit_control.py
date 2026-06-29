#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""Shared MIT-arm safety: gravity feedforward + torque clamp.

ONE implementation of the torque-feedforward safety used by BOTH the real hexarm
device (``robot_hexarm.py``) and the MuJoCo sim device (``mujoco_firefly_y6_dual.py``)
so their torque / impedance control behaves IDENTICALLY — the sim is then a faithful
pre-flight test for real hardware.

It is pure logic (numpy only, no pinocchio import): the caller supplies a
``gravity_fn(q) -> tau_g`` closure built from its own pinocchio model, and this module
owns the *behaviour* — which pose gravity is evaluated at, the scale, the effort clamp
and the slew limit. That behaviour is the dangerous part to get wrong (see the
zero-stiffness note below), so keeping it in one place guarantees parity.

Zero-stiffness rule (the raw-torque tipping fix): when the arm command has kp≈0 the arm
is held ONLY by this feedforward, so gravity MUST be evaluated at the MEASURED pose (a
raw-torque command carries a placeholder/zeros position) and at UNITY scale (an
over-unity scale, e.g. a position-mode 1.2 hack, is a net driving torque on an unheld
arm and tips it). Stiff (position) commands keep the commanded-pose + tuned-scale path.
"""

import numpy as np


class MitArmSafety:
    """Gravity feedforward + effort/slew clamp for the arm torque feedforward.

    Args:
        gravity_comp: add gravity feedforward.
        grav_scale: scale on the gravity torque (capped to 1.0 at zero stiffness).
        effort_limit: per-joint |tau| cap (array, or None = no clamp).
        tau_slew: max per-tick change in tau (float, or None = no slew limit).
    """

    ZERO_STIFF_KP = 1e-3  # arm kp at/below this is treated as "no position hold"

    def __init__(self, gravity_comp=False, grav_scale=1.0,
                 effort_limit=None, tau_slew=None, grav_scale_lowstiff=None):
        self._gravity_comp = bool(gravity_comp)
        self._grav_scale = float(grav_scale)
        # Separate gravity scale used at ZERO stiffness (raw torque / kp≈0), where the
        # arm is held only by this feedforward so a model that overestimates gravity
        # DRIVES the arm up. Tune it to the empirical float (run scratchpad/
        # diagnose_gravity.py). Independent of the position-mode scale. If unset, fall
        # back to the position scale capped at 1.0 (never amplify an unheld arm).
        self._grav_scale_lowstiff = (min(self._grav_scale, 1.0)
                                     if grav_scale_lowstiff is None
                                     else float(grav_scale_lowstiff))
        self._effort_limit = (np.asarray(effort_limit, dtype=np.float64)
                              if effort_limit is not None else None)
        self._tau_slew = None if tau_slew is None else float(tau_slew)
        self._last_tau = {}  # per-arm slew state, keyed by slew_key

    def apply(self, arm_tor, cmd_pos_arm, measured_q, cmd_kp_arm,
              gravity_fn, slew_key="default"):
        """Add gravity feedforward (zero-stiffness-safe) then clamp.

        Args:
            arm_tor: feedforward torque for the arm joints (n,).
            cmd_pos_arm: commanded arm position (n,) — used for gravity ONLY when the
                command is stiff (a placeholder in raw-torque commands).
            measured_q: MEASURED arm pose (n,) — used for gravity at zero stiffness.
            cmd_kp_arm: commanded per-joint kp (n,) — decides stiff vs zero-stiffness.
            gravity_fn: ``q -> tau_g`` (the caller's pinocchio gravity), or None.
            slew_key: identifies the arm for per-arm slew state ("left"/"right"/...).
        """
        arm_tor = np.asarray(arm_tor, dtype=np.float64).copy()
        if self._gravity_comp and gravity_fn is not None:
            kp = np.asarray(cmd_kp_arm, dtype=np.float64)
            zero_stiff = float(np.max(kp)) <= self.ZERO_STIFF_KP
            q_grav, g_scale = None, None
            if zero_stiff:
                if measured_q is not None:
                    q_grav = np.asarray(measured_q, dtype=np.float64)
                    g_scale = self._grav_scale_lowstiff
                # else: NO measured pose -> do NOT apply gravity. At zero stiffness
                # the commanded position is a placeholder (zeros in raw torque); using
                # it would drive the unheld arm. Skipping lets the arm sag gently
                # instead of being shot away.
            else:
                q_grav = np.asarray(cmd_pos_arm, dtype=np.float64)
                g_scale = self._grav_scale
            if q_grav is not None:
                try:
                    arm_tor = arm_tor + g_scale * np.asarray(gravity_fn(q_grav),
                                                             dtype=np.float64)
                except Exception as e:
                    print(f"\033[91m[mit_control] gravity failed: {e}\033[0m")
        return self.clamp(arm_tor, slew_key)

    def clamp(self, tau, slew_key="default"):
        """Effort clamp + per-tick slew limit. No-op (returns input) unless an
        effort_limit or tau_slew was configured. Used standalone by the Cartesian
        paths (whose gravity is already in ``tau``)."""
        if self._effort_limit is None and self._tau_slew is None:
            return np.asarray(tau, dtype=np.float64)
        tau = np.asarray(tau, dtype=np.float64).copy()
        if self._effort_limit is not None:
            n = min(tau.shape[0], self._effort_limit.shape[0])
            tau[:n] = np.clip(tau[:n], -self._effort_limit[:n], self._effort_limit[:n])
        if self._tau_slew is not None:
            last = self._last_tau.get(slew_key)
            if last is not None and last.shape == tau.shape:
                step = np.clip(tau - last, -self._tau_slew, self._tau_slew)
                tau = last + step
        self._last_tau[slew_key] = tau.copy()
        return tau

    @staticmethod
    def from_config(robot_config: dict) -> "MitArmSafety":
        """Build from the same config keys both devices use (gravity_comp,
        gravity_comp_scale, torque_safety.{effort_limit_Nm, slew_Nm_per_tick})."""
        ts = robot_config.get("torque_safety", {}) or {}
        return MitArmSafety(
            gravity_comp=bool(robot_config.get("gravity_comp", False)),
            grav_scale=float(robot_config.get("gravity_comp_scale", 1.0)),
            # Per-arm tunable float scale for raw torque / impedance (zero stiffness).
            grav_scale_lowstiff=robot_config.get("gravity_comp_scale_lowstiff"),
            effort_limit=ts.get("effort_limit_Nm"),
            tau_slew=ts.get("slew_Nm_per_tick"),
        )
