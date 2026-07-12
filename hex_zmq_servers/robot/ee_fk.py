#!/usr/bin/env python3
# -*- coding:utf-8 -*-
"""Shared end-effector FK for the high-rate PUB stream (``<side>/ee_pose`` topic).

ONE implementation used by BOTH the real hexarm device and the MuJoCo sim device,
built from the SAME gr100.urdf the real device already uses for gravity comp — so
the published EE pose has identical conventions on sim and real:

  * frame:  ``link_6`` pose in THIS ARM'S OWN BASE frame (not the dual-arm world;
            apply arm_spacing / right_base_in_left yourself if you need world).
  * payload order: ``[x, y, z, qx, qy, qz, qw]`` — quaternion is **xyzw** (ROS
            ``geometry_msgs/Pose`` convention; note soda_os core uses wxyz internally).

Kept dependency-light: pinocchio imported lazily; construction fails soft (caller
publishes no ``.ee`` topic when unavailable).
"""
from pathlib import Path

import numpy as np

_URDF = Path(__file__).resolve().parent / "hexarm" / "urdf" / "firefly_y6" / "gr100.urdf"


class EEPoseFK:
    """Tiny FK evaluator: 6 arm joints -> [x,y,z,qx,qy,qz,qw] in the arm base frame."""

    def __init__(self, frame: str = "link_6", urdf_path=None):
        import pinocchio as pin  # lazy: only when a pub actually wants .ee
        self._pin = pin
        self._model = pin.buildModelFromUrdf(str(urdf_path or _URDF))
        self._data = self._model.createData()
        self._fid = self._model.getFrameId(frame)
        if self._fid >= self._model.nframes:
            raise ValueError(f"frame {frame!r} not in {_URDF.name}")

    def compute(self, q_arm) -> np.ndarray:
        pin = self._pin
        q = np.asarray(q_arm, dtype=np.float64).ravel()[:self._model.nq]
        pin.forwardKinematics(self._model, self._data, q)
        pin.updateFramePlacement(self._model, self._data, self._fid)
        T = self._data.oMf[self._fid]
        quat = pin.Quaternion(T.rotation).coeffs()  # Eigen coeffs order = (x, y, z, w)
        return np.concatenate([np.asarray(T.translation), np.asarray(quat)])

    def wrench(self, q_arm, tau_arm, sign: float = 1.0) -> np.ndarray:
        """Quasi-static EE wrench ``[fx,fy,fz, mx,my,mz]`` at ``frame`` in the arm BASE
        frame, solving ``J(q)^T F = tau_arm`` (least-squares, so it degrades gracefully
        near singularities instead of blowing up).

        ``tau_arm`` is the per-joint EXTERNAL torque — measured motor effort minus the
        modeled gravity at the measured pose, i.e. the SAME ``<side>/tau_ext`` estimate.
        ``sign`` flips the convention: with the default (+1, matching
        ``tools/ee_wrench_check.py``) the raw solve is reported; set -1 after the
        known-weight calibration so a downward hung load reads as -z (the external
        wrench applied TO the robot). MODEL-BASED, quasi-static — NOT an F/T sensor;
        joint friction sets a noise floor and fast motion adds unmodeled inertial error."""
        pin = self._pin
        q = np.asarray(q_arm, dtype=np.float64).ravel()[:self._model.nq]
        tau = np.asarray(tau_arm, dtype=np.float64).ravel()[:self._model.nq]
        J = np.asarray(pin.computeFrameJacobian(
            self._model, self._data, q, self._fid, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED))
        F, *_ = np.linalg.lstsq(J.T, tau, rcond=None)
        return float(sign) * F
