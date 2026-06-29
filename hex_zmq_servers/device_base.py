#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-16
################################################################

import threading
from collections import deque
from abc import ABC, abstractmethod

import numpy as np

from .zmq_base import MAX_SEQ_NUM


class HexDeviceBase(ABC):

    def __init__(self, realtime_mode: bool = False):
        # variables
        self._max_seq_num = MAX_SEQ_NUM
        self._realtime_mode = realtime_mode
        # thread
        self._working = threading.Event()

    def __del__(self):
        self.close()

    def is_working(self) -> bool:
        return self._working.is_set()

    def _wait_for_working(self):
        while not self._working.is_set():
            print("waiting for device to work")
            self._working.wait(0.1)

    @staticmethod
    def _rads_normalize(rads: np.ndarray) -> np.ndarray:
        """Wrap angle(s) to [-pi, pi)."""
        return (rads + np.pi) % (2.0 * np.pi) - np.pi

    @staticmethod
    def _apply_pos_limits(
        rads: np.ndarray,
        lower_bound: np.ndarray,
        upper_bound: np.ndarray,
    ) -> np.ndarray:
        """Clamp joint targets to [lower, upper], normalizing RELATIVE TO EACH
        JOINT'S RANGE CENTER first. A plain wrap to [-pi, pi) flips a joint whose
        range upper bound sits on the +pi boundary (e.g. joint_3 = [0, 3.14]): a
        near-pi target wraps to ~-pi, is judged out-of-range, and snaps to the
        OPPOSITE bound (0.0) — a ~pi discontinuity that, under stiff PD, becomes a
        max-torque shove. Wrapping about the range center keeps a near-bound target
        near that bound (for any range < 2*pi wide), so the clip is monotone.
        SHARED by the sim (HexMujocoBase) and real (HexRobotBase) devices — single
        source of truth so both plants clamp joint targets IDENTICALLY."""
        lower_bound = np.asarray(lower_bound, dtype=np.float64)
        upper_bound = np.asarray(upper_bound, dtype=np.float64)
        center = 0.5 * (lower_bound + upper_bound)
        normed_rads = center + HexDeviceBase._rads_normalize(
            np.asarray(rads, dtype=np.float64) - center)
        return np.clip(normed_rads, lower_bound, upper_bound)

    @abstractmethod
    def work_loop(self, hex_queues: list[deque | threading.Event]):
        raise NotImplementedError(
            "`work_loop` should be implemented by the child class")

    @abstractmethod
    def close(self):
        raise NotImplementedError(
            "`close` should be implemented by the child class")
