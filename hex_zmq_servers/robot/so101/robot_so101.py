#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# SO-101 Leader Arm Device Driver
# Uses Feetech STS3215 servos via scservo_sdk
# Modeled after robot_gello.py (Dynamixel-based)
################################################################

import os
import subprocess
import time
import threading
import numpy as np
from collections import deque

from ..robot_base import HexRobotBase
from ...zmq_base import (
    hex_zmq_ts_now,
    hex_zmq_ts_delta_ms,
    HexRate,
)
from ...hex_launch import hex_log, HEX_LOG_LEVEL

import scservo_sdk as scs

# Feetech STS3215 register addresses
SCS_PRESENT_POSITION_ADDR = 56
SCS_PRESENT_POSITION_LEN = 2
SCS_GOAL_POSITION_ADDR = 42
SCS_GOAL_POSITION_LEN = 2
SCS_TORQUE_ENABLE_ADDR = 40
SCS_TORQUE_ENABLE_VAL = 1
SCS_TORQUE_DISABLE_VAL = 0

ROBOT_CONFIG = {
    "idxs": [1, 2, 3, 4, 5, 6],
    "invs": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
    # Midpoint of [range_min, range_max] from LeRobot calibration.
    # raw Present_Position already has homing_offset applied in hardware,
    # so we subtract range midpoint to center the values around 0.
    "range_midpoints": [2029, 2025, 1938, 2095, 2047, 2069],
    "limits": [
        [-2.7, 2.7],
        [-2.09, 2.09],
        [-3.14, 3.14],
        [-1.57, 1.57],
        [-3.14, 3.14],
        [0.0, 1.0],
    ],
    "device": "/dev/ttyACM0",
    "baudrate": 1000000,
    "max_retries": 3,
    "torque_enabled": False,
    "sens_ts": True,
}


def _decode_sign_magnitude(raw: int) -> int:
    """Decode Feetech sign-magnitude encoding (bit 15 = sign)."""
    if raw & 0x8000:
        return -(raw & 0x7FFF)
    return raw


class HexRobotSO101(HexRobotBase):

    def __init__(
        self,
        robot_config: dict = ROBOT_CONFIG,
        realtime_mode: bool = False,
    ):
        HexRobotBase.__init__(self, realtime_mode)

        try:
            self.__idxs = np.array(robot_config["idxs"])
            self.__invs = np.array(robot_config["invs"])
            self._limits = np.array(robot_config["limits"])
            self.__device = robot_config["device"]
            self.__baudrate = robot_config["baudrate"]
            self.__max_retries = robot_config["max_retries"]
            self.__range_midpoints = np.array(robot_config.get("range_midpoints",
                                                                     [2048] * len(robot_config["idxs"])))
            self.__torque_enabled = robot_config["torque_enabled"]
            self.__sens_ts = robot_config["sens_ts"]
        except KeyError as ke:
            missing_key = ke.args[0]
            raise ValueError(
                f"robot_config is not valid, missing key: {missing_key}")

        # STS3215: 4096 steps per revolution
        self.__servo_to_rad = np.pi / 2048
        self.__port_handler = None
        self.__packet_handler = None
        self.__group_sync_read = None
        self.__lock = threading.Lock()
        self._dofs = [self.__idxs.shape[0]]

        # open device
        for attempt in range(self.__max_retries):
            print(
                f"Attempting to initialize Feetech driver (attempt {attempt + 1}/{self.__max_retries})"
            )
            if self.__try_open_device():
                break
            else:
                time.sleep(1.0)

        # start work loop
        self._working.set()

    def work_loop(self, hex_queues: list[deque | threading.Event]):
        states_queue = hex_queues[0]
        cmds_queue = hex_queues[1]
        stop_event = hex_queues[2]

        states_count = 0
        last_cmds_seq = -1
        rate = HexRate(1000)
        while self._working.is_set() and not stop_event.is_set():
            # states
            ts, states = self.__get_states()
            if states is not None:
                states_queue.append((ts, states_count, states))
                states_count = (states_count + 1) % self._max_seq_num

            # cmds (SO-101 leader typically doesn't receive commands,
            # but we handle them for completeness)
            cmds_pack = None
            try:
                cmds_pack = cmds_queue[
                    -1] if self._realtime_mode else cmds_queue.popleft()
            except IndexError:
                pass
            if cmds_pack is not None:
                ts, seq, cmds = cmds_pack
                if seq != last_cmds_seq:
                    last_cmds_seq = seq
                    if hex_zmq_ts_delta_ms(hex_zmq_ts_now(), ts) < 200.0:
                        self.__set_cmds(cmds)

            # sleep
            rate.sleep()

        # close
        self.close()

    def __get_states(self):
        with self.__lock:
            ts = hex_zmq_ts_now()
            servo_values = np.zeros(self._dofs, dtype=int)
            dxl_comm_result = self.__group_sync_read.txRxPacket()
            if dxl_comm_result != scs.COMM_SUCCESS:
                print(f"warning, comm failed: {dxl_comm_result}")
                return None, None
            for i, motor_id in enumerate(self.__idxs):
                if self.__group_sync_read.isAvailable(
                        motor_id, SCS_PRESENT_POSITION_ADDR,
                        SCS_PRESENT_POSITION_LEN):
                    raw_value = self.__group_sync_read.getData(
                        motor_id, SCS_PRESENT_POSITION_ADDR,
                        SCS_PRESENT_POSITION_LEN)
                    servo_values[i] = _decode_sign_magnitude(raw_value)
                else:
                    raise RuntimeError(
                        f"Failed to get position for Feetech motor ID {motor_id}"
                    )

            # Subtract range midpoint to center values around 0
            # (homing_offset is already applied in motor EEPROM)
            centered = servo_values - self.__range_midpoints
            rads = self._apply_pos_limits(
                centered * self.__servo_to_rad * self.__invs,
                self._limits[:, 0],
                self._limits[:, 1],
            )
            return ts if self.__sens_ts else hex_zmq_ts_now(), rads

    def __set_cmds(self, cmds: np.ndarray):
        """Write goal positions (for force feedback in the future)."""
        if len(cmds) != len(self.__idxs):
            print(
                "\033[91mThe length of commands must match the number of servos\033[0m"
            )
            return False
        if not self.__torque_enabled:
            return False

        group_sync_write = scs.GroupSyncWrite(
            self.__port_handler,
            self.__packet_handler,
            SCS_GOAL_POSITION_ADDR,
            SCS_GOAL_POSITION_LEN,
        )

        for motor_id, cmd_rad in zip(self.__idxs, cmds):
            position_value = int(cmd_rad / self.__servo_to_rad)
            # Encode as 2 bytes
            param = [
                scs.SCS_LOBYTE(position_value),
                scs.SCS_HIBYTE(position_value),
            ]
            if not group_sync_write.addParam(motor_id, param):
                print(
                    f"\033[91mFailed to set goal for Feetech motor ID {motor_id}\033[0m"
                )
                return False

        dxl_comm_result = group_sync_write.txPacket()
        group_sync_write.clearParam()

        if dxl_comm_result != scs.COMM_SUCCESS:
            print(f"\033[91mFailed to syncwrite goal position\033[0m")
            return False
        return True

    def __try_open_device(self):
        if not self.__check_device_availability():
            print("Port is busy, attempting to free it...")
            if not self.__kill_processes_using_device():
                print("Failed to free device, trying to fix permissions...")
                self.__fix_device_permissions()
            time.sleep(2)

        try:
            self.__initialize_hardware()
            print(
                f"Successfully initialized Feetech driver on {self.__device}"
            )
            return True
        except Exception as e:
            print(f"Failed to initialize Feetech driver: {e}")
            if "Permission denied" in str(e):
                print(
                    "Please add permission to the device: sudo chmod 666 "
                    + self.__device)
                return False
            return False

    def __check_device_availability(self):
        try:
            if not os.path.exists(self.__device):
                print(f"Device {self.__device} does not exist")
                return False
            result = subprocess.run(["lsof", self.__device],
                                    capture_output=True,
                                    text=True)
            if result.returncode == 0:
                lines = result.stdout.strip().split("\n")
                if len(lines) > 1:
                    print(
                        f"Device {self.__device} is being used by other processes:"
                    )
                    for line in lines[1:]:
                        print(f"  {line}")
                    return False
            return True
        except Exception as e:
            print(f"Error checking device availability: {e}")
            return False

    def __kill_processes_using_device(self):
        try:
            result = subprocess.run(
                ["fuser", "-k", self.__device],
                capture_output=True,
                text=True,
            )
            if result.returncode == 0:
                print(f"Killed processes using {self.__device}")
                time.sleep(1)
                return True
            return False
        except Exception as e:
            print(f"Error killing processes: {e}")
            return False

    def __fix_device_permissions(self):
        try:
            result = subprocess.run(["sudo", "chmod", "666", self.__device],
                                    capture_output=True,
                                    text=True)
            if result.returncode == 0:
                print(f"Fixed permissions for {self.__device}")
                return True
            return False
        except Exception as e:
            print(f"Error fixing device permissions: {e}")
            return False

    def __initialize_hardware(self):
        self.__prepare_device()

        self.__port_handler = scs.PortHandler(self.__device)
        self.__packet_handler = scs.PacketHandler(0)  # Protocol 0 for STS3215
        self.__group_sync_read = scs.GroupSyncRead(
            self.__port_handler,
            self.__packet_handler,
            SCS_PRESENT_POSITION_ADDR,
            SCS_PRESENT_POSITION_LEN,
        )

        if not self.__port_handler.openPort():
            raise RuntimeError("Failed to open the port")

        if not self.__port_handler.setBaudRate(self.__baudrate):
            raise RuntimeError(
                f"Failed to change the baudrate, {self.__baudrate}")

        for motor_id in self.__idxs:
            if not self.__group_sync_read.addParam(motor_id):
                raise RuntimeError(
                    f"Failed to add parameter for Feetech motor ID {motor_id}")

        if self.__torque_enabled:
            try:
                self.__hardware_set_torque_mode(self.__torque_enabled)
            except Exception as e:
                print(f"device: {self.__device}, {e}")

    def __prepare_device(self):
        if not self.__check_device_availability():
            print(
                f"Device {self.__device} is not available, attempting to fix..."
            )
            self.__kill_processes_using_device()
            self.__fix_device_permissions()
            if not self.__check_device_availability():
                print(f"Warning: Device {self.__device} may still have issues")

    def __hardware_set_torque_mode(self, enable: bool):
        torque_value = SCS_TORQUE_ENABLE_VAL if enable else SCS_TORQUE_DISABLE_VAL
        with self.__lock:
            for motor_id in self.__idxs:
                dxl_comm_result, dxl_error = self.__packet_handler.write1ByteTxRx(
                    self.__port_handler, motor_id, SCS_TORQUE_ENABLE_ADDR,
                    torque_value)
                if dxl_comm_result != scs.COMM_SUCCESS or dxl_error != 0:
                    raise RuntimeError(
                        f"Failed to set torque mode for Feetech motor ID {motor_id}"
                    )
        self.__torque_enabled = enable

    def close(self):
        if not self._working.is_set():
            return
        self._working.clear()
        if self.__port_handler is not None:
            self.__port_handler.closePort()
        hex_log(HEX_LOG_LEVEL["info"], "HexRobotSO101 closed")
