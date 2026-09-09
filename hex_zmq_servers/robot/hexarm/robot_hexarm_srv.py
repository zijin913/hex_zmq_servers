#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-14
################################################################

# Crash diagnostics: on a fatal native signal (SIGSEGV/SIGABRT/SIGFPE) dump EVERY
# thread's Python stack to stderr -> the arm-server err log. This is what locates a
# HexFellow-SDK native crash (the segfault seen after repeated
# PscApiCommunicationTimeout parks), which otherwise kills the process with no trace.
import faulthandler
faulthandler.enable()
try:  # best-effort: also allow a core dump (needs the host core_pattern to keep it)
    import resource
    resource.setrlimit(resource.RLIMIT_CORE,
                       (resource.RLIM_INFINITY, resource.RLIM_INFINITY))
except Exception:
    pass

import numpy as np

try:
    from ..robot_base import HexRobotServerBase
    from .robot_hexarm import HexRobotHexarm
except (ImportError, ValueError):
    import sys
    from pathlib import Path
    this_file = Path(__file__).resolve()
    project_root = this_file.parents[3]
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))
    from hex_zmq_servers.robot.robot_base import HexRobotServerBase
    from hex_zmq_servers.robot.hexarm.robot_hexarm import HexRobotHexarm

NET_CONFIG = {
    "ip": "127.0.0.1",
    "port": 12345,
    "realtime_mode": False,
    "deque_maxlen": 10,
    "client_timeout_ms": 200,
    "server_timeout_ms": 1_000,
    "server_num_workers": 4,
}

ROBOT_CONFIG = {
    "device_ip": "172.18.8.161",
    "device_port": 8439,
    "control_hz": 250,
    "arm_type": "archer_l6y",
    "mit_kp": [200.0, 200.0, 200.0, 75.0, 15.0, 15.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "mit_kd": [12.5, 12.5, 12.5, 6.0, 0.31, 0.31, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    "sens_ts": True,
}


class HexRobotHexarmServer(HexRobotServerBase):

    def __init__(
        self,
        net_config: dict = NET_CONFIG,
        params_config: dict = ROBOT_CONFIG,
    ):
        HexRobotServerBase.__init__(self, net_config)

        # robot
        self._device = HexRobotHexarm(params_config,
                                      net_config.get("realtime_mode", False))

    def _process_request(self, recv_hdr: dict, recv_buf: np.ndarray):
        command = recv_hdr["cmd"]
        if command in {
            "seq_clear", "set_cmds", "set_control_mode", "clear_fault"
        } and not self.mutation_authorized(recv_hdr):
            # State/health remain observable while fullbody is down. Motion,
            # mode changes and fault clears are capability-gated at the last
            # software boundary before the real device.
            return self.no_ts_hdr(recv_hdr, False), None
        if command == "is_working":
            return self.no_ts_hdr(recv_hdr, self._device.is_working()), None
        elif command == "seq_clear":
            return self.no_ts_hdr(recv_hdr, self._seq_clear()), None
        elif command == "get_dofs":
            dofs = self._device.get_dofs()
            return self.no_ts_hdr(recv_hdr, dofs is not None), dofs
        elif command == "get_limits":
            limits = self._device.get_limits()
            return self.no_ts_hdr(recv_hdr, limits is not None), limits
        elif command == "get_states":
            return self._get_states(recv_hdr)
        elif command == "set_cmds":
            return self._set_cmds(recv_hdr, recv_buf)
        elif command == "set_control_mode":
            ok = self._device.set_control_mode(recv_hdr.get("args"))
            return self.no_ts_hdr(recv_hdr, ok), None
        elif command == "clear_fault":
            ok = self._device.clear_fault()
            return self.no_ts_hdr(recv_hdr, ok), None
        elif command == "get_fault":
            fault = self._device.get_fault()
            return self.no_ts_hdr(recv_hdr, fault is not None), fault
        else:
            raise ValueError(f"unknown command: {recv_hdr['cmd']}")


if __name__ == "__main__":
    import argparse, json
    from hex_zmq_servers.zmq_base import hex_server_helper

    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    hex_server_helper(cfg, HexRobotHexarmServer)
