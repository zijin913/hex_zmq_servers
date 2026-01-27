#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# Copyright 2025 Dong Zhaorui. All rights reserved.
# Author: Dong Zhaorui 847235539@qq.com
# Date  : 2025-09-14
################################################################

import numpy as np
from collections import deque

try:
    from ..robot_base import HexRobotServerBase
    from .robot_hello import HexRobotHello
except (ImportError, ValueError):
    import sys
    from pathlib import Path
    this_file = Path(__file__).resolve()
    project_root = this_file.parents[3]
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))
    from hex_zmq_servers.robot.robot_base import HexRobotServerBase
    from hex_zmq_servers.robot.hello.robot_hello import HexRobotHello

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
    "sens_ts": True,
}


class HexRobotHelloServer(HexRobotServerBase):

    def __init__(
        self,
        net_config: dict = NET_CONFIG,
        params_config: dict = ROBOT_CONFIG,
    ):
        HexRobotServerBase.__init__(self, net_config)
        self._rgbs_queue = deque(maxlen=1)
        self._rgbs_seq = -1

        # robot
        self._device = HexRobotHello(params_config,
                                     net_config.get("realtime_mode", False))

    def work_loop(self):
        try:
            self._device.work_loop([
                self._states_queue,
                self._rgbs_queue,
                self._stop_event,
            ])
        finally:
            self._device.close()

    def _process_request(self, recv_hdr: dict, recv_buf: np.ndarray):
        if recv_hdr["cmd"] == "is_working":
            return self.no_ts_hdr(recv_hdr, self._device.is_working()), None
        elif recv_hdr["cmd"] == "seq_clear":
            return self.no_ts_hdr(recv_hdr, self._seq_clear()), None
        elif recv_hdr["cmd"] == "get_dofs":
            dofs = self._device.get_dofs()
            return self.no_ts_hdr(recv_hdr, dofs is not None), dofs
        elif recv_hdr["cmd"] == "get_limits":
            limits = self._device.get_limits()
            return self.no_ts_hdr(recv_hdr, limits is not None), limits
        elif recv_hdr["cmd"] == "get_states":
            return self._get_states(recv_hdr)
        elif recv_hdr["cmd"] == "set_rgbs":
            return self.__set_rgbs(recv_hdr, recv_buf)
        else:
            raise ValueError(f"unknown command: {recv_hdr['cmd']}")

    def __set_rgbs(self, recv_hdr: dict, recv_buf: np.ndarray):
        seq = recv_hdr.get("args", None)
        if self._seq_clear_flag:
            self._seq_clear_flag = False
            self._rgbs_seq = -1
            return self.no_ts_hdr(recv_hdr, False), None

        if seq is not None:
            delta = (seq - self._rgbs_seq) % self._max_seq_num
            if delta >= 0 and delta < 1e6:
                self._rgbs_seq = seq
                self._rgbs_queue.append((recv_hdr["ts"], seq, recv_buf))
                return self.no_ts_hdr(recv_hdr, True), None
            else:
                return self.no_ts_hdr(recv_hdr, False), None
        else:
            return self.no_ts_hdr(recv_hdr, False), None


if __name__ == "__main__":
    import argparse, json
    from hex_zmq_servers.zmq_base import hex_server_helper

    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    hex_server_helper(cfg, HexRobotHelloServer)
