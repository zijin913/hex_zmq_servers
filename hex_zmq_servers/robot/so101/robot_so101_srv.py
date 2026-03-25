#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# SO-101 Leader Arm ZMQ Server
################################################################

import numpy as np

try:
    from ..robot_base import HexRobotServerBase
    from .robot_so101 import HexRobotSO101
except (ImportError, ValueError):
    import sys
    from pathlib import Path
    this_file = Path(__file__).resolve()
    project_root = this_file.parents[3]
    if str(project_root) not in sys.path:
        sys.path.insert(0, str(project_root))
    from hex_zmq_servers.robot.robot_base import HexRobotServerBase
    from hex_zmq_servers.robot.so101.robot_so101 import HexRobotSO101

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
    "idxs": [1, 2, 3, 4, 5, 6],
    "invs": [1.0, 1.0, 1.0, 1.0, 1.0, 1.0],
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


class HexRobotSO101Server(HexRobotServerBase):

    def __init__(
        self,
        net_config: dict = NET_CONFIG,
        params_config: dict = ROBOT_CONFIG,
    ):
        HexRobotServerBase.__init__(self, net_config)

        # robot
        self._device = HexRobotSO101(params_config,
                                      net_config.get("realtime_mode", False))

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
        elif recv_hdr["cmd"] == "set_cmds":
            return self._set_cmds(recv_hdr, recv_buf)
        else:
            raise ValueError(f"unknown command: {recv_hdr['cmd']}")


if __name__ == "__main__":
    import argparse, json
    from hex_zmq_servers.zmq_base import hex_server_helper

    parser = argparse.ArgumentParser()
    parser.add_argument("--cfg", type=str, required=True)
    args = parser.parse_args()
    cfg = json.loads(args.cfg)

    hex_server_helper(cfg, HexRobotSO101Server)
