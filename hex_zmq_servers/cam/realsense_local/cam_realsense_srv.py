#!/usr/bin/env python3
# -*- coding:utf-8 -*-
################################################################
# RealSense Camera Server
################################################################

import sys
from pathlib import Path
# Add parent directory to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent.parent))

import numpy as np
from hex_zmq_servers.cam.cam_base import HexCamServerBase
from hex_zmq_servers.cam.realsense_local.cam_realsense import HexCamRealsense, CAMERA_CONFIG

NET_CONFIG = {
    "ip": "127.0.0.1",
    "port": 12346,  # Different port from robot
    "client_timeout_ms": 200,
    "server_timeout_ms": 1_000,
    "server_num_workers": 4,
}


class HexCamRealsenseServer(HexCamServerBase):
    """RealSense camera ZMQ server."""

    def __init__(
        self,
        net_config: dict = NET_CONFIG,
        camera_config: dict = CAMERA_CONFIG,
    ):
        HexCamServerBase.__init__(self, net_config)
        self._device = HexCamRealsense(camera_config)

    def _process_request(self, recv_hdr: dict, recv_buf: np.ndarray):
        """Process incoming requests."""
        try:
            cmd = recv_hdr["cmd"]
        except KeyError:
            print(f"\033[91mRequest missing 'cmd' field\033[0m")
            return {"cmd": "unknown_failed"}, None

        # Handle get_rgb
        if cmd == "get_rgb":
            return self._get_frame(recv_hdr, depth_flag=False)

        # Handle get_depth
        elif cmd == "get_depth":
            return self._get_frame(recv_hdr, depth_flag=True)

        # Handle get_intri
        elif cmd == "get_intri":
            try:
                intri = self._device.get_intri()
                return {
                    "cmd": "get_intri_ok",
                    "args": intri.tolist()
                }, intri
            except Exception as e:
                print(f"\033[91mget_intri failed: {e}\033[0m")
                return {"cmd": "get_intri_failed"}, None

        # Unknown command
        else:
            print(f"\033[91mUnknown command: {cmd}\033[0m")
            return {"cmd": f"{cmd}_failed"}, None


if __name__ == "__main__":
    import sys
    import json
    sys.path.insert(0, "/home/ubuntu/soda/hex_zmq_servers")

    from hex_zmq_servers.zmq_base import hex_server_helper

    # Parse config from command line
    if len(sys.argv) > 1 and sys.argv[1] == '--cfg':
        cfg = json.loads(sys.argv[2])
    else:
        # Default config
        cfg = {
            "net": NET_CONFIG,
            "params": CAMERA_CONFIG,
        }

    hex_server_helper(cfg, HexCamRealsenseServer)
