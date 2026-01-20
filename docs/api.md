<h1 align="center">API</h1>

<p align="center">
    <a href="api.md">
        <img src="https://img.shields.io/badge/中文-active-2ea44f?style=flat-square&logo=googletranslate" />
    </a>
</p>

Public APIs are exposed from the top-level package `hex_zmq_servers`. Optional device classes are available only when the matching extra is installed.

| Module / file                | Class or function                                                            | Description                                                                                                                             |
| ---------------------------- | ---------------------------------------------------------------------------- | --------------------------------------------------------------------------------------------------------------------------------------- |
| `hex_launch`                 | `HexLaunch`                                                                  | Launches and monitors multiple nodes (subprocesses); builds `--cfg` from `cfg_path` and `cfg`.                                          |
| `hex_launch`                 | `HexNodeConfig`                                                              | Holds and parses node configs; `parse_node_params_dict`, `get_node_cfgs_from_launch`, `get_launch_params_cfgs`, `get_cfgs`, `add_cfgs`. |
| `hex_launch`                 | `HEX_LOG_LEVEL`                                                              | Dict: `"info"`=0, `"warn"`=1, `"err"`=2.                                                                                                |
| `hex_launch`                 | `hex_log(level, message)`                                                    | Log by level.                                                                                                                           |
| `hex_launch`                 | `hex_err(message)`                                                           | Print to stderr.                                                                                                                        |
| `hex_launch`                 | `hex_dict_str(dict_raw, indent=0)`                                           | Pretty-print dict.                                                                                                                      |
| `device_base`                | `HexDeviceBase`                                                              | Abstract device; `work_loop`, `close`, `is_working`.                                                                                    |
| `zmq_base`                   | `hex_zmq_ts_to_ns(ts)`                                                       | Convert `{s, ns}` to nanoseconds.                                                                                                       |
| `zmq_base`                   | `ns_to_hex_zmq_ts(ns)`                                                       | Convert nanoseconds to `{s, ns}`.                                                                                                       |
| `zmq_base`                   | `hex_ns_now()`                                                               | Current time in ns (PTP if `HEX_PTP_CLOCK` set).                                                                                        |
| `zmq_base`                   | `hex_zmq_ts_now()`                                                           | `{s, ns}` for now.                                                                                                                      |
| `zmq_base`                   | `hex_zmq_ts_delta_ms(curr_ts, hdr_ts)`                                       | Delta in ms.                                                                                                                            |
| `zmq_base`                   | `HexRate(hz, spin_threshold_ns=10_000)`                                      | Rate limiter; `sleep()`, `reset()`.                                                                                                     |
| `zmq_base`                   | `HexZMQClientBase`                                                           | Abstract ZMQ client; `request(req_dict, req_buf)`, `is_working`, `close`.                                                               |
| `zmq_base`                   | `HexZMQServerBase`                                                           | Abstract ZMQ server; `start()`, `work_loop()`, `_process_request`, `close`, `no_ts_hdr`.                                                |
| `zmq_base`                   | `hex_server_helper(cfg, server_cls)`                                         | Runs `server_cls(net, params)` with `cfg["net"]`, `cfg["params"]`; `start()`, `work_loop()`, signal handling.                           |
| `zmq_base`                   | `HexZMQDummyClient`                                                          | Dummy client; `single_test()`.                                                                                                          |
| `zmq_base`                   | `HexZMQDummyServer`                                                          | Dummy server; handles `"test"`.                                                                                                         |
| `robot`                      | `HexRobotBase`, `HexRobotClientBase`, `HexRobotServerBase`                   | Robot base abstractions.                                                                                                                |
| `robot`                      | `HexRobotDummy`, `HexRobotDummyClient`, `HexRobotDummyServer`                | Dummy robot.                                                                                                                            |
| `robot`                      | `HexRobotGello`, `HexRobotGelloClient`, `HexRobotGelloServer`                | Gello (Dynamixel).                                                                                                                      |
| `robot`                      | `HexRobotHexarm`, `HexRobotHexarmClient`, `HexRobotHexarmServer`             | HexArm.                                                                                                                                 |
| `robot`                      | `HEXARM_URDF_PATH_DICT`                                                      | Maps arm+gripper to URDF path.                                                                                                          |
| `cam`                        | `HexCamBase`, `HexCamClientBase`, `HexCamServerBase`                         | Camera base abstractions.                                                                                                               |
| `cam`                        | `HexCamDummy`, `HexCamDummyClient`, `HexCamDummyServer`                      | Dummy camera.                                                                                                                           |
| `cam`                        | `HexCamRGB`, `HexCamRGBClient`, `HexCamRGBServer`                            | RGB (V4L2).                                                                                                                             |
| `cam` (optional `berxel`)    | `HexCamBerxel`, `HexCamBerxelClient`, `HexCamBerxelServer`                   | Berxel RGB-D.                                                                                                                           |
| `cam` (optional `realsense`) | `HexCamRealsense`, `HexCamRealsenseClient`, `HexCamRealsenseServer`          | RealSense RGB-D.                                                                                                                        |
| `mujoco` (optional `mujoco`) | `HexMujocoBase`, `HexMujocoClientBase`, `HexMujocoServerBase`                | MuJoCo base.                                                                                                                            |
| `mujoco` (optional `mujoco`) | `HexMujocoArcherY6`, `HexMujocoArcherY6Client`, `HexMujocoArcherY6Server`    | Archer Y6 sim.                                                                                                                          |
| `mujoco` (optional `mujoco`) | `HexMujocoE3Desktop`, `HexMujocoE3DesktopClient`, `HexMujocoE3DesktopServer` | E3 Desktop sim.                                                                                                                         |
| `hex_zmq_servers` (package)  | `HEX_ZMQ_SERVERS_PATH_DICT`                                                  | Maps device key (e.g. `"cam_rgb"`) to server script path.                                                                               |
| `hex_zmq_servers` (package)  | `HEX_ZMQ_CONFIGS_PATH_DICT`                                                  | Maps device key to default config JSON path.                                                                                            |

---