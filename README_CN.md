<h1 align="center">HEXFELLOW ZMQ SERVERS</h1>

<p align="center">
    <a href="https://github.com/hexfellow/hex_zmq_servers/stargazers">
        <img src="https://img.shields.io/github/stars/hexfellow/hex_zmq_servers?style=flat-square&logo=github" />
    </a>
    <a href="https://github.com/hexfellow/hex_zmq_servers/issues">
        <img src="https://img.shields.io/github/issues/hexfellow/hex_zmq_servers?style=flat-square&logo=github" />
    </a>
    <a href="https://github.com/hexfellow/hex_zmq_servers/contributors">
        <img src="https://img.shields.io/github/contributors/hexfellow/hex_zmq_servers?style=flat-square&logo=github" />
    </a>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
    <a href="README.md">
        <img src="https://img.shields.io/badge/EN-active-2ea44f?style=flat-square&logo=googletranslate" />
    </a>
</p>

---

# 📖 Overview

## What is `hex_zmq_servers`

`hex_zmq_servers` provides a client–server layer on top of ZeroMQ to control and stream data from HEXFELLOW hardware (robots, RGB/RGB-D cameras) and MuJoCo-based simulators. Servers run device logic and command loops; clients send requests (e.g. `get_rgb`, `get_state`, `set_target`) and receive headers plus optional binary buffers (e.g. images, joint state).

## What problem it solves

- **Decoupled control**: Run device drivers and control loops in separate processes; clients connect over TCP.
- **Unified transport**: All devices use the same ZMQ request/response pattern (JSON header + NumPy buffer).
- **Multi-node management**: `HexLaunch` and `HexNodeConfig` start and monitor multiple server/client nodes from one process.

## Target users

- Engineers integrating HEXFELLOW robots and cameras into larger systems.
- Researchers running experiments with real hardware and/or MuJoCo.

---

# 📦 Installation

## 3.1 Requirements

- **Python**: >= 3.10 (3.10, 3.11, 3.12 supported).
- **OS**: Linux (POSIX); classifiers specify Linux.
- **Core dependencies** (from `pyproject.toml`):
  - `pyzmq>=27.0.1`
  - `hex_device>=1.3.1,<1.4.0`
  - `hex_robo_utils>=0.2.0,<0.3.0`
  - `dynamixel-sdk==3.8.4`
  - `opencv-python>=4.2`

Optional device support (install via extras):

| Extra       | Purpose                                      |
| ----------- | -------------------------------------------- |
| `berxel`    | Berxel RGB-D: `berxel_py_wrapper>=2.0.182`   |
| `realsense` | RealSense RGB-D: `pyrealsense2>=2.56.5.9235` |
| `mujoco`    | MuJoCo sims: `mujoco>=3.3.3`                 |
| `all`       | `berxel` + `realsense` + `mujoco`            |

---

## 3.2 Install from Package Manager (PyPI)

```bash
# Full install including optional devices (Berxel, RealSense, MuJoCo)
pip install hex_zmq_servers[all]
```

```bash
# Core only (no Berxel, RealSense, MuJoCo)
pip install hex_zmq_servers
```

---

## 3.3 Install from Source

Clone and install in editable mode. The `venv.sh` script expects [uv](https://github.com/astral-sh/uv).

```bash
git clone https://github.com/hexfellow/hex_zmq_servers.git
cd hex_zmq_servers
./venv.sh
```

- `./venv.sh` — creates `.venv`, installs `hex_zmq_servers` with `[all]` and `examples/adv/requirements.txt` (e.g. `pygame` for some examples).
- `./venv.sh --min` — installs the core package only (no optional device extras). Some examples will not run.
- `./venv.sh --pkg-only` — installs the package only, skips example-related dependencies.

Activate before running examples:

```bash
source .venv/bin/activate
```

---

# 📚 Tutorials

## 4.1 Basic Tutorial

**Minimal working example: ZMQ dummy server + client.**

1. **Start the dummy server** (one terminal). Config must include `"net"` and `"params"`:

```bash
python -m hex_zmq_servers.zmq_base --cfg '{"net":{"ip":"127.0.0.1","port":12345,"realtime_mode":false,"deque_maxlen":10,"client_timeout_ms":200,"server_timeout_ms":1000,"server_num_workers":4},"params":{}}'
```

2. **Run the dummy client** (another terminal; requires a source checkout, since examples are not in the PyPI package):

```bash
python examples/basic/zmq_dummy/cli.py --cfg '{"net":{"ip":"127.0.0.1","port":12345}}'
```

The client calls `HexZMQDummyClient.single_test()` in a loop; the server replies to `"test"` with `"test_ok"`.

**Core usage pattern**: build a client with `net_config` (at least `ip`, `port`), call `request({"cmd": "..."}, req_buf)` or device-specific methods (e.g. `get_rgb`, `get_state`). Servers are started via `hex_server_helper(cfg, ServerClass)` or by running the `*_srv.py` script with `--cfg`.

---

## 4.2 Advanced Tutorial

**Launching server + client with `HexLaunch` and `HexNodeConfig`.**

1. **Define node params** (including `name`, `node_path`, `cfg_path`, `cfg`).
2. **Implement `get_node_cfgs`** so it returns a `HexNodeConfig` (e.g. via `HexNodeConfig.parse_node_params_dict`).
3. **Run `HexLaunch(node_cfgs).run()`** to start all nodes; it builds `--cfg` from `cfg_path` + `cfg` and spawns subprocesses.

Example layout (as in `examples/basic/zmq_dummy/launch.py`):

```python
import os
from hex_zmq_servers import HexLaunch, HexNodeConfig
from hex_zmq_servers import HEX_ZMQ_SERVERS_PATH_DICT, HEX_ZMQ_CONFIGS_PATH_DICT

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
HEX_ZMQ_SERVERS_DIR = os.path.join(SCRIPT_DIR, "..", "..", "..", "hex_zmq_servers")

NODE_PARAMS_DICT = {
    "zmq_dummy_cli": {
        "name": "zmq_dummy_cli",
        "node_path": os.path.join(HEX_ZMQ_SERVERS_DIR, "..", "examples", "basic", "zmq_dummy", "cli.py"),
        "cfg_path": os.path.join(HEX_ZMQ_SERVERS_DIR, "..", "examples", "basic", "zmq_dummy", "cli.json"),
        "cfg": {"net": {"ip": "127.0.0.1", "port": 12345}},
    },
    "zmq_dummy_srv": {
        "name": "zmq_dummy_srv",
        "node_path": HEX_ZMQ_SERVERS_PATH_DICT["zmq_dummy"],
        "cfg_path": HEX_ZMQ_CONFIGS_PATH_DICT["zmq_dummy"],
        "cfg": {"net": {"ip": "127.0.0.1", "port": 12345}},
    },
}

def get_node_cfgs(node_params_dict=NODE_PARAMS_DICT, launch_arg=None):
    return HexNodeConfig.parse_node_params_dict(node_params_dict, NODE_PARAMS_DICT)

def main():
    launch = HexLaunch(get_node_cfgs())
    launch.run()

if __name__ == "__main__":
    main()
```

**Customization / extension**: Implement new servers by subclassing `HexZMQServerBase`, defining `work_loop` and `_process_request`, and running them with `hex_server_helper(cfg, YourServerClass)`. New clients subclass `HexZMQClientBase` and use `request(req_dict, req_buf)`.

---

## 4.3 Examples

| Type         | Path                                                              | Description                                          |
| ------------ | ----------------------------------------------------------------- | ---------------------------------------------------- |
| **Basic**    | `examples/basic/zmq_dummy/`                                       | ZMQ request/response only; no real device.           |
| **Basic**    | `examples/basic/cam_rgb/`                                         | RGB camera server + client (`get_intri`, `get_rgb`). |
| **Basic**    | `examples/basic/robot_dummy/`, `robot_gello/`, `robot_hexarm/`    | Robot control (dummy, Gello, HexArm).                |
| **Basic**    | `examples/basic/cam_berxel/`, `cam_realsense/`                    | RGB-D (requires `berxel` / `realsense`).             |
| **Basic**    | `examples/basic/mujoco_archer_y6/`, `mujoco_e3_desktop/`          | MuJoCo (requires `mujoco`).                          |
| **Advanced** | `examples/adv/traj_circle/`                                       | Circle trajectory on HexArm (client + launch).       |
| **Advanced** | `examples/adv/gello_sim/`, `gello_real/`, `joy_sim/`, `joy_real/` | Teleoperation.                                       |
| **Advanced** | `examples/adv/multi_launch/`, `multi_launch_mujoco/`, etc.        | Multi-node launch only.                              |

- **Basic**: Single-device; run `launch.py` or start server + `cli.py` manually.
- **Advanced**: Multi-device or teleoperation; some need `examples/adv/requirements.txt` (e.g. `pygame`).

See [examples/README.md](examples/README.md) for the full list and per-example READMEs.

---

# 📑 API List

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

# 💡 Example List

| Name                          | Purpose                    | Path                                                                                           |
| ----------------------------- | -------------------------- | ---------------------------------------------------------------------------------------------- |
| `zmq_dummy`                   | ZMQ communication test     | [examples/basic/zmq_dummy](examples/basic/zmq_dummy/README.md)                                 |
| `robot_dummy`                 | Dummy robot                | [examples/basic/robot_dummy](examples/basic/robot_dummy/README.md)                             |
| `robot_gello`                 | Gello robot                | [examples/basic/robot_gello](examples/basic/robot_gello/README.md)                             |
| `robot_hexarm`                | HexArm robot               | [examples/basic/robot_hexarm](examples/basic/robot_hexarm/README.md)                           |
| `cam_dummy`                   | Dummy camera               | [examples/basic/cam_dummy](examples/basic/cam_dummy/README.md)                                 |
| `cam_rgb`                     | RGB camera                 | [examples/basic/cam_rgb](examples/basic/cam_rgb/README.md)                                     |
| `cam_berxel`                  | Berxel RGB-D               | [examples/basic/cam_berxel](examples/basic/cam_berxel/README.md)                               |
| `cam_realsense`               | RealSense RGB-D            | [examples/basic/cam_realsense](examples/basic/cam_realsense/README.md)                         |
| `mujoco_archer_y6`            | MuJoCo Archer Y6           | [examples/basic/mujoco_archer_y6](examples/basic/mujoco_archer_y6/README.md)                   |
| `mujoco_e3_desktop`           | MuJoCo E3 Desktop          | [examples/basic/mujoco_e3_desktop](examples/basic/mujoco_e3_desktop/README.md)                 |
| `double_gello_sim`            | Gello + MuJoCo teleop      | [examples/adv/double_gello_sim](examples/adv/double_gello_sim/README.md)                       |
| `double_gello_real`           | Gello + E3-Desktop teleop  | [examples/adv/double_gello_real](examples/adv/double_gello_real/README.md)                     |
| `gello_sim`                   | Gello + MuJoCo teleop      | [examples/adv/gello_sim](examples/adv/gello_sim/README.md)                                     |
| `gello_real`                  | Gello + HexArm teleop      | [examples/adv/gello_real](examples/adv/gello_real/README.md)                                   |
| `joy_sim`                     | Joystick + MuJoCo          | [examples/adv/joy_sim](examples/adv/joy_sim/README.md)                                         |
| `joy_real`                    | Joystick + HexArm          | [examples/adv/joy_real](examples/adv/joy_real/README.md)                                       |
| `force_feedback`              | Force feedback teleop      | [examples/adv/force_feedback](examples/adv/force_feedback/README.md)                           |
| `zero_gravity`                | Zero-gravity (torque comp) | [examples/adv/zero_gravity](examples/adv/zero_gravity/README.md)                               |
| `traj_axis`                   | Trajectory along axis      | [examples/adv/traj_axis](examples/adv/traj_axis/README.md)                                     |
| `traj_circle`                 | Trajectory circle          | [examples/adv/traj_circle](examples/adv/traj_circle/README.md)                                 |
| `traj_point`                  | Trajectory point           | [examples/adv/traj_point](examples/adv/traj_point/README.md)                                   |
| `multi_berxel`                | Multiple Berxel            | [examples/adv/multi_berxel](examples/adv/multi_berxel/README.md)                               |
| `multi_realsense`             | Multiple RealSense         | [examples/adv/multi_realsense](examples/adv/multi_realsense/README.md)                         |
| `multi_launch`                | Multi-node launch          | [examples/adv/multi_launch](examples/adv/multi_launch/README.md)                               |
| `multi_launch_mujoco`         | Multi-node MuJoCo          | [examples/adv/multi_launch_mujoco](examples/adv/multi_launch_mujoco/README.md)                 |
| `multi_launch_force_feedback` | Multi-node force feedback  | [examples/adv/multi_launch_force_feedback](examples/adv/multi_launch_force_feedback/README.md) |
| `multi_launch_zero_gravity`   | Multi-node zero-gravity    | [examples/adv/multi_launch_zero_gravity](examples/adv/multi_launch_zero_gravity/README.md)     |
| `multi_launch_berxel`         | Multi-node Berxel          | [examples/adv/multi_launch_berxel](examples/adv/multi_launch_berxel/README.md)                 |
| `multi_launch_realsense`      | Multi-node RealSense       | [examples/adv/multi_launch_realsense](examples/adv/multi_launch_realsense/README.md)           |

---

# 📄 License

Apache License 2.0. See [LICENSE](LICENSE).

---

# 👥 Contributors

- **Author**: [Dong Zhaorui](https://github.com/IBNBlank)
- **Maintainer**: [jecjune](https://github.com/Jecjune)

Repository: [hexfellow/hex_zmq_servers](https://github.com/hexfellow/hex_zmq_servers) · Issues: [hex_zmq_servers issues](https://github.com/hexfellow/hex_zmq_servers/issues) · Wiki: [hex_zmq_servers wiki](https://github.com/hexfellow/hex_zmq_servers/wiki)
