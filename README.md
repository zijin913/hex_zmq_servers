<h1 align="center">HEXFELLOW ZMQ SERVERS</h1>

<p align="center">
    <a href="https://github.com/hexfellow/hex_zmq_servers/issues">
        <img src="https://img.shields.io/github/issues/hexfellow/hex_zmq_servers?style=flat-square&logo=github" />
    </a>
    <a href="https://github.com/hexfellow/hex_zmq_servers/stargazers">
        <img src="https://img.shields.io/github/stars/hexfellow/hex_zmq_servers?style=flat-square&logo=github" />
    </a>
    <a href="https://github.com/hexfellow/hex_zmq_servers/forks">
        <img src="https://img.shields.io/github/forks/hexfellow/hex_zmq_servers?style=flat-square&logo=github" />
    </a>
    <a href="https://doi.org/10.5281/zenodo.18309954">
        <img src="https://zenodo.org/badge/1088506315.svg" alt="DOI">
    </a>
    &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;
    <a href="README_CN.md">
        <img src="https://img.shields.io/badge/中文-active-2ea44f?style=flat-square&logo=googletranslate" />
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

- Engineers integrating HEXFELLOW robots into their systems.
- Researchers running experiments with HEXFELLOW robots.

---

# 📦 Installation

## Requirements

- **Python**
- **OS**: `Linux` / `macOS`
- **Core dependencies**:
  - `pyzmq`
  - `hex_device`
  - `hex_robo_utils`
  - `opencv-python`

Optional device support (install via extras):

| Extra       | Purpose                                         |
| ----------- | ----------------------------------------------- |
| `berxel`    | Berxel RGB-D: `berxel_py_wrapper`               |
| `realsense` | RealSense RGB-D: `pyrealsense2`                 |
| `dynamixel` | Dynamixel: `dynamixel-sdk`                      |
| `mujoco`    | MuJoCo sims: `mujoco`                           |
| `all`       | `berxel` + `dynamixel` + `realsense` + `mujoco` |

## Install from PyPI

For those who don't need examples, you can install the package from PyPI.

- **Full install**: includes all optional devices (Berxel, RealSense, Dynamixel, MuJoCo)

    ```bash
    pip install hex_zmq_servers[all]
    ```

- **Core install**: only the core package (no optional devices)

    ```bash
    pip install hex_zmq_servers
    ```

## Install from Source

For those who need examples, you can install the package from source code with examples.

**Noet**: We use [**uv**](https://github.com/astral-sh/uv) to manage the Python environment. Please install it first.

1. Clone and install in editable mode. The `venv.sh` script expects [uv](https://github.com/astral-sh/uv).

    ```bash
    git clone https://github.com/hexfellow/hex_zmq_servers.git
    cd hex_zmq_servers
    ./venv.sh
    ```

   - `./venv.sh` — creates `.venv`, installs `hex_zmq_servers` with `[all]` and `examples/adv/requirements.txt` (e.g. `pygame` for some examples).
   - `./venv.sh --min` — installs the core package only (no optional device extras). Some examples will not run.
   - `./venv.sh --pkg-only` — installs the package only, skips example-related dependencies.

2. Activate before running examples:

    ```bash
    source .venv/bin/activate
    ```

---

# 📚 Tutorial

See [**Tutorial**](docs/tutorial.md) for details of all tutorials.

# 📑 API

See [**API**](docs/api.md) for details of all APIs.

# 💡 Example

See [**Example**](docs/example.md) for details of all examples.

---

# 🏷️ Citation

If you want to cite this project in your work, you can use the following BibTeX entry:

```bibtex
@software{hex_zmq_servers,
  author    = {Dong, Zhaorui},
  title     = {Hex ZMQ Servers: A ZeroMQ-Based Embodied AI Communication Framework},
  year      = {2025},
  publisher = {Zenodo},
  version   = {v1.0.0},
  doi       = {10.5281/zenodo.18309960},
  url       = {https://doi.org/10.5281/zenodo.18309960}
}
```

---

# 📄 License

Apache License 2.0. See [LICENSE](LICENSE).

---

# 🌟 Star History

[![Star History Chart](https://api.star-history.com/svg?repos=hexfellow/hex_zmq_servers&type=Date)](https://star-history.com/#hexfellow/hex_zmq_servers&Date)

---
