<h1 align="center">TUTORIAL</h1>

<p align="center">
    <a href="tutorial.md">
        <img src="https://img.shields.io/badge/EN-active-2ea44f?style=flat-square&logo=googletranslate" />
    </a>
</p>


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