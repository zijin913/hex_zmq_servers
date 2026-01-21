# ZMQ Dummy Example

## Description

This example shows the basic ZeroMQ communication mechanism of the framework. It demonstrates the request-response pattern between client and server without any specific device.

## Structure

```bash
zmq_dummy/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

None.

### Environment

None.

## Usage

1. Activate the virtual environment

    ```bash
    cd path/to/hex_zmq_servers
    source .venv/bin/activate
    ```

2. Run the launch script

    Run the launch script:

    ```bash
    cd examples/basic/zmq_dummy
    python launch.py
    ```

    The output should be like this:

    ```bash
    Using system clock
    [launcher] Terminal settings recorded
    [launcher] Started 2 nodes
    [launcher] Starting zmq_dummy_cli
    [launcher] Starting zmq_dummy_srv
    [zmq_dummy_srv] Using system clock
    [zmq_dummy_cli] Using system clock
    [zmq_dummy_srv] test received
    [zmq_dummy_srv] recv_hdr: {'cmd': 'test', 'ts': {'s': 4297, 'ns': 581289423}, 'args': None, 'dtype': 'uint8', 'shape': [0]}
    [zmq_dummy_srv] recv_buf: []
    [zmq_dummy_srv] test received
    [zmq_dummy_srv] recv_hdr: {'cmd': 'test', 'ts': {'s': 4297, 'ns': 681398189}, 'args': None, 'dtype': 'uint8', 'shape': [0]}
    [zmq_dummy_srv] recv_buf: []
    [zmq_dummy_srv] test received
    [zmq_dummy_srv] recv_hdr: {'cmd': 'test', 'ts': {'s': 4297, 'ns': 781394920}, 'args': None, 'dtype': 'uint8', 'shape': [0]}
    [zmq_dummy_srv] recv_buf: []
    [zmq_dummy_srv] test received
    [zmq_dummy_srv] recv_hdr: {'cmd': 'test', 'ts': {'s': 4297, 'ns': 881398640}, 'args': None, 'dtype': 'uint8', 'shape': [0]}
    [zmq_dummy_srv] recv_buf: []
    [zmq_dummy_srv] test received
    [zmq_dummy_srv] recv_hdr: {'cmd': 'test', 'ts': {'s': 4297, 'ns': 981410178}, 'args': None, 'dtype': 'uint8', 'shape': [0]}
    [zmq_dummy_srv] recv_buf: []
    [zmq_dummy_srv] test received
    [zmq_dummy_srv] recv_hdr: {'cmd': 'test', 'ts': {'s': 4298, 'ns': 81405052}, 'args': None, 'dtype': 'uint8', 'shape': [0]}
    [zmq_dummy_srv] recv_buf: []
    [zmq_dummy_srv] test received
    [zmq_dummy_srv] recv_hdr: {'cmd': 'test', 'ts': {'s': 4298, 'ns': 181396538}, 'args': None, 'dtype': 'uint8', 'shape': [0]}
    [zmq_dummy_srv] recv_buf: []
    ...
    ```
