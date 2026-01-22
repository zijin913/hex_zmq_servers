# Robot Hello Example

## Description

This example shows how to use Hello robot device. It reads the joint positions from Hello robot and provides position and velocity.

## Structure

```bash
robot_hello/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- Hello robot device

## Usage

1. **Important⚠️** Modify the device parameters in `launch.py`

    Modify the `DEVICE_IP` and `HEXARM_DEVICE_PORT` in `launch.py` to match your device. `CAN0` => `8439`, `CAN1` => `9439`.

    Assuming:
    - The controller ip is `172.18.5.116`
    - The Hello arm is plugged into `CAN0`, which means the device port is `8439`

    ```python
    ...
    DEVICE_IP = "172.18.5.116"
    HEXARM_DEVICE_PORT = 8439
    ...
    ```

2. Activate the virtual environment

    ```bash
    cd path/to/hex_zmq_servers
    source .venv/bin/activate
    ```

3. Run the launch script

    Run the launch script:

    ```bash
    cd examples/basic/robot_hello
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```
