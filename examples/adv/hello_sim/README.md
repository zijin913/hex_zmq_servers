# Hello Sim Teleoperation Example

## Description

This example shows how to use Hello robot to control Mujoco Archer Y6 simulation in real-time. It demonstrates teleoperation where Hello serves as the master device and the simulated robot follows its movements.

## Structure

```bash
hello_sim/
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

    Modify the `DEVICE_IP` and `HELLO_DEVICE_PORT` in `launch.py` to match your device.

    Assuming:
    - The controller ip is `172.18.5.116`
    - The Hello device is plugged into `CAN0`, which means the device port is `8439`

    ```python
    ...
    DEVICE_IP = "172.18.5.116"
    HELLO_DEVICE_PORT = 8439
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
    cd examples/adv/hello_sim
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```
