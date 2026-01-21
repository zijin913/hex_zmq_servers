# Force Feedback Teleoperation Example

## Description

This example demonstrates bilateral teleoperation with force feedback. The master HexArm can feel the forces encountered by the slave HexArm, providing realistic haptic feedback for improved operation precision and safety.

## Structure

```bash
force_feedback/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- HexArm robot

## Usage

1. **Important⚠️** Modify the device parameters in `launch.py`

    Modify the `ARM_TYPE`, `GRIPPER_TYPE`, `MASTER_DEVICE_IP`, `SLAVE_DEVICE_IP`, `MASTER_DEVICE_PORT` and `SLAVE_DEVICE_PORT` in `launch.py` to match your devices. `CAN0` => `8439`, `CAN1` => `9439`.

    Assuming:
    - The arm is `archer_y6` with `gp100` gripper
    - The controller IP is `172.18.5.116`
    - The master arm is plugged into `CAN0`, which means the device port is `8439`
    - The slave arm is plugged into `CAN1`, which means the device port is `9439`

    ```python
    ...
    ARM_TYPE = "archer_y6"
    GRIPPER_TYPE = "gp100"
    MASTER_DEVICE_IP = "172.18.5.116"
    SLAVE_DEVICE_IP = "172.18.5.116"
    MASTER_DEVICE_PORT = 8439
    SLAVE_DEVICE_PORT = 9439
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
    cd examples/adv/force_feedback
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```

## Safety Notice

- ⚠️ Make sure there is enough safe space around the robot and you can cut off power at any time.
