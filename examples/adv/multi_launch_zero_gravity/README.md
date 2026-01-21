# Zero Gravity Example

## Description

This example demonstrates how to use multiple HexArm robots using multi-launch. It allows for simultaneous zero gravity mode of multiple HexArm robots.

## Structure

```bash
multi_launch_zero_gravity/
├── launch.py  # multi-launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- HexArm robot

## Usage

1. **Important⚠️** Modify the device parameters in `launch.py`

    Modify the `ARM_TYPE`, `GRIPPER_TYPE`, `ZERO_GRAVITY_0_DEVICE_IP`, `ZERO_GRAVITY_0_DEVICE_PORT`, `ZERO_GRAVITY_1_DEVICE_IP` and `ZERO_GRAVITY_1_DEVICE_PORT` in `launch.py` to match your devices. `CAN0` => `8439`, `CAN1` => `9439`.

    Assuming:
    - The arm is `archer_y6` with `gp100` gripper
    - The controller ip is `172.18.18.92`
    - The first zero gravity arm is plugged into `CAN0`, which means the device port is `8439`
    - The second zero gravity arm is plugged into `CAN1`, which means the device port is `9439`

    ```python
    ...
    ARM_TYPE = "archer_y6"
    GRIPPER_TYPE = "gp100"
    ZERO_GRAVITY_0_DEVICE_IP = "172.18.18.92"
    ZERO_GRAVITY_0_DEVICE_PORT = 8439
    ZERO_GRAVITY_1_DEVICE_IP = "172.18.18.92"
    ZERO_GRAVITY_1_DEVICE_PORT = 9439
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
    cd examples/adv/multi_launch_zero_gravity
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```

## Safety Notice

- ⚠️ Make sure there is enough safe space around the robot and you can cut off power at any time.
