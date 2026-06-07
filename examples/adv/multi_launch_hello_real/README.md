# Multi Launch Hello Real Example

## Description

This example demonstrates how to use multiple Hello + HexArm real robots using multi-launch. It starts two `hello_real` teleoperation pipelines at the same time, showing advanced usage of dynamically loading node configurations from another launch file and merging them together.

## Structure

```bash
multi_launch_hello_real/
├── launch.py  # multi-launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- Hello robot devices (2 sets)
- HexArm robots (2 arms)

## Usage

1. **Important⚠️** Modify the device parameters in `launch.py`

    Modify the following parameters in `launch.py` to match your devices. `CAN0` => `8439`, `CAN1` => `9439`.

    - `ARM_TYPE`, `GRIPPER_TYPE`
    - `HELLO_REAL_0_HELLO_DEVICE_IP`, `HELLO_REAL_0_HELLO_DEVICE_PORT`
    - `HELLO_REAL_0_HEXARM_DEVICE_IP`, `HELLO_REAL_0_HEXARM_DEVICE_PORT`
    - `HELLO_REAL_1_HELLO_DEVICE_IP`, `HELLO_REAL_1_HELLO_DEVICE_PORT`
    - `HELLO_REAL_1_HEXARM_DEVICE_IP`, `HELLO_REAL_1_HEXARM_DEVICE_PORT`

    Assuming:

    - The arm type is `archer_d6y` with `gp80` gripper
    - The Hello devices are connected to controller ip `172.18.13.174`
    - The HexArm controllers are at ip `172.18.16.228`
    - The first Hello + HexArm pair is plugged into `CAN0`, which means the device port is `8439`
    - The second Hello + HexArm pair is plugged into `CAN1`, which means the device port is `9439`

    ```python
    ...
    ARM_TYPE = "archer_d6y"
    GRIPPER_TYPE = "gp80"

    # hello_real 0
    HELLO_REAL_0_HELLO_DEVICE_IP = "172.18.13.174"
    HELLO_REAL_0_HELLO_DEVICE_PORT = 8439
    HELLO_REAL_0_HEXARM_DEVICE_IP = "172.18.16.228"
    HELLO_REAL_0_HEXARM_DEVICE_PORT = 8439

    # hello_real 1
    HELLO_REAL_1_HELLO_DEVICE_IP = "172.18.13.174"
    HELLO_REAL_1_HELLO_DEVICE_PORT = 9439
    HELLO_REAL_1_HEXARM_DEVICE_IP = "172.18.16.228"
    HELLO_REAL_1_HEXARM_DEVICE_PORT = 9439
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
    cd examples/adv/multi_launch_hello_real
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```

    **Note**: The Hello’s LED indicates the robot’s status.
    - **Yellow：** The robots are initializing. If the LED stays yellow for more than 20 seconds, please lift the Hello slightly and hold it still.
    - **Green：** The robots are operating normally.
    - **Red：** The robots are not connected.

## Safety Notice

- ⚠️ Make sure there is enough safe space around all robots and you can cut off power at any time.
