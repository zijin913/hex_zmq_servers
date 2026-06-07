# Hello Real Teleoperation Example

## Description

This example shows how to use Hello robot to control a real HexArm robot in real-time. It demonstrates bilateral teleoperation where Hello serves as the master device and HexArm follows its movements.

## Structure

```bash
hello_real/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- Hello robot device
- HexArm robot

## Usage

1. **Important⚠️** Modify the device parameters in `launch.py`

    Modify the `ARM_TYPE`, `GRIPPER_TYPE`, `DEVICE_IP`, `HELLO_DEVICE_PORT` and `HEXARM_DEVICE_PORT` in `launch.py` to match your devices. `CAN0` => `8439`, `CAN1` => `9439`.

    Assuming:
    - The arm is `archer_l6y` with `gp100` gripper
    - The controller ip is `172.18.5.116`
    - The Hello device is plugged into `CAN0`, which means the device port is `8439`
    - The HexArm is plugged into `CAN1`, which means the device port is `9439`

    ```python
    ...
    ARM_TYPE = "archer_l6y"
    GRIPPER_TYPE = "gp100"
    DEVICE_IP = "172.18.5.116"
    HELLO_DEVICE_PORT = 8439
    HEXARM_DEVICE_PORT = 9439
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
    cd examples/adv/hello_real
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

- ⚠️ Make sure there is enough safe space around the robot and you can cut off power at any time.
