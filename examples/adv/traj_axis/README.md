# Trajectory Axis Example

## Description

This example shows how to generate and execute a trajectory axis with the HexArm robot.

## Structure

```bash
traj_axis/
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

    Modify the `ARM_TYPE`, `GRIPPER_TYPE`, `DEVICE_IP` and `HEXARM_DEVICE_PORT` in `launch.py` to match your device. `CAN0` => `8439`, `CAN1` => `9439`.

    Assuming:
    - The arm is `archer_y6` with `gp100` gripper
    - The controller ip is `172.18.5.116`
    - The arm is plugged into `CAN0`, which means the device port is `8439`

    ```python
    ...
    ARM_TYPE = "archer_y6"
    GRIPPER_TYPE = "gp100"
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
    cd examples/adv/traj_axis
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```

## Safety Notice

- ⚠️ Make sure there is enough safe space around the robot and you can cut off power at any time.
