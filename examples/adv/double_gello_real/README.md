# GELLO Real Teleoperation Example

## Description

This example shows how to use GELLO robot to control a real E3 Desktop robot in real-time. It demonstrates bilateral teleoperation where GELLO serves as the master device and E3 Desktop follows its movements.

## Structure

```bash
double_gello_real/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- GELLO robot device * 2
- E3 Desktop robot

### Environment

**For GELLO robot:**

1. Check USB connection:

    ```bash
    ls /dev/ttyUSB*
    ```

2. Set device permission (Assuming the device ports are `/dev/ttyUSB0` and `/dev/ttyUSB1`):

    ```bash
    sudo chmod 666 /dev/ttyUSB0
    sudo chmod 666 /dev/ttyUSB1
    ```

3. (Optional) Add user to dialout group (permanent solution):

    ```bash
    sudo usermod -aG dialout $USER
    # Logout and login again
    ```

## Usage

1. **Important⚠️** Modify the device parameters in `launch.py`

    Modify the `LEFT_GELLO_DEVICE`, `RIGHT_GELLO_DEVICE`, `ARM_TYPE`, `GRIPPER_TYPE`, `DEVICE_LEFT_IP`, `DEVICE_RIGHT_IP`, `HEXARM_LEFT_DEVICE_PORT` and `HEXARM_RIGHT_DEVICE_PORT` in `launch.py` to match your devices. `CAN0` => `8439`, `CAN1` => `9439`.

    Assuming:
    - GELLO device ports are `/dev/ttyUSB0` and `/dev/ttyUSB1`
    - The arm is `archer_y6` with `gp100` gripper
    - The controller IP is `172.18.5.116`
    - The left arm is plugged into `CAN0`, which means the device port is `8439`
    - The right arm is plugged into `CAN1`, which means the device port is `9439`

    ```python
    ...
    LEFT_GELLO_DEVICE = "/dev/ttyUSB0"
    RIGHT_GELLO_DEVICE = "/dev/ttyUSB1"
    ARM_TYPE = "archer_y6"
    GRIPPER_TYPE = "gp100"
    DEVICE_LEFT_IP = "172.18.5.116"
    DEVICE_RIGHT_IP = "172.18.5.116"
    HEXARM_LEFT_DEVICE_PORT = 8439
    HEXARM_RIGHT_DEVICE_PORT = 9439
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
    cd examples/adv/double_gello_real
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```

## Safety Notice

- ⚠️ Make sure there is enough safe space around the robot and you can cut off power at any time.
