# GELLO Real Teleoperation Example

## Description

This example shows how to use GELLO robot to control a real HexArm robot in real-time. It demonstrates bilateral teleoperation where GELLO serves as the master device and HexArm follows its movements.

## Structure

```bash
gello_real/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- GELLO robot device
- HexArm robot

### Environment

**For GELLO robot:**

1. Check USB connection:

    ```bash
    ls /dev/ttyUSB*
    ```

2. Set device permission (Assuming the device port is `/dev/ttyUSB0`):

    ```bash
    sudo chmod 666 /dev/ttyUSB0
    ```

3. (Optional) Add user to dialout group (permanent solution):

    ```bash
    sudo usermod -aG dialout $USER
    # Logout and login again
    ```

## Usage

1. **Important⚠️** Modify the device parameters in `launch.py`

    Modify the `GELLO_DEVICE`, `ARM_TYPE`, `GRIPPER_TYPE`, `DEVICE_IP` and `HEXARM_DEVICE_PORT` in `launch.py` to match your devices. `CAN0` => `8439`, `CAN1` => `9439`.

    Assuming:
    - GELLO device port is `/dev/ttyUSB0`
    - The arm is `archer_y6` with `gp100` gripper
    - The controller ip is `172.18.5.116`
    - The arm is plugged into `CAN0`, which means the device port is `8439`

    ```python
    ...
    GELLO_DEVICE = "/dev/ttyUSB0"
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
    cd examples/adv/gello_real
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```

## Safety Notice

- ⚠️ Make sure there is enough safe space around the robot and you can cut off power at any time.
