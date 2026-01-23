# Zero Gravity Example

## Description

This example demonstrates zero gravity mode (also called free-drive or gravity compensation mode) for HexArm robot. In this mode, the robot actively compensates its own gravity, allowing operators to manually move the robot with minimal force. This is ideal for teaching programming and path planning.

## Structure

```bash
zero_gravity/
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
    cd examples/adv/zero_gravity
    python launch.py
    ```

    The output should be like this:

    ```bash
    Using system clock
    [launcher] Terminal settings recorded
    [launcher] Started 2 nodes
    [launcher] Starting zero_gravity_cli
    [launcher] Starting robot_hexarm_srv
    [zero_gravity_cli] Using system clock
    [robot_hexarm_srv] Using system clock
    [robot_hexarm_srv] 2026-01-21 18:17:51 - hex_device - INFO - Your control frequency is 500Hz, the report frequency is Rf500HzHz now.
    [robot_hexarm_srv] 2026-01-21 18:17:51 - hex_device - INFO - HexDevice Api started.
    [robot_hexarm_srv] 2026-01-21 18:17:51 - hex_device - INFO - HexDeviceApi initialized.
    [robot_hexarm_srv] Arm not found
    [robot_hexarm_srv] 2026-01-21 18:17:51 - hex_device - INFO - Get log from server: New connection from [::ffff:172.18.24.73]:52270 at Instant { tv_sec: 20, tv_nsec: 247727258 }. Assigning session id 2. Current Backend Git hash: 2420e05, Build time: 2026-01-19T11:44:57.240368642+08:00
    [robot_hexarm_srv] 2026-01-21 18:17:51 - hex_device - INFO - Begin periodic task for ArmArcher_27
    [robot_hexarm_srv] 2026-01-21 18:17:51 - hex_device - WARNING - Arm: Can not control the arm, now holder is ID: 0, waiting...
    [robot_hexarm_srv] 2026-01-21 18:17:51 - hex_device - INFO - Arm init success
    [zero_gravity_cli] client recv failed; recreate socket
    [zero_gravity_cli] client recv failed; recreate socket
    [zero_gravity_cli] client recv failed; recreate socket
    [robot_hexarm_srv] The length of mit_kp and mit_kd is greater than the number of motors
    [robot_hexarm_srv] 2026-01-21 18:17:52 - hex_device - INFO - Arm: You can control the arm now! Your session ID: 2
    ...
    ```

## Safety Notice

- ⚠️ Make sure there is enough safe space around the robot and you can cut off power at any time.
