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

### Environment

1. Find the robot IP address.

2. (**Important**) Modify the `ARM_TYPE` and `GRIPPER_TYPE` in `launch.py` to match your device model before running the example.

3. (**Important**) Modify the `ZERO_GRAVITY_0_DEVICE_IP`, `ZERO_GRAVITY_0_DEVICE_PORT`, `ZERO_GRAVITY_1_SRV_PORT` and `ZERO_GRAVITY_1_DEVICE_PORT` in `launch.py` to match your device port (e.g., `ZERO_GRAVITY_0_DEVICE_IP = "192.168.1.101"`, `ZERO_GRAVITY_0_DEVICE_PORT = 8439`, `ZERO_GRAVITY_1_SRV_PORT = 12346` and `ZERO_GRAVITY_1_DEVICE_PORT = 8439`) before running the example.
   1. `CAN0` => `8439`
   2. `CAN1` => `9439`

## Usage

- Assuming you have installed the library from source code, and your `working directory` is `hex_zmq_servers/examples/adv/multi_launch_zero_gravity`, you can run the example by:

    ```bash
    source ../../../.venv/bin/activate
    python3 launch.py
    ```
