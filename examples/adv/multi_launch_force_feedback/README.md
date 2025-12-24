# Force Feedback Teleoperation Example

## Description

This example demonstrates how to use multiple Force feedback devices using multi-launch. It allows for simultaneous teleoperation of multiple HexArm robots using force feedback.

## Structure

```bash
multi_launch_force_feedback/
├── launch.py  # multi-launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- HexArm robot

### Environment

1. Find the robot IP address.

2. (**Important**) Modify the `ARM_TYPE` and `GRIPPER_TYPE` in `launch.py` to match your device model before running the example.

3. (**Important**) Modify the `FORCE_FEEDBACK_0_MASTER_DEVICE_IP`, `FORCE_FEEDBACK_0_SLAVE_DEVICE_IP`, `FORCE_FEEDBACK_0_MASTER_DEVICE_PORT`, `FORCE_FEEDBACK_0_SLAVE_DEVICE_PORT`, `FORCE_FEEDBACK_1_MASTER_DEVICE_IP`, `FORCE_FEEDBACK_1_SLAVE_DEVICE_IP`, `FORCE_FEEDBACK_1_MASTER_DEVICE_PORT` and `FORCE_FEEDBACK_1_SLAVE_DEVICE_PORT` in `launch.py` to match your device port (e.g., `FORCE_FEEDBACK_0_MASTER_DEVICE_IP = "192.168.1.101"`, `FORCE_FEEDBACK_0_SLAVE_DEVICE_IP = "192.168.1.101"`, `FORCE_FEEDBACK_0_MASTER_DEVICE_PORT = 8439`, `FORCE_FEEDBACK_0_SLAVE_DEVICE_PORT = 8439`, `FORCE_FEEDBACK_1_MASTER_DEVICE_IP = "192.168.1.101"`, `FORCE_FEEDBACK_1_SLAVE_DEVICE_IP = "192.168.1.101"`, `FORCE_FEEDBACK_1_MASTER_DEVICE_PORT = 9439` and `FORCE_FEEDBACK_1_SLAVE_DEVICE_PORT = 9439`) before running the example.
   1. `CAN0` => `8439`
   2. `CAN1` => `9439`

## Usage

- Assuming you have installed the library from source code, and your `working directory` is `hex_zmq_servers/examples/adv/multi_launch_force_feedback`, you can run the example by:

    ```bash
    source ../../../.venv/bin/activate
    python3 launch.py
    ```
