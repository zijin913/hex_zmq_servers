# Joystick Sim Control Example

## Description

This example shows how to use a gamepad to control Mujoco Archer Y6 simulation.

## Structure

```bash
joy_sim/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- USB gamepad

### Environment

None.

## Usage

1. Activate the virtual environment

    ```bash
    cd path/to/hex_zmq_servers
    source .venv/bin/activate
    ```

2. Run the launch script

    Run the launch script:

    ```bash
    cd examples/adv/joy_sim
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```
