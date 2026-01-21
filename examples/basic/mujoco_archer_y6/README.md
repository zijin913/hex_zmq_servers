# Mujoco Archer Y6 Example

## Description

This example shows how to use Mujoco simulation for Archer Y6 robot. It simulates the Archer Y6 robot in Mujoco physics engine and provides position, velocity, torque control, as well as RGB and depth image feedback.

## Structure

```bash
mujoco_archer_y6/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

None.

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
    cd examples/basic/mujoco_archer_y6
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```
