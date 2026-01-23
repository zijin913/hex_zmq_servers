# GELLO Sim Teleoperation Example

## Description

This example shows how to use GELLO robot to control Mujoco Archer Y6 simulation in real-time. It demonstrates teleoperation where GELLO serves as the master device and the simulated robot follows its movements.

## Structure

```bash
gello_sim/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- GELLO robot device

### Environment

1. Check GELLO device connection:

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

    Modify the `GELLO_DEVICE` in `launch.py` to match your device port before running the example.

    Assuming the device port is `/dev/ttyUSB0`, you can modify the `launch.py` as follows:

    ```python
    ...
    GELLO_DEVICE = "/dev/ttyUSB0"
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
    cd examples/adv/gello_sim
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```
