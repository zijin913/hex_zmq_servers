# Multi RealSense Example

## Description

This example shows how to use multiple RealSense RGB-D camera devices. It captures RGB and depth images from multiple RealSense cameras.

## Structure

```bash
multi_realsense/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- RealSense RGB-D camera

### Environment

1. Check device connection:

    ```bash
    lsusb | grep RealSense
    ```

## Usage

1. **Important⚠️** Modify the device parameters in `launch.py`

    Modify the `CAM_*_SERIAL_NUMBER` in `launch.py` to match your devices before running the example.

    Assuming the device serial numbers are `243422071854`, `243422071878` and `243422073194`, you can modify the `launch.py` as follows:

    ```python
    ...
    CAM_0_SERIAL_NUMBER = "243422071854"
    CAM_1_SERIAL_NUMBER = "243422071878"
    CAM_2_SERIAL_NUMBER = "243422073194"
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
    cd examples/adv/multi_realsense
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```
