# Camera Realsense Example

## Description

This example shows how to use Realsense RGB-D camera device. It captures RGB and depth images from Realsense camera.

## Structure

```bash
cam_realsense/
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

    Modify the `SERIAL_NUMBER` in `launch.py` to match your device before running the example.

    Assuming the device serial number is `243422071854`, you can modify the `launch.py` as follows:

    ```python
    ...
    SERIAL_NUMBER = "243422071854"
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
    cd examples/basic/cam_realsense
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```
