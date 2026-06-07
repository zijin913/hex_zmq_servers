# Camera RGB Example

## Description

This example shows how to use RGB camera device. It captures RGB images from /dev/video* camera.

## Structure

```bash
cam_rgb/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- RGB camera

### Environment

1. Check device connection:

    ```bash
    ls /dev/video*
    ```

## Usage

1. **Important⚠️** Modify the device parameters in `launch.py`

    Modify the `CAM_PATH` in `launch.py` to match your device before running the example.

    Assuming the device path is `/dev/video0`, you can modify the `launch.py` as follows:

    ```python
    ...
    CAM_PATH = "/dev/video0"
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
    cd examples/basic/cam_rgb
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```
