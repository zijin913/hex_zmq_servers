# Multi Berxel Example

## Description

This example shows how to use multiple Berxel RGB-D camera devices. It captures RGB and depth images from multiple Berxel cameras.

## Structure

```bash
multi_berxel/
├── cli.py     # client code (working code)
├── cli.json   # client configuration
├── launch.py  # launch script
└── README.md  # this file
```

## Dependencies

### Hardware

- Berxel RGB-D camera

### Environment

1. Check device connection:

    ```bash
    lsusb | grep Berxel
    ```

2. If permission denied, you may need to add udev rules. For details, please refer to the [Berxel Website](https://www.hessian-matrix.com/%e4%b8%8b%e8%bd%bd%e4%b8%ad%e5%bf%83/).

## Usage

1. **Important⚠️** Modify the device parameters in `launch.py`

    Modify the `CAM_*_SERIAL_NUMBER` in `launch.py` to match your devices before running the example.

    Assuming the device serial numbers are `P050HYX5421E2A008`, `P050HYX5421E2A009` and `P050HYX5421E2A010`, you can modify the `launch.py` as follows:

    ```python
    ...
    CAM_0_SERIAL_NUMBER = "P050HYX5421E2A008"
    CAM_1_SERIAL_NUMBER = "P050HYX5421E2A009"
    CAM_2_SERIAL_NUMBER = "P050HYX5421E2A010"
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
    cd examples/adv/multi_berxel
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```
