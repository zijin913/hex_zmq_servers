# Camera Dummy Example

## Description

This example shows how to use camera device. It simulates a camera and provides RGB and depth image feedback.

## Structure

```bash
cam_dummy/
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
    cd examples/basic/cam_dummy
    python launch.py
    ```

    The output should be like this:

    ```bash
    ...
    ```
