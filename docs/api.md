<h1 align="center">API</h1>

---

# Overview

This document lists servers, clients, and utility functions in `hex_zmq_servers`.

---

# Common Utilities

## Functions

| Function                                                  | Description                                      |
| --------------------------------------------------------- | ------------------------------------------------ |
| `hex_zmq_ts_to_ns(ts: dict)->int`                         | Convert `{s, ns}` to nanoseconds.                |
| `ns_to_hex_zmq_ts(ns: int)->dict`                         | Convert nanoseconds to `{s, ns}`.                |
| `hex_ns_now()->int`                                       | Current time in ns (PTP if `HEX_PTP_CLOCK` set). |
| `hex_zmq_ts_now()->dict`                                  | `{s, ns}` for now.                               |
| `hex_zmq_ts_delta_ms(curr_ts: dict, hdr_ts: dict)->float` | Delta in ms.                                     |

## Classes

### HexRate

- A Class used to fix the rate of a loop.
- Usage:
  
  ```python
  rate = HexRate(hz)
  while True:
    ...
    rate.sleep()
  ```


---

# Devices

- client common api:
  
    | Function       | Description                     |
    | -------------- | ------------------------------- |
    | `is_working()` | Check if the client is working. |
    | `close()`      | Close the client.               |

- server common parameters:

    ```python
    "net": {
        "ip": str,
        "port": int,
        "realtime_mode": bool,
        "deque_maxlen": int,
        "client_timeout_ms": int,
        "server_timeout_ms": int,
        "server_num_workers": int
    },
    ```

## Camera Devices

- camera client common api:

    | Function                          | Description          |
    | --------------------------------- | -------------------- |
    | `get_rgb(newest: bool = False)`   | Get the RGB image.   |
    | `get_depth(newest: bool = False)` | Get the depth image. |

### Dummy Camera

- Dummy camera client extra api:
    None

- Dummy camera server parameters:
    None

- Usage:
  See [examples/basic/cam_dummy](../examples/basic/cam_dummy/README.md) for details.

### RGB Camera

- RGB camera client extra api:

    | Function      | Description            |
    | ------------- | ---------------------- |
    | `get_intri()` | Get camera intrinsics. |

- RGB camera server parameters:

    ```python
    "params": {
        "cam_path": str,
        "resolution": [int, int],
        "crop": [int, int, int, int],
        "exposure": int,
        "temperature": int,
        "frame_rate": int,
        "sens_ts": bool
    }
    ```

- Usage:
  See [examples/basic/cam_rgb](../examples/basic/cam_rgb/README.md) for details.

### Berxel Camera

- Berxel camera client extra api:

    | Function      | Description            |
    | ------------- | ---------------------- |
    | `get_intri()` | Get camera intrinsics. |

- Berxel camera server parameters:

    ```python
    "params": {
        "serial_number": str,
        "exposure": int,
        "gain": int,
        "frame_rate": int,
        "sens_ts": bool
    }
    ```

- Usage:
  See [examples/basic/cam_berxel](../examples/basic/cam_berxel/README.md) for details.

### RealSense Camera

- RealSense camera client extra api:

    | Function      | Description            |
    | ------------- | ---------------------- |
    | `get_intri()` | Get camera intrinsics. |

- RealSense camera server parameters:

    ```python
    "params": {
        "serial_number": str,
        "resolution": [int, int],
        "frame_rate": int,
        "sens_ts": bool
    }
    ```

- Usage:
  See [examples/basic/cam_realsense](../examples/basic/cam_realsense/README.md) for details.

## Robot Devices

- robot client common api:

    | Function                           | Description                       |
    | ---------------------------------- | --------------------------------- |
    | `seq_clear()`                      | Clear sequence number cache.      |
    | `get_dofs()`                       | Get DOF list.                     |
    | `get_limits()`                     | Get position limits.              |
    | `get_states(newest: bool = False)` | Get the latest robot states.      |
    | `set_cmds(cmds: np.ndarray)`       | Send command array to the server. |

### Dummy Robot

- Dummy robot client extra api:
    None

- Dummy robot server parameters:

    ```python
    "params": {
        "dofs": [int],
        "limits": [[[float, float]]],
        "states_init": [[float, float, float]]
    }
    ```

- Usage:
  See [examples/basic/robot_dummy](../examples/basic/robot_dummy/README.md) for details.

### HexArm Robot

- HexArm robot client extra api:
    None

- HexArm robot server parameters:

    ```python
    "params": {
        "device_ip": str,
        "device_port": int,
        "control_hz": int,
        "arm_type": str,
        "mit_kp": [float],
        "mit_kd": [float],
        "sens_ts": bool
    }
    ```

- Usage:
  See [examples/basic/robot_hexarm](../examples/basic/robot_hexarm/README.md) for details.

### Gello Robot

- Gello robot client extra api:
    None

- Gello robot server parameters:

    ```python
    "params": {
        "idxs": [int],
        "invs": [float],
        "limits": [[float, float]],
        "device": str,
        "baudrate": int,
        "max_retries": int,
        "torque_enabled": bool,
        "sens_ts": bool
    }
    ```

- Usage:
  See [examples/basic/robot_gello](../examples/basic/robot_gello/README.md) for details.

## Mujoco Devices

- mujoco client common api:

    | Function                                          | Description                       |
    | ------------------------------------------------- | --------------------------------- |
    | `reset()`                                         | Reset the MuJoCo state.           |
    | `seq_clear()`                                     | Clear sequence number cache.      |
    | `get_dofs()`                                      | Get DOF list.                     |
    | `get_limits()`                                    | Get position limits.              |
    | `get_states(robot_name: str, newest: bool=False)` | Get robot or object states.       |
    | `get_rgb(camera_name: str, newest: bool=False)`   | Get RGB image from a camera.      |
    | `get_depth(camera_name: str, newest: bool=False)` | Get depth image from a camera.    |
    | `set_cmds(cmds: np.ndarray)`                      | Send command array to the server. |
    | `get_intri()`                                     | Get camera intrinsics.            |

### Archer Y6 MuJoCo

- Archer Y6 client extra api:
    None

- Archer Y6 client recv config:

    ```python
    {
        "rgb": bool,
        "depth": bool,
        "obj": bool
    }
    ```

- Archer Y6 server parameters:

    ```python
    "params": {
        "control_hz": int,
        "states_rate": int,
        "img_rate": int,
        "tau_ctrl": bool,
        "mit_kp": [float],
        "mit_kd": [float],
        "cam_type": str,
        "headless": bool,
        "sens_ts": bool
    }
    ```

- Usage:
  See [examples/basic/mujoco_archer_y6](../examples/basic/mujoco_archer_y6/README.md) for details.

### E3-Desktop MuJoCo

- E3-Desktop client extra api:

    | Function                                      | Description                          |
    | --------------------------------------------- | ------------------------------------ |
    | `set_cmds(cmds: np.ndarray, robot_name: str)` | Send commands for left or right arm. |

- E3-Desktop client recv config:

    ```python
    {
        "head_rgb": bool,
        "head_depth": bool,
        "left_rgb": bool,
        "left_depth": bool,
        "right_rgb": bool,
        "right_depth": bool,
        "obj": bool
    }
    ```

- E3-Desktop server parameters:

    ```python
    "params": {
        "control_hz": int,
        "states_rate": int,
        "img_rate": int,
        "tau_ctrl": bool,
        "mit_kp": [float],
        "mit_kd": [float],
        "cam_type": [str, str, str],
        "headless": bool,
        "sens_ts": bool
    }
    ```

- Usage:
  See [examples/basic/mujoco_e3_desktop](../examples/basic/mujoco_e3_desktop/README.md) for details.
