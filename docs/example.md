<h1 align="center">EXAMPLE</h1>

<p align="center">
    <a href="example.md">
        <img src="https://img.shields.io/badge/中文-active-2ea44f?style=flat-square&logo=googletranslate" />
    </a>
</p>

---

# Overview

This document shows how to use various devices and functions in `hex_zmq_servers`.

# Basic Examples

| Name                | Description            | Path                                                                           |
| ------------------- | ---------------------- | ------------------------------------------------------------------------------ |
| `zmq_dummy`         | ZMQ communication test | [examples/basic/zmq_dummy](examples/basic/zmq_dummy/README.md)                 |
| `robot_dummy`       | Dummy robot            | [examples/basic/robot_dummy](examples/basic/robot_dummy/README.md)             |
| `robot_gello`       | Gello robot            | [examples/basic/robot_gello](examples/basic/robot_gello/README.md)             |
| `robot_hexarm`      | HexArm robot           | [examples/basic/robot_hexarm](examples/basic/robot_hexarm/README.md)           |
| `cam_dummy`         | Dummy camera           | [examples/basic/cam_dummy](examples/basic/cam_dummy/README.md)                 |
| `cam_rgb`           | RGB camera             | [examples/basic/cam_rgb](examples/basic/cam_rgb/README.md)                     |
| `cam_berxel`        | Berxel RGB-D           | [examples/basic/cam_berxel](examples/basic/cam_berxel/README.md)               |
| `cam_realsense`     | RealSense RGB-D        | [examples/basic/cam_realsense](examples/basic/cam_realsense/README.md)         |
| `mujoco_archer_y6`  | MuJoCo Archer Y6       | [examples/basic/mujoco_archer_y6](examples/basic/mujoco_archer_y6/README.md)   |
| `mujoco_e3_desktop` | MuJoCo E3 Desktop      | [examples/basic/mujoco_e3_desktop](examples/basic/mujoco_e3_desktop/README.md) |



- **robot_dummy**
  - Description: Dummy robot example, showing how to use robot device.
  - [Details](basic/robot_dummy/README.md)
- **robot_gello**
  - Description: GELLO robot example, showing how to use GELLO robot.
  - [Details](basic/robot_gello/README.md)
- **robot_hexarm**
  - Description: HexArm robot example, showing how to use HexArm robot.
  - [Details](basic/robot_hexarm/README.md)
- **cam_dummy**
  - Description: Dummy camera example, showing how to use camera device.
  - [Details](basic/cam_dummy/README.md)
- **cam_rgb**
  - Description: RGB camera example, showing how to use RGB camera.
  - [Details](basic/cam_rgb/README.md)
- **cam_berxel**
  - Description: Berxel depth camera example, showing how to use Berxel RGB-D camera.
  - [Details](basic/cam_berxel/README.md)
- **cam_realsense**
  - Description: Realsense RGB-D camera example, showing how to use Realsense RGB-D camera.
  - [Details](basic/cam_realsense/README.md)
- **mujoco_archer_y6**
  - Description: Archer Y6 simulation example, showing how to use Archer Y6 simulation.
  - [Details](basic/mujoco_archer_y6/README.md)
- **mujoco_e3_desktop**
  - Description: E3 Desktop simulation example, showing how to use E3 Desktop simulation.
  - [Details](basic/mujoco_e3_desktop/README.md)
- **zmq_dummy**
  - Description: ZMQ communication test example, showing how to communicate with the device server.
  - [Details](basic/zmq_dummy/README.md)

# Advanced Examples
| Name                          | Description                | Path                                                                                           |
| ----------------------------- | -------------------------- | ---------------------------------------------------------------------------------------------- |
| `double_gello_sim`            | Gello + MuJoCo teleop      | [examples/adv/double_gello_sim](examples/adv/double_gello_sim/README.md)                       |
| `double_gello_real`           | Gello + E3-Desktop teleop  | [examples/adv/double_gello_real](examples/adv/double_gello_real/README.md)                     |
| `gello_sim`                   | Gello + MuJoCo teleop      | [examples/adv/gello_sim](examples/adv/gello_sim/README.md)                                     |
| `gello_real`                  | Gello + HexArm teleop      | [examples/adv/gello_real](examples/adv/gello_real/README.md)                                   |
| `joy_sim`                     | Joystick + MuJoCo          | [examples/adv/joy_sim](examples/adv/joy_sim/README.md)                                         |
| `joy_real`                    | Joystick + HexArm          | [examples/adv/joy_real](examples/adv/joy_real/README.md)                                       |
| `force_feedback`              | Force feedback teleop      | [examples/adv/force_feedback](examples/adv/force_feedback/README.md)                           |
| `zero_gravity`                | Zero-gravity (torque comp) | [examples/adv/zero_gravity](examples/adv/zero_gravity/README.md)                               |
| `traj_axis`                   | Trajectory along axis      | [examples/adv/traj_axis](examples/adv/traj_axis/README.md)                                     |
| `traj_circle`                 | Trajectory circle          | [examples/adv/traj_circle](examples/adv/traj_circle/README.md)                                 |
| `traj_point`                  | Trajectory point           | [examples/adv/traj_point](examples/adv/traj_point/README.md)                                   |
| `multi_berxel`                | Multiple Berxel            | [examples/adv/multi_berxel](examples/adv/multi_berxel/README.md)                               |
| `multi_realsense`             | Multiple RealSense         | [examples/adv/multi_realsense](examples/adv/multi_realsense/README.md)                         |
| `multi_launch`                | Multi-node launch          | [examples/adv/multi_launch](examples/adv/multi_launch/README.md)                               |
| `multi_launch_mujoco`         | Multi-node MuJoCo          | [examples/adv/multi_launch_mujoco](examples/adv/multi_launch_mujoco/README.md)                 |
| `multi_launch_force_feedback` | Multi-node force feedback  | [examples/adv/multi_launch_force_feedback](examples/adv/multi_launch_force_feedback/README.md) |
| `multi_launch_zero_gravity`   | Multi-node zero-gravity    | [examples/adv/multi_launch_zero_gravity](examples/adv/multi_launch_zero_gravity/README.md)     |
| `multi_launch_berxel`         | Multi-node Berxel          | [examples/adv/multi_launch_berxel](examples/adv/multi_launch_berxel/README.md)                 |
| `multi_launch_realsense`      | Multi-node RealSense       | [examples/adv/multi_launch_realsense](examples/adv/multi_launch_realsense/README.md)           |


- **double_gello_sim**
  - Description: GELLO + Mujoco simulation teleoperation example, showing how to use GELLO controller to control Mujoco simulation.
  - [Details](adv/double_gello_sim/README.md)
- **double_gello_real**
  - Description: GELLO + E3-Desktop teleoperation example, showing how to use GELLO controller to control E3-Desktop robot.
  - [Details](adv/double_gello_real/README.md)
- **gello_sim**
  - Description: GELLO + Mujoco simulation teleoperation example, showing how to use GELLO controller to control Mujoco simulation.
  - [Details](adv/gello_sim/README.md)
- **gello_real**
  - Description: GELLO + HexArm robot teleoperation example, showing how to use Gello controller to control HexArm robot.
  - [Details](adv/gello_real/README.md)
- **joy_sim**
  - Description: Joystick + Mujoco simulation control example, showing how to use Joystick to control Mujoco simulation.
  - [Details](adv/joy_sim/README.md)
- **joy_real**
  - Description: Joystick + HexArm robot control example, showing how to use Joystick to control HexArm robot.
  - [Details](adv/joy_real/README.md)
- **force_feedback**
  - Description: Force feedback teleoperation example, showing how to control HexArm robot using another HexArm robot with force feedback.
  - [Details](adv/force_feedback/README.md)
- **zero_gravity**
  - Description: Zero gravity test example, showing how to use torque compensation to compensate the gravity of HexArm robot.
  - [Details](adv/zero_gravity/README.md)
- **traj_axis**
  - Description: Trajectory axis example, showing how to generate and execute a trajectory axis with the HexArm robot.
  - [Details](adv/traj_axis/README.md)
- **traj_circle**
  - Description: Trajectory circle example, showing how to generate and execute a trajectory circle with the HexArm robot.
  - [Details](adv/traj_circle/README.md)
- **traj_point**
  - Description: Trajectory point example, showing how to generate and execute a trajectory point with the HexArm robot.
  - [Details](adv/traj_point/README.md)
- **multi_berxel**
  - Description: Multi Berxel example, showing how to use multiple Berxel RGB-D camera devices.
  - [Details](adv/multi_berxel/README.md)
- **multi_realsense**
  - Description: Multi Realsense example, showing how to use multiple Realsense RGB-D camera devices.
  - [Details](adv/multi_realsense/README.md)
- **multi_launch**
  - Description: Multi launch example, showing how to use multiple launch files.
  - [Details](adv/multi_launch/README.md)
- **multi_launch_mujoco**
  - Description: Multi launch Mujoco example, showing how to use multiple launch files to start multiple Mujoco simulation.
  - [Details](adv/multi_launch_mujoco/README.md)
- **multi_launch_force_feedback**
  - Description: Multi launch Force feedback example, showing how to use multiple launch files to start multiple Force feedback devices.
  - [Details](adv/multi_launch_force_feedback/README.md)
- **multi_launch_zero_gravity**
  - Description: Multi launch Zero gravity example, showing how to use multiple launch files to start multiple HexArm robots in zero gravity mode.
  - [Details](adv/multi_launch_zero_gravity/README.md)
- **multi_launch_berxel**
  - Description: Multi launch Berxel example, showing how to use multiple launch files to start multiple Berxel RGB-D camera devices.
  - [Details](adv/multi_launch_berxel/README.md)
- **multi_launch_realsense**
  - Description: Multi launch Realsense example, showing how to use multiple launch files to start multiple Realsense RGB-D camera devices.
  - [Details](adv/multi_launch_realsense/README.md)
