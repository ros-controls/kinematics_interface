# kinematics_interface_pinocchio
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![ROS Version](https://img.shields.io/badge/ROS-Humble-brightgreen.svg?logo=ros)](https://docs.ros.org/en/humble/index.html)

A [Pinocchio](https://github.com/stack-of-tasks/pinocchio)-based [ROS2 Kinematics Interface](https://github.com/ros-controls/kinematics_interface) implementation. This can be used instead of the default [KDL implementation](https://github.com/ros-controls/kinematics_interface/tree/master/kinematics_interface_kdl) shipped with default ROS installations.

Requires pinocchio to be installed: `apt install ros-humble-pinocchio`.

## Roadmap

- [x] Replicate KDL implemenations and capabilities with Pinocchio
- [x] Package as a ROS2 plugin which can be used in place of `kinematics_interface_kdl`.
- [ ] Benchmark and compare performances
- [ ] Allow arbitrary frames to be used as base (currently uses root joint from model)
- [ ] Add support for floating-base robots
- [ ] Add support for continuous joints and other composite joints supported by Pinocchio
