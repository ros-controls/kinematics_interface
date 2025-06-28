
# kinematics_interface_pinocchio

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)  
[![ROS Humble](https://img.shields.io/badge/ROS-Humble-brightgreen.svg?logo=ros)](https://docs.ros.org/en/humble/index.html)
[![ROS Jazzy](https://img.shields.io/badge/ROS-Jazzy-brightgreen.svg?logo=ros)](https://docs.ros.org/en/jazzy/index.html)
[![ROS Rolling](https://img.shields.io/badge/ROS-Rolling-brightgreen.svg?logo=ros)](https://docs.ros.org/en/rolling/index.html)

A [Pinocchio](https://github.com/stack-of-tasks/pinocchio)-based [ROS 2 Kinematics Interface](https://github.com/ros-controls/kinematics_interface) plugin. This can serve as a drop-in replacement for the default [KDL plugin](https://github.com/ros-controls/kinematics_interface/tree/master/kinematics_interface_kdl) in ROS 2, offering a different backend for forward and inverse kinematics.
Requires pinocchio to be installed: `apt install ros-rolling-pinocchio`.

---

## Overview

This package provides:

- A Pinocchio-based implementation of the `kinematics_interface` for ROS 2.
- The same plugin interface as the default KDL plugin (`kinematics_interface_kdl`).
- A path to potentially better performance (to be benchmarked) and additional features enabled by Pinocchio.

### Installation

You can install this package directly from your ROS 2 distribution’s repositories:

```bash
sudo apt-get update
sudo apt-get install ros-${ROS_DISTRO}-kinematics-interface-pinocchio
```

Alternatively, you can clone this repository into your ROS workspace and build from source (requires pinocchio to be installed/built).

---

## Build Status

| ROS Distro | Build Status                                                                                                                                                                                                                                        |
| ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Humble     | [![Build and Test (ROS 2 Humble)](https://github.com/justagist/kinematics_interface_pinocchio/actions/workflows/ros2-humble-ci.yml/badge.svg)](https://github.com/justagist/kinematics_interface_pinocchio/actions/workflows/ros2-humble-ci.yml)    |
| Jazzy      | [![Build and Test (ROS 2 Jazzy)](https://github.com/justagist/kinematics_interface_pinocchio/actions/workflows/ros2-jazzy-ci.yml/badge.svg)](https://github.com/justagist/kinematics_interface_pinocchio/actions/workflows/ros2-jazzy-ci.yml)       |
| Rolling    | [![Build and Test (ROS 2 Rolling)](https://github.com/justagist/kinematics_interface_pinocchio/actions/workflows/ros2-rolling-ci.yml/badge.svg)](https://github.com/justagist/kinematics_interface_pinocchio/actions/workflows/ros2-rolling-ci.yml) |


---

## Usage

To use this plugin in place of the default `KDL` plugin:

1. Update your ROS 2 package dependencies to include `kinematics_interface_pinocchio`.
2. In your configuration or launch files, specify the Pinocchio plugin shared library instead of `kinematics_interface_kdl`.

An example snippet:

```xml
<!-- Example usage in a ROS 2 controller configuration -->
<plugin>kinematics_interface_pinocchio/KinematicsInterfacePinocchio</plugin>
```

---

## TODO

- [x] Replicate KDL implementations and capabilities with Pinocchio  
- [x] Package as a ROS 2 plugin usable in place of `kinematics_interface_kdl`  
- [ ] Benchmark and compare performance  
- [ ] Allow arbitrary frames to be used as base (currently uses the model’s root joint)  
- [ ] Add support for floating-base robots  
- [ ] Add support for continuous joints and other composite joints supported by Pinocchio  

---

## License

This project is licensed under the [Apache 2.0 License](LICENSE). See the [LICENSE](LICENSE) file for details.

---  

Contributions welcome!
