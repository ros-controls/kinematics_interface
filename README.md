# kinematics_interface
This is a ROS 2 package for using C++ kinematics frameworks in the context of ROS 2 control. A kinematics interface is designed to allow ROS 2 controllers to control robots in Cartesian space. This package also contains a basic implementation of the interface using KDL.

## Build status

ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`master`](https://github.com/ros-controls/kinematics_interface/tree/master) | [![Rolling Binary Build](https://github.com/ros-controls/kinematics_interface/actions/workflows/rolling-binary-build-main.yml/badge.svg?branch=master)](https://github.com/ros-controls/kinematics_interface/actions/workflows/rolling-binary-build-main.yml?branch=master) <br /> [![Rolling Semi-Binary Build](https://github.com/ros-controls/kinematics_interface/actions/workflows/rolling-semi-binary-build-main.yml/badge.svg?branch=master)](https://github.com/ros-controls/kinematics_interface/actions/workflows/rolling-semi-binary-build-main.yml?branch=master) |   | [kinematics_interface](https://index.ros.org/p/kinematics_interface/#rolling)
**Humble** | [`master`](https://github.com/ros-controls/kinematics_interface/tree/master) | [![Humble Binary Build](https://github.com/ros-controls/kinematics_interface/actions/workflows/humble-binary-build-main.yml/badge.svg?branch=master)](https://github.com/ros-controls/kinematics_interface/actions/workflows/humble-binary-build-main.yml?branch=master) <br /> [![Humble Semi-Binary Build](https://github.com/ros-controls/kinematics_interface/actions/workflows/humble-semi-binary-build-main.yml/badge.svg?branch=master)](https://github.com/ros-controls/kinematics_interface/actions/workflows/humble-semi-binary-build-main.yml?branch=master) |   | [kinematics_interface](https://index.ros.org/p/kinematics_interface/#humble)

### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

[Detailed build status](.github/workflows/README.md)

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `$NAME$.repos`
