# kinematics_interface
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![codecov](https://codecov.io/gh/ros-controls/kinematics_interface/branch/humble/graph/badge.svg?token=NS73VKPG9V)](https://codecov.io/gh/ros-controls/kinematics_interface/tree/humble)

This is a ROS 2 package for using C++ kinematics frameworks in the context of ROS 2 control. A kinematics interface is designed to allow ROS 2 controllers to control robots in Cartesian space. This package also contains a basic implementation of the interface using KDL.

## Build status

ROS2 Distro | Branch | Build status | Documentation | Released packages
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`master`](https://github.com/ros-controls/kinematics_interface/tree/master) | [![Rolling Binary Build](https://github.com/ros-controls/kinematics_interface/actions/workflows/rolling-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/kinematics_interface/actions/workflows/rolling-binary-build.yml) <br /> [![Rolling Semi-Binary Build](https://github.com/ros-controls/kinematics_interface/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/kinematics_interface/actions/workflows/rolling-semi-binary-build.yml) | [API](http://docs.ros.org/en/rolling/p/kinematics_interface/) [API kdl](http://docs.ros.org/en/rolling/p/kinematics_interface_kdl/) | [kinematics_interface](https://index.ros.org/p/kinematics_interface/#rolling)
**Iron** | [`master`](https://github.com/ros-controls/kinematics_interface/tree/master) | see above  |  [API](http://docs.ros.org/en/iron/p/kinematics_interface/) [API kdl](http://docs.ros.org/en/iron/p/kinematics_interface_kdl/)  | [kinematics_interface](https://index.ros.org/p/kinematics_interface/#iron)
**Humble** | [`humble`](https://github.com/ros-controls/kinematics_interface/tree/humble) | [![Humble Binary Build](https://github.com/ros-controls/kinematics_interface/actions/workflows/humble-binary-build.yml/badge.svg?branch=humble)](https://github.com/ros-controls/kinematics_interface/actions/workflows/humble-binary-build.yml) <br /> [![Humble Semi-Binary Build](https://github.com/ros-controls/kinematics_interface/actions/workflows/humble-semi-binary-build.yml/badge.svg?branch=humble)](https://github.com/ros-controls/kinematics_interface/actions/workflows/humble-semi-binary-build.yml) | [API](http://docs.ros.org/en/humble/p/kinematics_interface/) [API kdl](http://docs.ros.org/en/humble/p/kinematics_interface_kdl/)  | [kinematics_interface](https://index.ros.org/p/kinematics_interface/#humble)

### Explanation of different build types

**NOTE**: There are three build stages checking current and future compatibility of the package.

[Detailed build status](.github/workflows/README.md)

1. Binary builds - against released packages (main and testing) in ROS distributions. Shows that direct local build is possible.

   Uses repos file: `$NAME$-not-released.<ros-distro>.repos`

1. Semi-binary builds - against released core ROS packages (main and testing), but the immediate dependencies are pulled from source.
   Shows that local build with dependencies is possible and if fails there we can expect that after the next package sync we will not be able to build.

   Uses repos file: `$NAME$.repos`
