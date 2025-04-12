# kinematics_interface
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![codecov](https://codecov.io/gh/ros-controls/kinematics_interface/graph/badge.svg?token=NS73VKPG9V)](https://codecov.io/gh/ros-controls/kinematics_interface)

This is a ROS 2 package for using C++ kinematics frameworks in the context of ROS 2 control. A kinematics interface is designed to allow ROS 2 controllers to control robots in Cartesian space. This package also contains a basic implementation of the interface using KDL.

## Build status
ROS2 Distro | Branch | Build status | Documentation | Package Build |
:---------: | :----: | :----------: | :-----------: | :---------------:
**Rolling** | [`master`](https://github.com/ros-controls/kinematics_interface/tree/master) | [![Rolling Binary Build](https://github.com/ros-controls/kinematics_interface/actions/workflows/rolling-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/kinematics_interface/actions/workflows/rolling-binary-build.yml) <br> [![Rolling Semi-Binary Build](https://github.com/ros-controls/kinematics_interface/actions/workflows/rolling-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/kinematics_interface/actions/workflows/rolling-semi-binary-build.yml) <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Rdev__kinematics_interface__ubuntu_noble_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Rdev__kinematics_interface__ubuntu_noble_amd64/) | [API](http://docs.ros.org/en/rolling/p/kinematics_interface/) <br> [API kdl](http://docs.ros.org/en/rolling/p/kinematics_interface_kdl/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Rbin_uN64__kinematics_interface__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Rbin_uN64__kinematics_interface__ubuntu_noble_amd64__binary/)
**Jazzy** | [`jazzy`](https://github.com/ros-controls/kinematics_interface/tree/jazzy) | [![Jazzy Binary Build](https://github.com/ros-controls/kinematics_interface/actions/workflows/jazzy-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/kinematics_interface/actions/workflows/jazzy-binary-build.yml) <br> [![Jazzy Semi-Binary Build](https://github.com/ros-controls/kinematics_interface/actions/workflows/jazzy-semi-binary-build.yml/badge.svg?branch=master)](https://github.com/ros-controls/kinematics_interface/actions/workflows/jazzy-semi-binary-build.yml) <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Jdev__kinematics_interface__ubuntu_noble_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Jdev__kinematics_interface__ubuntu_noble_amd64/) | [API](http://docs.ros.org/en/jazzy/p/kinematics_interface/) <br> [API kdl](http://docs.ros.org/en/jazzy/p/kinematics_interface_kdl/) | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Jbin_uN64__kinematics_interface__ubuntu_noble_amd64__binary)](https://build.ros2.org/job/Jbin_uN64__kinematics_interface__ubuntu_noble_amd64__binary/)
**Humble** | [`humble`](https://github.com/ros-controls/kinematics_interface/tree/humble) | [![Humble Binary Build](https://github.com/ros-controls/kinematics_interface/actions/workflows/humble-binary-build.yml/badge.svg?branch=humble)](https://github.com/ros-controls/kinematics_interface/actions/workflows/humble-binary-build.yml) <br> [![Humble Semi-Binary Build](https://github.com/ros-controls/kinematics_interface/actions/workflows/humble-semi-binary-build.yml/badge.svg?branch=humble)](https://github.com/ros-controls/kinematics_interface/actions/workflows/humble-semi-binary-build.yml) <br> [![build.ros2.org](https://build.ros2.org/buildStatus/icon?job=Hdev__kinematics_interface__ubuntu_jammy_amd64&subject=build.ros2.org)](https://build.ros2.org/job/Hdev__kinematics_interface__ubuntu_jammy_amd64/) | [API](http://docs.ros.org/en/humble/p/kinematics_interface/) <br> [API kdl](http://docs.ros.org/en/humble/p/kinematics_interface_kdl/)  | [![Build Status](https://build.ros2.org/buildStatus/icon?job=Hbin_uJ64__kinematics_interface__ubuntu_jammy_amd64__binary)](https://build.ros2.org/job/Hbin_uJ64__kinematics_interface__ubuntu_jammy_amd64__binary/)

## Acknowledgements
The project has received major contributions from the following companies and institutions: [control.ros.org](https://control.ros.org/rolling/doc/acknowledgements/acknowledgements.html)
