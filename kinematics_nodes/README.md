# kinematics_nodes

ROS 2 nodes that expose kinematics functionality as services using the `kinematics_interface` package.

## Overview

This package provides executable nodes that load kinematics plugins (KDL, Pinocchio, or IKFast) and expose their functionality as ROS 2 services. The primary use case is providing a MoveIt-compatible inverse kinematics service that works with any kinematics plugin implementation.

## Nodes

### ik_plugin_service_node

A ROS 2 service node that provides inverse kinematics computations using any `kinematics_interface` plugin.

#### Features

- **Plugin-based Architecture**: Works with any kinematics plugin (KDL, Pinocchio, IKFast)
- **MoveIt-compatible Service**: Implements `moveit_msgs/srv/GetPositionIK` service interface
- **Automatic URDF Parsing**: Validates robot model and extracts joint names from kinematic chain
- **TF Transform Support**: Handles tool offset transforms and frame conversions automatically
- **Request Validation**: Validates seed states and joint configurations

#### Service

- **`custom_compute_ik`** (`moveit_msgs/srv/GetPositionIK`): Compute inverse kinematics for a given Cartesian pose

#### Parameters

| Parameter | Type | Required | Description |
|-----------|------|----------|-------------|
| `plugin_name` | string | Yes | Name of the kinematics plugin to load (e.g., `kinematics_interface_kdl/KinematicsInterfaceKDL`) |
| `robot_description` | string | Yes | URDF robot description as XML string |
| `base_link` | string | Yes | Name of the base link for the kinematic chain |
| `tip_link` | string | Yes | Name of the tip/flange link for the kinematic chain |
| `alpha` | double | No | Damping factor for Jacobian inverse (default: 0.000005) |

#### Usage Example

```bash
ros2 run kinematics_nodes ik_plugin_service_node --ros-args \
  -p plugin_name:="plugin_name" \
  -p robot_description:="robot_description" \
  -p base_link:="base_link" \
  -p tip_link:="flange"
```

Call the service:
```bash
ros2 service call /custom_compute_ik moveit_msgs/srv/GetPositionIK \
  '{ik_request: {group_name: "manipulator", ik_link_name: "tcp", pose_stamped: {pose: {position: {x: 0.5, y: 0.0, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}}}'
```

## Dependencies

- `rclcpp`: ROS 2 C++ client library
- `pluginlib`: For loading kinematics plugins
- `kinematics_interface`: Base kinematics interface
- `moveit_msgs`: MoveIt message definitions
- `geometry_msgs`: Geometry message types
- `sensor_msgs`: Sensor message types
- `tf2_eigen`: TF2 Eigen conversions
- `urdf`: URDF parsing
- `eigen`: Linear algebra library

## License

Apache License 2.0

## See Also

- [kinematics_interface](../kinematics_interface/): Base kinematics interface
- [kinematics_interface_kdl](../kinematics_interface_kdl/): KDL plugin implementation
- [kinematics_interface_pinocchio](../kinematics_interface_pinocchio/): Pinocchio plugin implementation
- [kinematics_interface_ikfast](../kinematics_interface_ikfast/): IKFast plugin implementation
