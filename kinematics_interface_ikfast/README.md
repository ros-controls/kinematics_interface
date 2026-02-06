# kinematics_interface_ikfast

A ROS 2 plugin that provides an IKFast-based implementation of the `kinematics_interface` for high-performance robot kinematics computations.

## Overview

This package implements the `KinematicsInterface` base class using IKFast-generated analytical kinematics solvers. IKFast produces optimized C++ code for specific robot geometries, offering significant performance advantages over numerical methods for compatible manipulators.


## Architecture

The `KinematicsInterfaceIKFast` class is an abstract base class that must be subclassed for specific robots. The subclass must implement three pure virtual methods that interface with the IKFast-generated code:

```cpp
virtual int get_num_joints_internal() = 0;  // Return number of joints
virtual void compute_fk(const double* j, double* etrans, double* erot) = 0;  // Forward kinematics
virtual void compute_ik(const double* etrans, const double* erot, const double* free, void* solutions) = 0;  // Inverse kinematics
```

## Usage

### Prerequisites

This plugin requires:
- ROS 2 (Jazzy, Rolling, Humble, or Kilted)
- IKFast-generated kinematics code for your specific robot
- `kinematics_interface` package

### Creating a Robot-Specific Plugin

#### Method 1: Using RTW (Recommended)

This plugin is designed to work with the [RosTeamWorkspace (RTW)](https://rtw.b-robotized.com) framework:

1. **Create a new ROS 2 package** for your robot:
   ```bash
   rosds
   create-new-package <robot_name> "<robot_name> IKFast Kinematics Interface Plugin package"
   ```

2. **Generate the plugin template**:
   ```bash
   cd <robot_name>
   ~/ros_team_workspace/scripts/ikfast_plugin/setup-ikfast-plugin.bash <robot_name>
   ```

3. **Add your IKFast solution** to the package. The expected structure:
   ```
   <robot_name>/
   ├── CMakeLists.txt
   ├── LICENSE
   ├── <robot_name>_ikfast.xml
   ├── include/
   │   └── <robot_name>/
   │       └── ikfast.h
   ├── package.xml
   ├── robot.urdf
   ├── src/
   │   ├── <robot_name>_ikfast.cpp          # IKFast-generated code
   │   └── <robot_name>_ikfast_plugin.cpp   # Plugin implementation
   └── test/
       └── test_<robot_name>_ikfast_plugin.cpp
   ```

   For generating IKFast code, see the [pyikfast repository](https://github.com/cyberbotics/pyikfast).

4. **Build and use**:
   ```bash
   colcon build --packages-select <robot_name>
   ```

#### Method 2: Manual Implementation

> [!NOTE]
> This method will be updated soon to provide more detailed instructions.

### Plugin Configuration

The plugin accepts the following ROS parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `tip` | string | `"flange"` | Name of the end-effector link |
| `base` | string | `"base_link"` | Name of the base link |
| `alpha` | double | `0.000005` | Damping factor for Jacobian inverse computation |

Example configuration:
```yaml
kinematics_interface:
  ros__parameters:
    tip: "tool0"
    base: "base_link"
    alpha: 0.00001
```

## API Reference

### Core Methods

#### `bool initialize(...)`
Initialize the plugin with robot description and parameters. Must be called before any other method.

#### `bool calculate_link_transform(...)`
Compute the forward kinematics transformation for the end-effector.

#### `bool convert_cartesian_pose_to_all_possible_joint_states(...)`
Get all valid IK solutions for a given Cartesian pose (up to 8 solutions).

#### `bool convert_cartesian_pose_to_closest_joint_state(...)`
Find the IK solution closest to a reference joint configuration.

#### `bool convert_cartesian_pose_to_joint_state_within_range(...)`
Find an IK solution where all joints are within specified limits.

#### `bool calculate_jacobian(...)`
Compute the 6xN Jacobian matrix using numerical differentiation.

#### `bool calculate_jacobian_inverse(...)`
Compute the damped least-squares inverse of the Jacobian.

#### `bool convert_cartesian_deltas_to_joint_deltas(...)`
Convert Cartesian velocity/twist to joint velocities using the Jacobian inverse.

#### `bool convert_joint_deltas_to_cartesian_deltas(...)`
Convert joint velocities to Cartesian velocity/twist using the Jacobian.

## Dependencies

- `kinematics_interface`: Base interface class
- `pluginlib`: For plugin registration
- `tf2_eigen`: For Eigen geometry types
- `rclcpp`: ROS 2 client library
- `Eigen3`: Linear algebra library

## License

Apache License 2.0

## See Also

- [kinematics_interface](../README.md): Base interface package
- [kinematics_interface_kdl](../kinematics_interface_kdl/): KDL-based implementation
- [kinematics_interface_pinocchio](../kinematics_interface_pinocchio/): Pinocchio-based implementation
- [pyikfast](https://github.com/cyberbotics/pyikfast): Tool for generating IKFast code
