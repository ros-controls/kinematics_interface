# kinematics_nodes

ROS 2 service node that loads any `kinematics_interface` plugin (KDL, IKFast, Pinocchio) and exposes a MoveIt-compatible IK service.

## Service

- **`custom_compute_ik`** (`moveit_msgs/srv/GetPositionIK`)

## Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `plugin_name` | string | Yes | | Kinematics plugin class name |
| `robot_description` | string | Yes | | URDF XML string |
| `base` | string | Yes | | Base link name |
| `tip` | string | Yes | | Tip/flange link name |
| `alpha` | double | No | 0.000005 | Jacobian damping factor |

## Usage

```xml
<node pkg="kinematics_nodes" exec="ik_plugin_service_node" output="screen">
  <param name="plugin_name" value="kinematics_interface_kdl/KinematicsInterfaceKDL"/>
  <param name="base" value="base_link"/>
  <param name="tip" value="flange"/>
  <param name="robot_description" value="$(var robot_description_content)"/>
</node>
```
