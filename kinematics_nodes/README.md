# kinematics_nodes

ROS 2 node that loads a `kinematics_interface` plugin and exposes a MoveIt-compatible IK service. Works with any plugin implementing the `kinematics_interface::KinematicsInterface` base class (KDL, IKFast, Pinocchio).

## How it works

1. Reads the URDF and extracts the kinematic chain between `base` and `tip` links
2. Loads the specified kinematics plugin via `pluginlib`
3. Exposes a `kinematics_compute_ik` service that accepts a target pose and returns joint positions

The node uses the same parameter names (`tip`, `base`, `alpha`) that the plugins expect, so plugin initialization works without extra parameter bridging.

## Service

- **`kinematics_compute_ik`** (`moveit_msgs/srv/GetPositionIK`) -- given a target pose, returns an IK joint solution

## Parameters

| Parameter | Type | Required | Default | Description |
|-----------|------|----------|---------|-------------|
| `plugin_name` | string | Yes | | Plugin class name |
| `robot_description` | string | Yes | | URDF XML string |
| `base` | string | Yes | | Base link of kinematic chain |
| `tip` | string | Yes | | End-effector link of kinematic chain |
| `alpha` | double | No | 0.000005 | Jacobian damping factor |

## Available plugins

> [!NOTE]
> IKFast plugins are generated per robot and are not shipped with `kinematics_interface`.
> Use the `plugin_name` exported by your robot-specific IKFast package (for example, `fanuc_lrmate200id_ikfast/FanucLrmate200idKinematics`).

| Plugin | `plugin_name` value |
|--------|---------------------|
| KDL | `kinematics_interface_kdl/KinematicsInterfaceKDL` |
| IKFast (Fanuc LRMate200iD) | `fanuc_lrmate200id_ikfast/FanucLrmate200idKinematics` |
| Pinocchio | `kinematics_interface_pinocchio/KinematicsInterfacePinocchio` |

## Launch example

```xml
<let name="robot_description_content"
     value="$(command '$(find-exec xacro) $(find-pkg-share my_robot)/urdf/robot.urdf.xacro')"/>

<node pkg="kinematics_nodes" exec="ik_plugin_service_node" output="screen">
  <param name="plugin_name" value="fanuc_lrmate200id_ikfast/FanucLrmate200idKinematics"/>
  <param name="base" value="base_link"/>
  <param name="tip" value="flange"/>
  <param name="robot_description" value="$(var robot_description_content)"/>
</node>
```
