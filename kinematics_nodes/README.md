# Kinematics Nodes

ROS2 service node for analytical kinematics solvers (IKFast). Exposes kinematics as a MoveIt-compatible service.

## Quick Start

### Launch

```bash
ros2 launch sfb_qa_cell_configuration bringup.launch.xml
```

### Test

```bash
# Simple test - checks if service works
ros2 run kinematics_nodes test_kinematics_service.sh
```

## Service

- **Name:** `/compute_plugin_ik`
- **Type:** `moveit_msgs/srv/GetPositionIK`
- **Input:** Cartesian pose + optional seed state
- **Output:** Joint angles or error code

## Parameters (Required)

| Parameter | Description |
|-----------|-------------|
| `plugin_name` | Kinematics plugin class name (e.g., `fanuc_lrmate200id_ikfast/FanucLrmate200idKinematics`) |
| `robot_description` | Robot URDF as XML string |
| `base_link` | Base frame name (e.g., `base_link`) |
| `tip_link` | End-effector frame name (e.g., `flange`) |
| `alpha` | Jacobian damping factor (default: `0.000005`) |

**Note:** All parameters except `alpha` are required. The node will fail to start if any required parameter is missing.

## Example Launch

```xml
<node pkg="kinematics_nodes" exec="ik_plugin_service_node" output="screen">
  <param name="plugin_name" value="fanuc_lrmate200id_ikfast/FanucLrmate200idKinematics"/>
  <param name="base_link" value="base_link"/>
  <param name="tip_link" value="flange"/>
  <param name="robot_description" value="$(var robot_description_content)"/>
</node>
```

## Example Service Call

```bash
ros2 service call /compute_plugin_ik moveit_msgs/srv/GetPositionIK "{
  ik_request: {
    group_name: 'manipulator_left',
    ik_link_name: 'left_gripper_tcp_link',
    pose_stamped: {
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 0.4, y: 0.0, z: 0.6},
        orientation: {w: 1.0}
      }
    }
  }
}"
```

## Features

- Loads any kinematics plugin via pluginlib
- Automatic URDF joint name extraction from kinematic chain
- Validates requests and returns MoveIt error codes
- Supports frame transforms via TF2
- Finds closest IK solution to seed state
- Handles tool offsets (TCP to flange transforms)

## Test Script

The `test_kinematics_service.sh` script verifies:
1. Service is available
2. IK request returns valid solution

**Usage:**
```bash
# With service running
ros2 run kinematics_nodes test_kinematics_service.sh
```
