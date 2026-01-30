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
- **Input:** Cartesian pose + seed state
- **Output:** Joint angles or error code

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `plugin_name` | `fanuc_lrmate200id_ikfast/FanucLrmate200idKinematics` | Kinematics plugin to load |
| `robot_description` | - | Robot URDF (required) |
| `base_link` | `base_link` | Base frame name |
| `tip_link` | `flange` | End-effector frame name |
| `alpha` | `0.000005` | Jacobian damping factor |

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
- Automatic URDF joint name extraction
- Validates requests and returns MoveIt error codes
- Finds closest IK solution to seed state

## Test Script

The `test_kinematics_service.sh` script verifies:
1. Service is available
2. IK request returns valid solution

**Usage:**
```bash
# With service running
ros2 run kinematics_nodes test_kinematics_service.sh
```
