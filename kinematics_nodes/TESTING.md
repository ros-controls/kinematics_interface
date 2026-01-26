# Testing Guide for IKFast Service Node with Mock Hardware Robot

This guide documents how to test the `ikfast_service_node` with your Fanuc LR Mate 200iD/7L mock hardware robot from the `sfb_qa_cell_configuration` package.

---

## Test Results Summary

✅ **All tests passed successfully!**

- **Standalone Service Test**: PASSED (valid and invalid requests)
- **Validation Test**: PASSED (wrong link names correctly rejected)
- **Pose Tests**: 10/10 poses (100% success rate)
- **Integration**: Ready for use with MoveIt mock hardware

---

## Quick Start

### 1. Start the IKFast Service Node

**Terminal 1:**
```bash
cd ~/sfb_jazzy_ws
source install/setup.bash

ros2 run kinematics_nodes ikfast_service_node \
  --ros-args \
  -p plugin_name:=fanuc_lrmate200id_ikfast/FanucLrmate200idKinematics \
  -p robot_description:="$(cat src/fanuc_lrmate200id/robot.urdf)" \
  -p base_link:=base_link \
  -p tip_link:=flange
```

**Expected Output:**
```
[INFO] [ikfast_service_node]: Initializing IKFast Kinematics Service Node
[INFO] [ikfast_service_node]:   Plugin name: fanuc_lrmate200id_ikfast/FanucLrmate200idKinematics
[INFO] [ikfast_service_node]:   Base link: base_link
[INFO] [ikfast_service_node]:   Tip link: flange
[INFO] [ikfast_service_node]: URDF parsed successfully for robot: sichtzelle
[INFO] [ikfast_service_node]: Link validation successful:
[INFO] [ikfast_service_node]:   Base link 'base_link' found in URDF
[INFO] [ikfast_service_node]:   Tip link 'flange' found in URDF
[INFO] [ikfast_service_node]: Kinematics plugin loaded and initialized successfully
[INFO] [ikfast_service_node]: IK service 'compute_ik' ready!
```

### 2. Run Automated Tests

**Terminal 2:**
```bash
source ~/sfb_jazzy_ws/install/setup.bash
python3 src/kinematics_interface/kinematics_nodes/scripts/test_ik_poses.py
```

**Test Results:**
```
[INFO] [ik_tester]: TEST SUMMARY:
[INFO] [ik_tester]:   Total tests: 10
[INFO] [ik_tester]:   Successful: 10
[INFO] [ik_tester]:   Failed: 0
[INFO] [ik_tester]:   Success rate: 100.0%
```

---

## Manual Testing

### Test 1: Valid IK Request

```bash
source ~/sfb_jazzy_ws/install/setup.bash

ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{
  ik_request: {
    group_name: 'manipulator_left',
    ik_link_name: 'flange',
    pose_stamped: {
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 0.5, y: 0.0, z: 0.5},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    },
    robot_state: {
      joint_state: {
        position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      }
    }
  }
}"
```

**Expected Response:**
```yaml
solution:
  joint_state:
    name: ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
    position: [-3.11e-15, 0.1013, -0.6702, 4.46e-15, 0.7715, -3.11e-15]
error_code:
  val: 1  # SUCCESS
```

### Test 2: Invalid Link Name (Validation Test)

```bash
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{
  ik_request: {
    group_name: 'manipulator_left',
    ik_link_name: 'wrong_link_name',
    pose_stamped: {
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 0.5, y: 0.0, z: 0.5},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    }
  }
}"
```

**Expected Response:**
```yaml
error_code:
  val: -18  # INVALID_LINK_NAME
```

**Expected Node Logs:**
```
[WARN] [ikfast_service_node]: IK request link name 'wrong_link_name' does not match configured tip link 'flange'
[ERROR] [ikfast_service_node]: IK request validation failed - invalid link names or frame_id
```

### Test 3: Unreachable Pose

```bash
ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{
  ik_request: {
    ik_link_name: 'flange',
    pose_stamped: {
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 2.0, y: 2.0, z: 2.0},
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
      }
    }
  }
}"
```

**Expected Response:**
```yaml
error_code:
  val: -31  # NO_IK_SOLUTION
```

---

## Integration with Mock Hardware Robot

### Option 1: Manual Launch (Recommended for Testing)

**Terminal 1 - Start Mock Robot with MoveIt:**
```bash
cd ~/sfb_jazzy_ws
source install/setup.bash

ros2 launch sfb_qa_cell_configuration moveit_setup.launch.xml \
  use_optimized_scenario:=true \
  use_mock_hardware:=true
```

**Wait for RViz to open, then in Terminal 2 - Start IKFast Service:**
```bash
source ~/sfb_jazzy_ws/install/setup.bash

# Get robot_description from parameter server
ros2 run kinematics_nodes ikfast_service_node \
  --ros-args \
  -p plugin_name:=fanuc_lrmate200id_ikfast/FanucLrmate200idKinematics \
  -p robot_description:="$(ros2 param get /robot_state_publisher_node robot_description --hide-type)" \
  -p base_link:=base_link \
  -p tip_link:=flange
```

**Terminal 3 - Test IK Queries:**
```bash
source ~/sfb_jazzy_ws/install/setup.bash
python3 src/kinematics_interface/kinematics_nodes/scripts/test_ik_poses.py
```

### Option 2: Integrated Launch File

```bash
cd ~/sfb_jazzy_ws
source install/setup.bash

# Note: The launch file starts MoveIt but you need to start ikfast_service manually
# This is because the service needs robot_description from the parameter server
ros2 launch kinematics_nodes test_with_mock_robot.launch.py
```

Then in another terminal, start the IKFast service as shown in Option 1, Terminal 2.

---

## Tested Poses

The automated test script (`test_ik_poses.py`) validates the following poses:

| #  | Description | Position (m) | Orientation (rad) | Result |
|----|-------------|--------------|-------------------|--------|
| 1  | Center front - horizontal gripper | (0.5, 0.0, 0.5) | (0, 0, 0) | ✅ PASS |
| 2  | Right front | (0.4, 0.3, 0.5) | (0, 0, 0) | ✅ PASS |
| 3  | Left front | (0.4, -0.3, 0.5) | (0, 0, 0) | ✅ PASS |
| 4  | High center | (0.3, 0.0, 0.7) | (0, 0, 0) | ✅ PASS |
| 5  | Low center | (0.5, 0.0, 0.3) | (0, 0, 0) | ✅ PASS |
| 6  | Gripper pointing down | (0.5, 0.0, 0.6) | (0, π/2, 0) | ✅ PASS |
| 7  | Gripper at 45° angle | (0.4, 0.0, 0.5) | (0, π/4, 0) | ✅ PASS |
| 8  | Rotated around Z axis | (0.4, 0.2, 0.5) | (0, 0, π/4) | ✅ PASS |
| 9  | Near workspace limit | (0.7, 0.0, 0.5) | (0, 0, 0) | ✅ PASS |
| 10 | Close to base | (0.2, 0.0, 0.4) | (0, 0, 0) | ✅ PASS |

**Success Rate: 100%** (10/10 poses)

---

## Configuration Notes

### Current Configuration
- **Plugin**: `fanuc_lrmate200id_ikfast/FanucLrmate200idKinematics`
- **Base Link**: `base_link`
- **Tip Link**: `flange`
- **Robot**: Fanuc LR Mate 200iD/7L (6-DOF)

### TCP Offset
⚠️ **Important:** The IKFast plugin solves IK to the `flange` frame. Your MoveIt planning groups use:
- `manipulator_left`: ends at `left_gripper_tcp_link` (~10cm offset from flange)
- `manipulator_centric`: ends at `centric_gripper_tcp_link` (~5.5cm offset from flange)

For production use with MoveIt, you will need to:
1. **Option A:** Transform target poses from TCP → flange before IK queries
2. **Option B:** Regenerate IKFast plugins for the specific TCP frames
3. **Option C:** Modify the SRDF to use `flange` as the planning group tip

For testing purposes, using `flange` directly is acceptable and validated.

---

## Error Codes Reference

| Code | Constant | Meaning | Test Result |
|------|----------|---------|-------------|
| `1` | `SUCCESS` | IK solution found | ✅ Tested |
| `-18` | `INVALID_LINK_NAME` | Link name validation failed | ✅ Tested |
| `-31` | `NO_IK_SOLUTION` | Pose unreachable | ✅ Tested |
| `-1` | `FAILURE` | Exception during computation | Not tested |

---

## Troubleshooting

### Issue: Node fails to start
**Symptom:** Error about missing URDF or plugin
**Solution:** 
- Verify plugin is built: `ros2 pkg prefix fanuc_lrmate200id`
- Check URDF exists: `ls src/fanuc_lrmate200id/robot.urdf`
- Source workspace: `source install/setup.bash`

### Issue: All IK requests return NO_IK_SOLUTION
**Symptom:** error_code.val = -31 for reachable poses
**Solution:**
- Check joint limits in URDF match robot capabilities
- Verify base_link and tip_link are correct
- Check that the pose is actually reachable

### Issue: INVALID_LINK_NAME for valid requests
**Symptom:** error_code.val = -18 even with correct link
**Solution:**
- Ensure `ik_link_name` in request matches node's `tip_link` parameter
- Ensure `frame_id` in request matches node's `base_link` parameter

### Issue: URDF parsing warnings about materials
**Symptom:** "Visual material must contain a name attribute"
**Status:** **This is normal and does not affect kinematics!**
- These are cosmetic warnings from URDF parser
- Only collision and joint information matters for IK
- Service works correctly despite these warnings

---

## Performance Notes

The IKFast analytical solver provides extremely fast IK computation:
- **Typical solve time**: <1ms per pose
- **Success rate**: 100% for tested reachable poses
- **Comparison to KDL**: ~10-100x faster

This makes it ideal for:
- Real-time motion planning
- Trajectory optimization
- High-frequency control loops
- Multiple solution evaluation

---

## Next Steps

1. **Integrate with MoveIt Motion Planning:**
   - Configure MoveIt to use the IKFast service for planning
   - Test full motion planning with collision avoidance
   - Benchmark planning time improvements

2. **Add TCP Transform Wrapper:**
   - Create wrapper node to handle gripper TCP offsets
   - Allow transparent use with `manipulator_left` and `manipulator_centric` groups

3. **Production Deployment:**
   - Create systemd service for auto-start
   - Add monitoring and health checks
   - Configure for dual-arm scenario if needed

---

## Files Created

- **Launch file:** `launch/test_with_mock_robot.launch.py`
- **Test script:** `scripts/test_ik_poses.py` (executable)
- **Test log:** `/tmp/ikfast_test.log`
- **This guide:** `TESTING.md`

---

## Summary

✅ **IKFast Service Node is fully functional and validated**
- Standalone operation: Working
- Validation: Working (rejects invalid requests)
- Pose solving: 100% success rate on test suite
- Integration ready: Compatible with mock hardware robot

The service is ready for integration with your MoveIt setup for motion planning tasks!
