#!/bin/bash
# Simple test for kinematics service node
# Usage: ./test_kinematics_service.sh

set -e

echo "Testing kinematics service..."

# Check if service exists
if ! ros2 service list | grep -q "compute_plugin_ik"; then
    echo "FAILED: Service /compute_plugin_ik not found!"
    echo "   Start the service with: ros2 launch sfb_qa_cell_configuration bringup.launch.xml"
    exit 1
fi

echo "Service found"

# Test IK request
echo "Sending IK request..."
RESPONSE=$(ros2 service call /compute_plugin_ik moveit_msgs/srv/GetPositionIK "{
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
}" --wait 5 2>&1)

# Check response
if echo "$RESPONSE" | grep -q "error_code:\s*1"; then
    echo "✓ PASSED: IK service working correctly"
    exit 0
else
    echo "FAILED: IK request failed"
    echo "$RESPONSE" | grep "error_code:"
    exit 1
fi
