#!/bin/bash
# Test script for ikfast_service_node validation
# Tests both valid and invalid configurations

set -e

ROBOT_URDF="/home/oguzhanb/sfb_jazzy_ws/src/fanuc_lrmate200id/robot.urdf"
PLUGIN_NAME="fanuc_lrmate200id_ikfast/FanucLrmate200idKinematics"

echo "=========================================="
echo "IKFast Service Node Test Suite"
echo "=========================================="
echo ""

# Test 1: Invalid base_link - should FAIL at startup
echo "Test 1: Invalid base_link parameter (should fail at startup)"
echo "----------------------------------------------------------"
echo "Running with base_link='otuzbvir' (does not exist in URDF)"
echo ""

ros2 run kinematics_nodes ikfast_service_node \
  --ros-args \
  -p plugin_name:="${PLUGIN_NAME}" \
  -p robot_description:="$(cat ${ROBOT_URDF})" \
  -p base_link:=otuzbvir \
  -p tip_link:=link_6 &

NODE_PID=$!
sleep 3

if ps -p $NODE_PID > /dev/null; then
  echo "❌ FAIL: Node should have failed but is still running!"
  kill $NODE_PID
  exit 1
else
  echo "✅ PASS: Node correctly failed with invalid base_link"
fi

echo ""
echo ""

# Test 2: Invalid tip_link - should FAIL at startup
echo "Test 2: Invalid tip_link parameter (should fail at startup)"
echo "----------------------------------------------------------"
echo "Running with tip_link='link2123123_62' (does not exist in URDF)"
echo ""

ros2 run kinematics_nodes ikfast_service_node \
  --ros-args \
  -p plugin_name:="${PLUGIN_NAME}" \
  -p robot_description:="$(cat ${ROBOT_URDF})" \
  -p base_link:=base_link \
  -p tip_link:=link2123123_62 &

NODE_PID=$!
sleep 3

if ps -p $NODE_PID > /dev/null; then
  echo "❌ FAIL: Node should have failed but is still running!"
  kill $NODE_PID
  exit 1
else
  echo "✅ PASS: Node correctly failed with invalid tip_link"
fi

echo ""
echo ""

# Test 3: Valid configuration - should START successfully
echo "Test 3: Valid configuration (should start successfully)"
echo "----------------------------------------------------------"
echo "Running with base_link='base_link', tip_link='link_6'"
echo ""

ros2 run kinematics_nodes ikfast_service_node \
  --ros-args \
  -p plugin_name:="${PLUGIN_NAME}" \
  -p robot_description:="$(cat ${ROBOT_URDF})" \
  -p base_link:=base_link \
  -p tip_link:=link_6 &

NODE_PID=$!
sleep 3

if ! ps -p $NODE_PID > /dev/null; then
  echo "❌ FAIL: Node should be running but has crashed!"
  exit 1
fi

echo "✅ PASS: Node started successfully with valid configuration"
echo ""
echo ""

# Test 4: Service call with wrong link name - should return error
echo "Test 4: IK request with wrong link name (should return error)"
echo "----------------------------------------------------------"
echo "Calling service with ik_link_name='wrong_link_name'"
echo ""

RESPONSE=$(ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{
  ik_request: {
    group_name: 'manipulator',
    ik_link_name: 'wrong_link_name',
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
}" 2>&1)

ERROR_CODE=$(echo "$RESPONSE" | grep "val=" | sed 's/.*val=\([^,]*\).*/\1/')

if [ "$ERROR_CODE" == "1" ]; then
  echo "❌ FAIL: Service returned SUCCESS but should have returned error!"
  echo "Response: $RESPONSE"
  kill $NODE_PID
  exit 1
else
  echo "✅ PASS: Service correctly returned error code (not SUCCESS)"
  echo "Error code: $ERROR_CODE"
fi

echo ""
echo ""

# Test 5: Service call with wrong frame_id - should return error
echo "Test 5: IK request with wrong frame_id (should return error)"
echo "----------------------------------------------------------"
echo "Calling service with frame_id='base_2_link'"
echo ""

RESPONSE=$(ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{
  ik_request: {
    group_name: 'manipulator',
    ik_link_name: 'link_6',
    pose_stamped: {
      header: {frame_id: 'base_2_link'},
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
}" 2>&1)

ERROR_CODE=$(echo "$RESPONSE" | grep "val=" | sed 's/.*val=\([^,]*\).*/\1/')

if [ "$ERROR_CODE" == "1" ]; then
  echo "❌ FAIL: Service returned SUCCESS but should have returned error!"
  echo "Response: $RESPONSE"
  kill $NODE_PID
  exit 1
else
  echo "✅ PASS: Service correctly returned error code (not SUCCESS)"
  echo "Error code: $ERROR_CODE"
fi

echo ""
echo ""

# Test 6: Valid service call - should return SUCCESS
echo "Test 6: Valid IK request (should return SUCCESS)"
echo "----------------------------------------------------------"
echo "Calling service with correct link_6 and base_link"
echo ""

RESPONSE=$(ros2 service call /compute_ik moveit_msgs/srv/GetPositionIK "{
  ik_request: {
    group_name: 'manipulator',
    ik_link_name: 'link_6',
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
}" 2>&1)

ERROR_CODE=$(echo "$RESPONSE" | grep "val=" | sed 's/.*val=\([^,]*\).*/\1/')

if [ "$ERROR_CODE" != "1" ]; then
  echo "❌ FAIL: Service should have returned SUCCESS (val=1) but got: $ERROR_CODE"
  echo "Response: $RESPONSE"
  kill $NODE_PID
  exit 1
else
  echo "✅ PASS: Service returned SUCCESS"
fi

# Clean up
kill $NODE_PID
wait $NODE_PID 2>/dev/null || true

echo ""
echo "=========================================="
echo "All Tests Passed! ✅"
echo "=========================================="
