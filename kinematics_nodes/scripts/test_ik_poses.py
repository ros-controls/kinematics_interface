#!/usr/bin/env python3
"""
Generic IK Service Test Script.
Tests the analytical/plug-in based IK service (e.g., IKFast).
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Point, Quaternion, PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import sys


class IKTester(Node):
    def __init__(self):
        super().__init__("ik_tester")

        # Configuration - Update these to match your setup!
        self.ik_service_name = "/compute_plugin_ik"  # Generic IK service name
        self.planning_group = "manipulator_left"  # Change to your planning group
        self.ik_link_name = "left_gripper_tcp_link"  # Change to your TCP link
        self.base_frame = "base_link"  # Change to your base frame

        self.client = self.create_client(GetPositionIK, self.ik_service_name)
        self.cmd_pub = self.create_publisher(
            JointTrajectory, "/position_trajectory_controller/joint_trajectory", 10
        )

        self.get_logger().info(f"Waiting for {self.ik_service_name} service...")
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f"Service {self.ik_service_name} not available!")
            sys.exit(1)

        self.get_logger().info(f"Service ready! Using:")
        self.get_logger().info(f"  Planning Group: {self.planning_group}")
        self.get_logger().info(f"  IK Link: {self.ik_link_name}")
        self.get_logger().info(f"  Base Frame: {self.base_frame}")

    def quaternion_from_euler(self, roll, pitch, yaw):
        """Convert Euler RPY to Quaternion."""
        cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
        cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
        cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def call_ik_service(self, pose_description, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """Requests IK solution and executes motion."""
        self.get_logger().info(f"--- Testing: {pose_description} ---")

        request = GetPositionIK.Request()
        request.ik_request.group_name = self.planning_group
        request.ik_request.ik_link_name = self.ik_link_name
        request.ik_request.pose_stamped.header.frame_id = self.base_frame
        request.ik_request.pose_stamped.pose.position = Point(x=x, y=y, z=z)
        request.ik_request.pose_stamped.pose.orientation = self.quaternion_from_euler(
            roll, pitch, yaw
        )

        # Use synchronous call (consistent with robot_client_node fix)
        self.get_logger().info(f"Requesting IK for pose: [{x}, {y}, {z}]")
        response = self.client.call(request)

        if response is None:
            self.get_logger().error("IK service call failed (no response)")
            return False

        if response.error_code.val == 1:  # MoveIt SUCCESS = 1
            # Get joint names and positions from service response
            joint_names = list(response.solution.joint_state.name)
            joint_positions = list(response.solution.joint_state.position)

            self.get_logger().info(f"IK Success!")
            self.get_logger().info(f"  Joints: {joint_names}")
            self.get_logger().info(f"  Positions: {[f'{p:.3f}' for p in joint_positions]}")

            self.execute_motion(joint_names, joint_positions)
            return True
        else:
            self.get_logger().error(f"IK Failed with error code: {response.error_code.val}")
            error_codes = {
                -1: "FAILURE",
                -2: "NO_IK_SOLUTION",
                -3: "INVALID_LINK_NAME",
                -4: "IK_TIMEOUT",
                -5: "STATE_INVALID",
                -6: "FRAME_TRANSFORM_FAILURE",
            }
            error_name = error_codes.get(response.error_code.val, "UNKNOWN")
            self.get_logger().error(f"Error: {error_name}")
            return False

    def execute_motion(self, names, positions):
        """Sends trajectory using joint names from the IK service."""
        msg = JointTrajectory()
        msg.joint_names = names

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(sec=2, nanosec=0)

        msg.points = [point]
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"Published trajectory to controller")

    def run_sequence(self):
        """Run a sequence of test poses."""
        test_poses = [
            ("Home Position", 0.4, 0.0, 0.7, 0.0, 0.0, 0.0),
            ("High Reach", 0.3, 0.0, 0.7, 0.0, 0.0, 0.0),
            ("Right Side", 0.4, 0.25, 0.4, 0.0, 0.0, 0.0),
            ("Left Side", 0.4, -0.25, 0.4, 0.0, 0.0, 0.0),
            ("Pointing Down", 0.5, 0.0, 0.5, 0.0, math.pi / 2, 0.0),
        ]

        import time

        for i, pose in enumerate(test_poses):
            self.get_logger().info(f"\n=== Pose {i+1}/{len(test_poses)} ===")
            if self.call_ik_service(*pose):
                time.sleep(3.0)  # Wait for motion to complete
            else:
                self.get_logger().warn("Skipping to next pose...")


def main(args=None):
    rclpy.init(args=args)
    tester = IKTester()
    try:
        tester.run_sequence()
    except KeyboardInterrupt:
        tester.get_logger().info("Test interrupted by user")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
