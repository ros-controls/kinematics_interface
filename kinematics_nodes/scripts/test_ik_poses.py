#!/usr/bin/env python3
"""
Universal IKFast Test Script.
Automatically maps joint names from the IK Service response to the Controller.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Point, Quaternion
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import math
import sys


class IKTester(Node):
    def __init__(self):
        super().__init__("ik_tester")

        self.client = self.create_client(GetPositionIK, "/compute_ikfast")
        self.cmd_pub = self.create_publisher(
            JointTrajectory, "/position_trajectory_controller/joint_trajectory", 10
        )

        self.get_logger().info("Waiting for /compute_ikfast service...")
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error("Service /compute_ikfast not available!")
            sys.exit(1)

        self.get_logger().info("Service ready! Starting dynamic motion sequence...")

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
        """Requests IK solution and uses returned joint names for execution."""
        self.get_logger().info(f"--- Testing: {pose_description} ---")

        request = GetPositionIK.Request()
        request.ik_request.group_name = "manipulator_centric"
        request.ik_request.ik_link_name = "flange"
        request.ik_request.pose_stamped.header.frame_id = "base_link"
        request.ik_request.pose_stamped.pose.position = Point(x=x, y=y, z=z)
        request.ik_request.pose_stamped.pose.orientation = self.quaternion_from_euler(
            roll, pitch, yaw
        )

        # Seed state is still good practice, even if empty
        request.ik_request.robot_state.joint_state.position = [0.0] * 6

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

        if future.done() and future.result().error_code.val == 1:
            response = future.result()
            # DYNAMIC MAPPING: Get names and positions directly from the service response
            joint_names = response.solution.joint_state.name
            joint_positions = response.solution.joint_state.position

            self.get_logger().info(f"IK Success. Found joints: {joint_names}")
            self.execute_motion(joint_names, joint_positions)
            return True
        else:
            self.get_logger().error(f"IK Failed or timed out.")
            return False

    def execute_motion(self, names, positions):
        """Sends trajectory using names provided by the IK service."""
        msg = JointTrajectory()
        msg.joint_names = names  # No longer hardcoded!

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(sec=1, nanosec=0)

        msg.points = [point]
        self.cmd_pub.publish(msg)

    def run_sequence(self):
        test_poses = [
            ("Home Position", 0.4, 0.0, 0.7, 0.0, 0.0, 0.0),
            ("High Reach", 0.3, 0.0, 0.7, 0.0, 0.0, 0.0),
            ("Right Side", 0.4, 0.25, 0.4, 0.0, 0.0, 0.0),
            ("Left Side", 0.4, -0.25, 0.4, 0.0, 0.0, 0.0),
            ("Pointing Down", 0.5, 0.0, 0.5, 0.0, math.pi / 2, 0.0),
        ]

        for pose in test_poses:
            if self.call_ik_service(*pose):
                import time

                time.sleep(2.0)


def main(args=None):
    rclpy.init(args=args)
    tester = IKTester()
    try:
        tester.run_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
