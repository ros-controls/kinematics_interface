#!/usr/bin/env python3
"""
Finalized IKFast Test Script with Motion Execution.
Specifically targets the 6-DOF arm while ignoring gripper/claw joints.
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
        super().__init__('ik_tester')
        
        # 1. Define the specific joints handled by the position_trajectory_controller
        # We exclude 'centric_claw3_joint' and 'left_claw_joint' to avoid controller errors.
        self.arm_joints = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        # 2. Setup Service Client for IK
        self.client = self.create_client(GetPositionIK, '/compute_ikfast')
        
        # 3. Setup Publisher for the Arm Controller
        self.cmd_pub = self.create_publisher(
            JointTrajectory, 
            '/position_trajectory_controller/joint_trajectory', 
            10
        )
        
        self.get_logger().info(f'Initializing IK Tester for joints: {self.arm_joints}')
        
        if not self.client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service /compute_ikfast not available!')
            sys.exit(1)
        
        self.get_logger().info('Service ready! Starting motion sequence...')

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
        """Requests IK solution and publishes to controller if found."""
        self.get_logger().info(f'--- Testing: {pose_description} ---')
        
        request = GetPositionIK.Request()
        request.ik_request.group_name = 'am'
        request.ik_request.ik_link_name = 'flange'
        request.ik_request.pose_stamped.header.frame_id = 'base_link'
        request.ik_request.pose_stamped.pose.position = Point(x=x, y=y, z=z)
        request.ik_request.pose_stamped.pose.orientation = self.quaternion_from_euler(roll, pitch, yaw)
        
        # Providing the names tells the IK service exactly which joints to solve for
        request.ik_request.robot_state.joint_state.name = self.arm_joints
        request.ik_request.robot_state.joint_state.position = [0.0] * 6 # Neutral seed
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.done() and future.result().error_code.val == 1:
            joint_positions = future.result().solution.joint_state.position
            self.get_logger().info(f'POSITIONSOZZ: {joint_positions}')
            self.get_logger().info(f'IK Success. Sending trajectory...')
            self.execute_motion(joint_positions)
            return True
        else:
            self.get_logger().error(f'IK Failed or timed out.')
            return False

    def execute_motion(self, positions):
        """Sends a single-point trajectory to the controller."""
        msg = JointTrajectory()
        msg.joint_names = self.arm_joints
        
        point = JointTrajectoryPoint()
        point.positions = list(positions)
        # 1.0s travel time for a smooth transition in mock hardware
        point.time_from_start = Duration(sec=1, nanosec=0)
        
        msg.points = [point]
        self.cmd_pub.publish(msg)

    def run_sequence(self):
        """Ordered list of poses to demonstrate robot capability."""
        test_poses = [
            ('Home Position', 0.4, 0.0, 0.7, 0.0, 0.0, 0.0),
            ('High Reach', 0.3, 0.0, 0.7, 0.0, 0.0, 0.0),
            ('Right Side', 0.4, 0.25, 0.4, 0.0, 0.0, 0.0),
            ('Left Side', 0.4, -0.25, 0.4, 0.0, 0.0, 0.0),
            ('Pointing Down', 0.5, 0.0, 0.5, 0.0, math.pi/2, 0.0),
        ]
        
        for pose in test_poses:
            if self.call_ik_service(*pose):
                # Pause to let the robot finish the movement visually
                import time
                time.sleep(2.0)

def main(args=None):
    rclpy.init(args=args)
    tester = IKTester()
    try:
        tester.run_sequence()
        tester.get_logger().info('Sequence complete!')
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()