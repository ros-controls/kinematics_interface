#!/usr/bin/env python3
"""
Standalone launch file for ikfast_service_node.
Loads robot description from a URDF file.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare arguments
    plugin_name_arg = DeclareLaunchArgument(
        'plugin_name',
        default_value='fanuc_lrmate200id_ikfast/FanucLrmate200idKinematics',
        description='Kinematics plugin name'
    )
    
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('fanuc_lrmate200id'),
            'robot.urdf'
        ]),
        description='Path to robot URDF file'
    )
    
    base_link_arg = DeclareLaunchArgument(
        'base_link',
        default_value='base_link',
        description='Base link of the kinematic chain'
    )
    
    tip_link_arg = DeclareLaunchArgument(
        'tip_link',
        default_value='flange',
        description='Tip link of the kinematic chain'
    )
    
    alpha_arg = DeclareLaunchArgument(
        'alpha',
        default_value='0.000005',
        description='Damping factor for Jacobian calculations'
    )
    
    # Get launch configuration
    plugin_name = LaunchConfiguration('plugin_name')
    urdf_file = LaunchConfiguration('urdf_file')
    base_link = LaunchConfiguration('base_link')
    tip_link = LaunchConfiguration('tip_link')
    alpha = LaunchConfiguration('alpha')
    
    # Read URDF file
    # Note: This uses a Python lambda to read the file at launch time
    from launch.substitutions import Command
    robot_description = Command(['cat ', urdf_file])
    
    # IKFast service node
    ikfast_service_node = Node(
        package='kinematics_nodes',
        executable='ikfast_service_node',
        name='ikfast_service_node',
        output='screen',
        parameters=[{
            'plugin_name': plugin_name,
            'robot_description': robot_description,
            'base_link': base_link,
            'tip_link': tip_link,
            'alpha': alpha,
        }]
    )
    
    return LaunchDescription([
        # Arguments
        plugin_name_arg,
        urdf_file_arg,
        base_link_arg,
        tip_link_arg,
        alpha_arg,
        
        # Node
        ikfast_service_node,
    ])
