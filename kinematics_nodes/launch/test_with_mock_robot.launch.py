#!/usr/bin/env python3
# Copyright (c) 2026 b»robotized
# Launch file for testing ikfast_service_node with mock hardware robot

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare arguments
    use_optimized_scenario_arg = DeclareLaunchArgument(
        'use_optimized_scenario',
        default_value='true',
        description='Use optimized scenario layout (true/false)'
    )
    
    use_mock_hardware_arg = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='true',
        description='Use mock hardware interface (true/false)'
    )
    
    plugin_name_arg = DeclareLaunchArgument(
        'plugin_name',
        default_value='fanuc_lrmate200id_ikfast/FanucLrmate200idKinematics',
        description='Kinematics plugin name'
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
    
    # Get launch configuration
    use_optimized_scenario = LaunchConfiguration('use_optimized_scenario')
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    plugin_name = LaunchConfiguration('plugin_name')
    base_link = LaunchConfiguration('base_link')
    tip_link = LaunchConfiguration('tip_link')
    
    # Include MoveIt setup launch file
    moveit_setup_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('sfb_qa_cell_configuration'),
                'launch',
                'moveit_setup.launch.xml'
            ])
        ),
        launch_arguments={
            'use_optimized_scenario': use_optimized_scenario,
            'use_mock_hardware': use_mock_hardware,
        }.items()
    )
    
    # IKFast service node
    # Note: We get robot_description from the parameter server after MoveIt starts
    ikfast_service_node = Node(
        package='kinematics_nodes',
        executable='ikfast_service_node',
        name='ikfast_service_node',
        output='screen',
        parameters=[{
            'plugin_name': plugin_name,
            'base_link': base_link,
            'tip_link': tip_link,
            'alpha': 0.000005,
        }],
        # Get robot_description from parameter server
        # This requires the node to be started after robot_state_publisher
        # We'll use a simple delay mechanism
    )
    
    return LaunchDescription([
        # Arguments
        use_optimized_scenario_arg,
        use_mock_hardware_arg,
        plugin_name_arg,
        base_link_arg,
        tip_link_arg,
        
        # Launch MoveIt with mock robot
        moveit_setup_launch,
        
        # Note: IKFast service node needs robot_description parameter
        # For now, start it manually after MoveIt is up, or use a timer
        # Uncomment below to auto-start (may need delay):
        # ikfast_service_node,
    ])
