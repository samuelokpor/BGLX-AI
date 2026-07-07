#!/usr/bin/env python3
"""
BGLX E-Trike RViz Display Launch File
Visualize the robot model in RViz with joint state GUI
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('etrike_description')
    
    # Paths
    urdf_file = os.path.join(pkg_dir, 'urdf', 'etrike.urdf.xacro')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'etrike.rviz')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_gui = LaunchConfiguration('use_gui', default='true')
    
    # Process xacro to get robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Robot State Publisher - publishes TF transforms
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint State Publisher GUI - allows manual joint control
    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=None  # Always run for now
    )
    
    # RViz
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint state publisher GUI'
        ),
        
        # Nodes
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
