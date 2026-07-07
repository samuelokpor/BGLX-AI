#!/usr/bin/env python3
"""
BGLX E-Trike SLAM Launch File
Launches slam_toolbox for mapping the environment
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('etrike_description')
    
    # Config file path
    slam_config = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'slam.rviz')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # SLAM Toolbox Node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config,
            {'use_sim_time': use_sim_time}
        ],
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        slam_toolbox_node,
        rviz_node,
    ])