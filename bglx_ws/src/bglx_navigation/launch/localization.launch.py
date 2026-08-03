#!/usr/bin/env python3
"""Localise against a saved map instead of mapping continuously.

Replaces slam.launch.py. Use SLAM to build a map once; use this every time
afterwards.

Three things this buys that SLAM mapping does not:

  1. Fixed bounds. The map stops growing, so "is the robot inside the mapped
     area" becomes a real question with a stable answer.
  2. Persistent coordinates. Landmarks recorded in one session mean the same
     thing in the next, which is what makes "go to the loading bay" work.
  3. A confidence signal. AMCL's particle spread, published as covariance on
     /amcl_pose, is an explicit "how lost am I" measure. Nothing else in this
     stack can produce one.

    ros2 launch bglx_navigation localization.launch.py map:=/path/to/map.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_map = os.path.join(os.path.expanduser('~'),
                               'projects', 'BGLX', 'maps', 'oxford.yaml')
    params = os.path.join(get_package_share_directory('bglx_navigation'),
                          'config', 'nav2_params.yaml')

    map_yaml = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    lifecycle_nodes = ['map_server', 'amcl']

    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='nav2_map_server', executable='map_server',
            name='map_server', output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'yaml_filename': map_yaml,
                         'topic_name': 'map',
                         'frame_id': 'map'}],
        ),
        Node(
            package='nav2_amcl', executable='amcl',
            name='amcl', output='screen',
            parameters=[params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_localization', output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}],
        ),
    ])
