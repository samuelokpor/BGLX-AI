#!/usr/bin/env python3
"""SLAM or saved-map localization plus the concept Nav2 and safety stack."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    nav = get_package_share_directory('bglx_navigation')
    desc = get_package_share_directory('etrike_description')
    mode = LaunchConfiguration('mode').perform(context)
    actions = []
    if mode == 'slam':
        actions.append(Node(package='slam_toolbox', executable='async_slam_toolbox_node',
                            name='slam_toolbox', output='screen',
                            parameters=[os.path.join(desc, 'config', 'slam_toolbox.yaml'),
                                        {'use_sim_time': True}]))
    elif mode == 'localization':
        map_path = os.path.abspath(os.path.expanduser(LaunchConfiguration('map').perform(context)))
        if not os.path.isfile(map_path):
            raise RuntimeError(f'Map does not exist: {map_path}. Use mode:=slam to map first.')
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(nav, 'launch', 'concept_localization.launch.py')),
            launch_arguments={'map': map_path, 'use_sim_time': 'true'}.items()))
    else:
        raise RuntimeError('mode must be slam or localization')
    actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(nav, 'launch', 'concept_navigation.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items()))
    rviz = os.path.join(desc, 'rviz', 'slam.rviz')
    actions.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                        arguments=['-d', rviz] if os.path.isfile(rviz) else [],
                        parameters=[{'use_sim_time': True}],
                        condition=IfCondition(LaunchConfiguration('rviz'))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='slam'),
        DeclareLaunchArgument('map', default_value=os.path.expanduser('~/projects/BGLX/maps/concept_oxford.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        OpaqueFunction(function=start),
    ])
