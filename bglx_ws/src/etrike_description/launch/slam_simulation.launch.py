#!/usr/bin/env python3
"""
BGLX E-Trike Combined Simulation + SLAM Launch
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    pkg_dir = get_package_share_directory('etrike_description')
    
    urdf_file = os.path.join(pkg_dir, 'urdf', 'etrike.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'campus.world')
    slam_config = os.path.join(pkg_dir, 'config', 'slam_toolbox.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'slam.rviz')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Gazebo Server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ]),
        launch_arguments={
            'world': world_file,
            'pause': 'false'
        }.items()
    )
    
    # Gazebo Client
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ])
    )
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('/joint_states', '/etrike/joint_states')
        ]
    )
    
    # Spawn Robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_etrike',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'bglx_etrike',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.3',
            '-Y', '0.0'
        ]
    )
    
    # SLAM Toolbox
    slam_toolbox = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[
                    slam_config,
                    {'use_sim_time': use_sim_time}
                ],
            )
        ]
    )
    
    # RViz
    rviz = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot,
        slam_toolbox,
        rviz,
    ])