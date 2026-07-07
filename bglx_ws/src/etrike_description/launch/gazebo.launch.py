#!/usr/bin/env python3
"""
BGLX E-Trike Gazebo Simulation Launch File
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Get package directory
    pkg_dir = get_package_share_directory('etrike_description')
    
    # Paths
    urdf_file = os.path.join(pkg_dir, 'urdf', 'etrike.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'campus.world')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=world_file)
    
    # Process xacro
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ]),
        launch_arguments={
            'world': world,
            'pause': 'false'
        }.items()
    )
    
    # Gazebo client
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
        }]
    )
    
    # Spawn robot in Gazebo
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
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'world',
            default_value=world_file,
            description='World file to load'
        ),
        
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot,
    ])