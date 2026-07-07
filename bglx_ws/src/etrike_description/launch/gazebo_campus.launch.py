#!/usr/bin/env python3
"""
BGLX E-Trike Gazebo Simulation Launch File
Professional delivery campus environment
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Get package directories
    pkg_dir = get_package_share_directory('etrike_description')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    
    # Paths
    urdf_file = os.path.join(pkg_dir, 'urdf', 'etrike.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'delivery_campus.world')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'etrike_sim.rviz')
    
    # Launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=world_file)
    gui = LaunchConfiguration('gui', default='true')
    
    # Process xacro to get robot description
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # Declare launch arguments
    declare_world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='World file to load'
    )
    
    declare_gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI'
    )
    
    # Gazebo server
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')
        ]),
        launch_arguments={
            'world': world,
            'verbose': 'true',
            'physics': 'ode',
        }.items()
    )
    
    # Gazebo client (GUI)
    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')
        ]),
        launch_arguments={
            'verbose': 'true',
        }.items()
    )
    
    # Robot State Publisher - with remapping to use Gazebo's joint_states
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
            ('joint_states', '/etrike/joint_states')  # Listen to Gazebo's joint states
        ]
    )
    
    # Spawn robot in Gazebo (with delay to let world load)
    spawn_robot = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                name='spawn_etrike',
                output='screen',
                arguments=[
                    '-topic', 'robot_description',
                    '-entity', 'bglx_etrike',
                    '-x', '-5.0',
                    '-y', '0.5',
                    '-z', '0.3',
                    '-Y', '0.0'
                ]
            )
        ]
    )
    
    # RViz (optional, with delay)
    rviz_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config] if os.path.exists(rviz_config) else [],
                parameters=[{'use_sim_time': use_sim_time}]
            )
        ]
    )
    
    # Teleop keyboard (for manual control)
    # Run separately: ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/etrike/cmd_vel
    
    return LaunchDescription([
        declare_world_arg,
        declare_gui_arg,
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot,
        rviz_node,
    ])