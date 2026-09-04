#!/usr/bin/env python3
"""BGLX Rover (Robot #002) Gazebo launch.

Mirrors etrike_description/launch/gazebo.launch.py:
  - robot_description built eagerly with xacro.process_file() and stripped of
    XML comments, because gazebo_ros2_control re-injects it as a --param
    override that rcl parses as YAML (a ': ' in a comment breaks it).
  - gzserver/gzclient come from gazebo_ros so libgazebo_ros_factory.so is
    loaded and /spawn_entity actually exists.
  - spawners sequenced after the entity is in the world.
"""
import os
import re
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_XML_COMMENT_RE = re.compile(r'<!--.*?-->', re.DOTALL)


def build_robot_description(xacro_path, controllers_file):
    doc = xacro.process_file(
        xacro_path, mappings={'controllers_file': controllers_file})
    urdf = _XML_COMMENT_RE.sub('', doc.toxml())
    urdf = re.sub(r'\n\s*\n+', '\n', urdf).strip()
    if '<!--' in urdf:
        raise RuntimeError('comments remain; gazebo_ros2_control will fail')
    return urdf


def generate_launch_description():
    pkg_dir = get_package_share_directory('bglx_rover_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'bglx_rover.urdf.xacro')
    controllers_file = os.path.join(pkg_dir, 'config', 'controllers.yaml')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default='')

    robot_description = build_robot_description(urdf_file, controllers_file)

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'pause': 'false'}.items())

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')))

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
        remappings=[('joint_states', '/joint_states')])

    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        name='spawn_bglx_rover', output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'bglx_rover',
                   '-x', '0.0', '-y', '0.0', '-z', '0.20'])

    jsb = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'])

    diff = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['diff_drive_controller',
                   '--controller-manager', '/controller_manager'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world', default_value=''),
        gzserver, gzclient, rsp, spawn,
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[diff])),
    ])
