#!/usr/bin/env python3
"""BGLX mesh concept in the existing Humble / Gazebo Classic stack."""
import os
import re
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def on_success(next_actions, stage):
    def finished(event, context):
        if event.returncode != 0:
            reason = f'{stage} failed with exit code {event.returncode}; see the preceding log.'
            return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]
        return next_actions
    return finished


def start(context):
    desc = get_package_share_directory('etrike_description')
    nav = get_package_share_directory('bglx_navigation')
    gazebo = get_package_share_directory('gazebo_ros')
    world = os.path.abspath(os.path.expanduser(LaunchConfiguration('world').perform(context)))
    if not os.path.isfile(world):
        raise RuntimeError(f'World file does not exist: {world}')
    config = os.path.join(desc, 'config', 'concept_controllers.yaml')
    doc = xacro.process_file(os.path.join(desc, 'urdf', 'concept.urdf.xacro'),
                             mappings={'controllers_file': config})
    # Required by the uploaded stack's gazebo_ros2_control parameter parser.
    robot = re.sub(r'<!--.*?-->', '', doc.toxml(), flags=re.DOTALL).strip()
    if '<!--' in robot or ': ' in robot:
        raise RuntimeError('Unsafe robot_description for the Classic parameter parser')
    state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', output='screen',
                 parameters=[{'use_sim_time': True, 'robot_description': robot}],
                 remappings=[('joint_states', '/joint_states')])
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_etrike',
                 output='screen', arguments=[
                     '-topic', 'robot_description', '-entity', 'bglx_etrike',
                     '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                     '-z', LaunchConfiguration('z'), '-Y', LaunchConfiguration('yaw'),
                     '-timeout', '90.0'])
    def controller(name):
        return Node(package='controller_manager', executable='spawner', output='screen',
                    arguments=[name, '--controller-manager', '/controller_manager',
                               '--controller-manager-timeout', '90'])
    jsb = controller('joint_state_broadcaster')
    drive = controller('tricycle_steering_controller')
    ekf = Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
               output='screen', parameters=[os.path.join(nav, 'config', 'ekf.yaml'),
                                            {'use_sim_time': True}])
    # Register before executing processes, and stop the launch on a failed stage.
    events = [
        RegisterEventHandler(OnProcessExit(target_action=spawn,
            on_exit=on_success([jsb], 'Robot spawn'))),
        RegisterEventHandler(OnProcessExit(target_action=jsb,
            on_exit=on_success([drive], 'Joint state broadcaster activation'))),
        RegisterEventHandler(OnProcessExit(target_action=drive,
            on_exit=on_success([ekf, LogInfo(msg='Concept controllers active; EKF starting.')],
                               'Tricycle controller activation'))),
    ]
    server = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(gazebo, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': world, 'pause': 'false', 'verbose': 'true'}.items())
    client = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(gazebo, 'launch', 'gzclient.launch.py')), condition=IfCondition(LaunchConfiguration('gui')))
    return events + [server, client, state, spawn]


def generate_launch_description():
    desc = get_package_share_directory('etrike_description')
    # Start in a small included calibration world. Pass world:=... for Oxford.
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value=os.path.join(desc, 'worlds', 'concept_test.world')),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.03'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=start),
    ])
