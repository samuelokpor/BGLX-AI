#!/usr/bin/env python3
"""
BGLX E-Trike Gazebo Simulation Launch File (Track A: tricycle + ros2_control)
DEST: bglx_ws/src/etrike_description/launch/gazebo.launch.py

Changes vs previous version:
  - FIX: robot_description is now built with xacro.process_file() at launch
    time and stripped of XML comments before being handed to
    robot_state_publisher.

    Why: gazebo_ros2_control (Humble) fetches the URDF from
    robot_state_publisher and re-injects it into its internal node as a
    command-line parameter override:  --param robot_description:=<urdf>
    rcl parses that value as YAML. Any line containing a colon-space, or
    ending in a colon, makes the YAML scanner treat it as a mapping key and
    the whole override fails to parse:

        [gazebo_ros2_control]: parser error Couldn't parse parameter
        override rule: '--param robot_description:=<?xml version="1.0" ?> ...

    The plugin then aborts before creating the controller_manager, so every
    `ros2 control list_controllers` hangs forever waiting on a service that
    will never exist. Our .xacro comments ("DEST: ...", "Exposes three
    joints to ros2_control:") trip exactly this. Stripping comments from the
    flattened runtime string fixes it; the source .xacro files keep theirs.

    NOTE: this cannot be done with Command(['xacro ', ...]) because launch
    substitutions are opaque until runtime and cannot be post-processed.

  - FIX: relay odom TF onto /tf.

    steering_controllers_library publishes the odom -> base_link transform
    on its own namespaced topic, /tricycle_steering_controller/tf_odometry,
    NOT on /tf. Nothing downstream looks there, so without this relay the
    'odom' frame simply does not exist: tf2_echo reports "Invalid frame ID",
    slam_toolbox cannot anchor map -> odom, and every Nav2 costmap stays
    empty. enable_odom_tf: true in controllers.yaml is necessary but not
    sufficient.

    Remapping this in the xacro <plugin><ros> block does NOT work: that
    remaps the plugin's own node, not the controllers spawned inside its
    controller_manager.

  - passes controllers_file into xacro
  - remaps robot_state_publisher's joint_states -> /joint_states
    (fed by joint_state_broadcaster)
  - spawns joint_state_broadcaster then tricycle_steering_controller,
    sequenced after the robot is spawned in Gazebo, then starts the odom
    TF relay once the controller is active

Requires: ros-humble-topic-tools
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


# XML comments do not nest, so a non-greedy DOTALL match is safe here.
_XML_COMMENT_RE = re.compile(r'<!--.*?-->', re.DOTALL)

# Where steering_controllers_library actually publishes odom TF.
ODOM_TF_TOPIC = '/tricycle_steering_controller/tf_odometry'


def build_robot_description(xacro_path: str, controllers_file: str) -> str:
    """Flatten the xacro and make the result safe to pass through rcl's
    YAML-based --param override parser (see module docstring)."""
    doc = xacro.process_file(
        xacro_path,
        mappings={'controllers_file': controllers_file},
    )
    urdf = doc.toxml()

    # Strip XML comments: these carry the ': ' / trailing-':' sequences that
    # break gazebo_ros2_control's parameter override parsing.
    urdf = _XML_COMMENT_RE.sub('', urdf)

    # Collapse the blank lines the comment removal leaves behind.
    urdf = re.sub(r'\n\s*\n+', '\n', urdf).strip()

    # Fail loudly at launch time rather than 30s into a hung spawner.
    if '<!--' in urdf:
        raise RuntimeError(
            'robot_description still contains XML comments after stripping; '
            'gazebo_ros2_control will fail to parse it.'
        )

    return urdf


def generate_launch_description():
    pkg_dir = get_package_share_directory('etrike_description')
    urdf_file = os.path.join(pkg_dir, 'urdf', 'etrike.urdf.xacro')
    world_file = os.path.join(pkg_dir, 'worlds', 'campus.world')
    controllers_file = os.path.join(pkg_dir, 'config', 'controllers.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    world = LaunchConfiguration('world', default=world_file)

    # Evaluated eagerly at launch-description generation time (not a
    # substitution), so it can be post-processed before it reaches the node.
    robot_description = build_robot_description(urdf_file, controllers_file)

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gzserver.launch.py')
        ]),
        launch_arguments={'world': world, 'pause': 'false'}.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'),
                         'launch', 'gzclient.launch.py')
        ])
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': use_sim_time}],
        # joint states are published by joint_state_broadcaster
        remappings=[('joint_states', '/joint_states')],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_etrike',
        output='screen',
        arguments=['-topic', 'robot_description', '-entity', 'bglx_etrike',
                   '-x', '0.0', '-y', '0.0', '-z', '0.3', '-Y', '0.0'],
    )

    # --- ros2_control spawners ---
    # The gazebo_ros2_control plugin declares no <ros><namespace>, so the
    # controller_manager comes up at the ROOT namespace: /controller_manager.
    # controllers.yaml is keyed 'controller_manager:' to match.
    jsb_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
    )

    tricycle_spawner = Node(
        package='controller_manager', executable='spawner', output='screen',
        arguments=['tricycle_steering_controller',
                   '--controller-manager', '/controller_manager'],
    )

    # --- odom TF relay ---
    # Started only after the controller is active, so the source topic
    # already exists and relay does not have to discover it cold.
    odom_tf_relay = Node(
        package='topic_tools', executable='relay',
        name='odom_tf_relay', output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[ODOM_TF_TOPIC, '/tf'],
    )

    # Sequence:
    #   spawn robot -> joint_state_broadcaster -> tricycle controller
    #                                          -> odom TF relay
    jsb_after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot, on_exit=[jsb_spawner])
    )

    tricycle_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=jsb_spawner, on_exit=[tricycle_spawner])
    )

    relay_after_tricycle = RegisterEventHandler(
        OnProcessExit(target_action=tricycle_spawner, on_exit=[odom_tf_relay])
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation time'),
        DeclareLaunchArgument('world', default_value=world_file,
                              description='World file to load'),
        gazebo_server,
        gazebo_client,
        robot_state_publisher,
        spawn_robot,
        jsb_after_spawn,
        tricycle_after_jsb,
        relay_after_tricycle,
    ])