#!/usr/bin/env python3
"""BGLX mesh concept in the existing Humble / Gazebo Classic stack."""
import os
import re
import json
import subprocess
import tempfile
import xacro
from pathlib import Path
import xml.etree.ElementTree as E
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node



def cleanup(path):
    Path(path).unlink(missing_ok=True)
    return []


def apply_colors(sdf, palette, mesh_dir):
    count = 0
    for visual in sdf.findall('.//link/visual'):
        uri = visual.find('geometry/mesh/uri')
        if uri is None:
            continue
        filename = Path(uri.text).name
        if filename not in palette or 'concept_color' not in uri.text:
            continue
        path = Path(mesh_dir, filename)
        if not path.is_file():
            raise RuntimeError(f'Material mesh missing: {path}')
        uri.text = path.as_uri()
        for old in visual.findall('material'):
            visual.remove(old)
        material = E.SubElement(visual, 'material')
        rgba = ' '.join(str(v) for v in palette[filename]['rgba'])
        E.SubElement(material, 'ambient').text = rgba
        E.SubElement(material, 'diffuse').text = rgba
        E.SubElement(material, 'specular').text = '0.15 0.15 0.15 1'
        E.SubElement(material, 'emissive').text = '0 0 0 1'
        count += 1
    if count != len(palette):
        raise RuntimeError(f'Expected {len(palette)} coloured visuals after SDF conversion; got {count}')
    return count


def colored_description(desc):
    urdf_path = Path(desc, 'urdf', 'etrike.urdf.xacro')
    controller_path = str(Path(desc, 'config', 'controllers.yaml'))
    robot = xacro.process_file(str(urdf_path), mappings={'controllers_file': controller_path}).toxml()
    robot = re.sub(r'<!--.*?-->', '', robot, flags=re.DOTALL).strip()
    robot = E.tostring(E.fromstring(robot), encoding='unicode')
    if ': ' in robot:
        raise RuntimeError('robot_description contains colon-space; cannot pass Classic parameter parser')
    with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf') as source:
        source.write(robot)
        source.flush()
        converted = subprocess.run(['gz', 'sdf', '-p', source.name],
                                   capture_output=True, text=True, check=True, timeout=45)
    sdf = E.fromstring(converted.stdout)
    urdf = E.fromstring(robot)
    if len(sdf.findall('.//sensor')) != len(urdf.findall('.//sensor')):
        raise RuntimeError('Sensor count changed during SDF conversion')
    palette = json.loads(Path(desc, 'meshes', 'concept_color', 'palette.json').read_text())
    count = apply_colors(sdf, palette, Path(desc, 'meshes', 'concept_color'))
    with tempfile.NamedTemporaryFile(mode='w', suffix='.sdf', prefix='bglx_color_', delete=False) as file:
        file.write(E.tostring(sdf, encoding='unicode', xml_declaration=False))
        filename = file.name
    print(f'BGLX: {count} explicit Gazebo material assignments; {len(sdf.findall(".//sensor"))} sensors retained.')
    return robot, filename


def on_success(next_actions, stage):
    def finished(event, context):
        if event.returncode != 0:
            reason = f'{stage} failed with exit code {event.returncode}; see the preceding log.'
            return [LogInfo(msg=reason), EmitEvent(event=Shutdown(reason=reason))]
        return next_actions
    return finished


def start(context):
    if LaunchConfiguration('use_sim_time').perform(context).lower() != 'true':
        raise RuntimeError('Gazebo integration requires use_sim_time:=true for stamped scan safety')
    desc = get_package_share_directory('etrike_description')
    nav = get_package_share_directory('bglx_navigation')
    gazebo = get_package_share_directory('gazebo_ros')
    world = os.path.abspath(os.path.expanduser(LaunchConfiguration('world').perform(context)))
    if not os.path.isfile(world):
        raise RuntimeError(f'World file does not exist: {world}')
    robot, sdf_path = colored_description(desc)
    scan_filter = Node(package='bglx_navigation', executable='concept_front_scan_filter',
                       name='bglx_front_scan_filter', output='screen',
                       parameters=[{'use_sim_time': True, 'robot_description': robot}])
    state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                 name='robot_state_publisher', output='screen',
                 parameters=[{'use_sim_time': True, 'robot_description': robot}],
                 remappings=[('joint_states', '/joint_states')])
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py', name='spawn_etrike',
                 output='screen', arguments=[
                     '-file', sdf_path, '-entity', 'bglx_etrike',
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
    from launch.actions import ExecuteProcess
    server = ExecuteProcess(
        cmd=['gzserver', world, '--verbose',
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so',
             '-s', 'libgazebo_ros_force_system.so',
             '--ros-args', '--params-file',
             os.path.join(
                 get_package_share_directory('etrike_description'),
                 'config', 'gazebo_clock.yaml')],
        output='screen')
    client = IncludeLaunchDescription(PythonLaunchDescriptionSource(
        os.path.join(gazebo, 'launch', 'gzclient.launch.py')), condition=IfCondition(LaunchConfiguration('gui')))
    events.append(RegisterEventHandler(OnShutdown(
        on_shutdown=lambda event, context: cleanup(sdf_path))))
    viewer = Node(package='rviz2', executable='rviz2', name='concept_sensor_viewer',
                  output='screen', parameters=[{'use_sim_time': True}],
                  arguments=['-d', os.path.join(desc, 'rviz', 'concept_sensors.rviz')],
                  condition=IfCondition(LaunchConfiguration('viewer')))
    events.append(RegisterEventHandler(OnProcessExit(target_action=scan_filter,
        on_exit=on_success([], "Front scan filter"))))
    return events + [server, client, state, scan_filter, spawn, viewer]


def generate_launch_description():
    desc = get_package_share_directory('etrike_description')
    default_world = str(Path(desc, 'worlds', 'campus.world'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Simulation requires true'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('viewer', default_value='false'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.03'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        OpaqueFunction(function=start),
    ])
