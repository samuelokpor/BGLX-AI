import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg = get_package_share_directory('bglx_navigation')
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    map_yaml = os.path.expanduser('~/projects/BGLX/bglx_ws/maps/campus_full.yaml')
    lifecycle_nodes = ['map_server', 'amcl']
    return LaunchDescription([
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[{'use_sim_time': True, 'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl', name='amcl', output='screen',
             parameters=[params, {'use_sim_time': True,
                                  'set_initial_pose': True,
                                  'initial_pose.x': 0.0, 'initial_pose.y': 0.0,
                                  'initial_pose.z': 0.0, 'initial_pose.yaw': 0.0}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'use_sim_time': True, 'autostart': True,
                          'node_names': lifecycle_nodes}]),
    ])
