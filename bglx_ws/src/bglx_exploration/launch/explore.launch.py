import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('bglx_exploration')
    params = os.path.join(pkg, 'config', 'explore.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=params),
        Node(
            package='bglx_exploration', executable='frontier_explorer',
            name='frontier_explorer', output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': True}],
        ),
    ])
