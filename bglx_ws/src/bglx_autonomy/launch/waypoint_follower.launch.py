import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('bglx_autonomy')
    default_params = os.path.join(pkg, 'config', 'waypoints.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        Node(
            package='bglx_autonomy',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            parameters=[LaunchConfiguration('params_file'), {'use_sim_time': True}],
        ),
    ])
