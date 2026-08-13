import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# DEST: bglx_ws/src/bglx_navigation/launch/navigation.launch.py
# Track A change: launch the cmd_vel_limiter (Nav2 -> steering controller
# bridge + speed-dependent steering limit).


def generate_launch_description():
    pkg = get_package_share_directory('bglx_navigation')
    bt_xml = os.path.join(pkg, 'config', 'bt', 'navigate_no_spin.xml')
    params = os.path.join(pkg, 'config', 'nav2_params.yaml')
    collision_params = os.path.join(
        pkg, 'config', 'collision_monitor.yaml'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')

    lifecycle_nodes = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]

    common = [params_file, {'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=params),

        Node(
            package='nav2_controller', executable='controller_server',
            output='screen', parameters=common,
            remappings=[('cmd_vel', '/etrike/cmd_vel')],
        ),
        Node(
            package='nav2_planner', executable='planner_server',
            output='screen', parameters=common,
        ),
        Node(
            package='nav2_behaviors', executable='behavior_server',
            output='screen', parameters=common,
            remappings=[('cmd_vel', '/etrike/cmd_vel')],
        ),
        Node(
            package='nav2_bt_navigator', executable='bt_navigator',
            output='screen', parameters=common + [{'default_nav_to_pose_bt_xml': bt_xml}],
        ),
        Node(
            package='nav2_waypoint_follower', executable='waypoint_follower',
            output='screen', parameters=common,
        ),

        # Independent delivery-robot proximity safety layer.
        # Nav2 -> Collision Monitor -> cmd_vel_limiter -> base controller
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[
                collision_params,
                {'use_sim_time': use_sim_time},
            ],
        ),

        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='nav2_lifecycle_manager', executable='lifecycle_manager',
                    name='lifecycle_manager_navigation', output='screen',
                    parameters=[{'use_sim_time': use_sim_time},
                                {'autostart': True},
                                {'node_names': lifecycle_nodes}],
                ),
            ],
        ),

        # Dedicated lifecycle manager for the independent collision-safety layer.
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_collision_monitor',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': True},
                {'node_names': ['collision_monitor']},
            ],
        ),

        # Speed-dependent steering limit + bridge to the tricycle controller.
        Node(
            package='bglx_navigation', executable='cmd_vel_limiter',
            name='cmd_vel_limiter', output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'wheelbase': 1.33,
                'max_steering_angle': 1.047,
                'max_lateral_accel': 1.5,
                'max_linear_vel': 2.78,
                'input_topic': '/etrike/collision_checked_cmd_vel',
                'output_topic': '/tricycle_steering_controller/reference_unstamped',
            }],
        ),
    ])
