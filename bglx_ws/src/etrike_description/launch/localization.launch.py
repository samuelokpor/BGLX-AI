#!/usr/bin/env python3

"""
BGLX E-Trike Saved-Map Localization

NORMAL DELIVERY MODE

Loads a previously serialized slam_toolbox pose graph,
localizes the trike against it, and launches RViz.

This is intentionally separate from slam.launch.py:

    slam.launch.py
        -> site mapping / commissioning

    localization.launch.py
        -> normal delivery operation
"""

import os

from ament_index_python.packages import (
    get_package_share_directory,
)

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
)

from launch.events import matches_action

from launch.substitutions import (
    LaunchConfiguration,
)

from launch_ros.actions import (
    LifecycleNode,
    Node,
)

from launch_ros.event_handlers import (
    OnStateTransition,
)

from launch_ros.events.lifecycle import (
    ChangeState,
)

from lifecycle_msgs.msg import Transition


def generate_launch_description():

    pkg_dir = get_package_share_directory(
        "etrike_description"
    )

    localization_config = os.path.join(
        pkg_dir,
        "config",
        "slam_localization.yaml",
    )

    rviz_config = os.path.join(
        pkg_dir,
        "rviz",
        "slam.rviz",
    )

    # Prototype default.
    #
    # map_file is the basename WITHOUT:
    #   .posegraph
    #   .data
    #
    # It can be overridden at launch time.
    default_map = os.path.expanduser(
        "~/.bglx/maps/oxford_dev"
    )

    use_sim_time = LaunchConfiguration(
        "use_sim_time"
    )

    map_file = LaunchConfiguration(
        "map_file"
    )

    # -------------------------------------------------------
    # SLAM Toolbox Localization
    # -------------------------------------------------------

    slam_toolbox = LifecycleNode(
        package="slam_toolbox",
        executable=(
            "localization_slam_toolbox_node"
        ),
        name="slam_toolbox",
        namespace="",
        output="screen",
        parameters=[
            localization_config,
            {
                "use_sim_time": use_sim_time,
                "use_lifecycle_manager": False,

                # Explicitly keep this node in
                # localization-only mode.
                "mode": "localization",

                # Serialized pose graph basename.
                "map_file_name": map_file,

                # Prototype starts at HOME.
                "map_start_pose": [
                    0.0,
                    0.0,
                    0.0,
                ],
            },
        ],
    )

    # slam_toolbox localization is a LifecycleNode.
    #
    # Configure it automatically...
    configure_slam = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=(
                matches_action(
                    slam_toolbox
                )
            ),
            transition_id=(
                Transition.TRANSITION_CONFIGURE
            ),
        )
    )

    # ...then activate automatically after the
    # configure transition completes.
    activate_slam = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=(
                slam_toolbox
            ),
            start_state="configuring",
            goal_state="inactive",
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=(
                            matches_action(
                                slam_toolbox
                            )
                        ),
                        transition_id=(
                            Transition
                            .TRANSITION_ACTIVATE
                        ),
                    )
                )
            ],
        )
    )

    # -------------------------------------------------------
    # RViz
    # -------------------------------------------------------

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            rviz_config,
        ],
        parameters=[
            {
                "use_sim_time":
                    use_sim_time
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
            description=(
                "Use simulation/Gazebo clock"
            ),
        ),

        DeclareLaunchArgument(
            "map_file",
            default_value=default_map,
            description=(
                "Serialized slam_toolbox "
                "pose-graph basename"
            ),
        ),

        slam_toolbox,
        configure_slam,
        activate_slam,
        rviz,
    ])
