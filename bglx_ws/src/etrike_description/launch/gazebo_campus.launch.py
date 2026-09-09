from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    desc = Path(get_package_share_directory('etrike_description'))
    actions = [DeclareLaunchArgument('world', default_value=str(desc/'worlds'/'oxford_college.world')),
               IncludeLaunchDescription(PythonLaunchDescriptionSource(str(desc/'launch'/'gazebo.launch.py')),
                   launch_arguments={'world': LaunchConfiguration('world')}.items())]

    return LaunchDescription(actions)
