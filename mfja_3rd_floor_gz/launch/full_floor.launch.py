import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_path = get_package_share_directory('mfja_3rd_floor_bringup')
    base_launch = os.path.join(pkg_path, 'launch', 'full_floor.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument(
            'pause_during_switch_update',
            default_value='false',
            choices=['true', 'false'],
            description='Pause Gazebo while applying visual switch pose updates.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'pause_during_switch_update': LaunchConfiguration('pause_during_switch_update'),
            }.items(),
        ),
    ])
