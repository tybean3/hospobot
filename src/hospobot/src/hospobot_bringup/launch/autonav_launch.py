import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    hospobot_bringup_dir = get_package_share_directory('hospobot_bringup')
    hospobot_launch_path = os.path.join(hospobot_bringup_dir, 'launch', 'hospobot_launch.py')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_arg = LaunchConfiguration('map', default='')
    launch_webdash = LaunchConfiguration('launch_webdash', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('map', default_value='', description='Map file or name in global_maps for localization'),
        DeclareLaunchArgument('launch_webdash', default_value='false', description='Launch web server and rosbridge if true'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hospobot_launch_path),
            launch_arguments={
                'nav_mode': 'localization',
                'use_sim_time': use_sim_time,
                'map': map_arg,
                'launch_webdash': launch_webdash
            }.items()
        )
    ])
