import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Rosbridge WebSocket Server (Port 9090)
    rosbridge_server = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    # 2. HTTP Server serving web_dash static assets (Port 8000)
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000'],
        cwd='/home/hospobot/hospobot_ws/web_dash/',
        output='screen'
    )

    return LaunchDescription([
        rosbridge_server,
        web_server
    ])

