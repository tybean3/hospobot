import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node

def generate_launch_description():

    # Package Directories
    rover_description_dir = get_package_share_directory('rover_description')
    rover_bringup_dir = get_package_share_directory('rover_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_file = os.path.join(rover_bringup_dir, 'maps', 'hospital_map.yaml')

    # Xacro parsing
    urdf_file = os.path.join(rover_description_dir, 'urdf', 'rover.urdf.xacro')

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_file]),
            'use_sim_time': use_sim_time
        }]
    )

    # 2. ODESC Hardware Node (Differential Drive and Odometry)
    odesc_hardware_node = Node(
        package='odesc_hardware',
        executable='odesc_drive_node',
        name='odesc_hardware',
        output='screen',
        parameters=[{
            'left_serial_port': '/dev/ttyAMA0',
            'right_serial_port': '/dev/ttyAMA3',
            'baudrate': 115200,
            'track_width': 0.4,
            'wheel_radius': 0.08,
            'odom_freq': 20.0,
            'invert_left': False,
            'invert_right': True
        }]
    )

    # 2.1 Cmd Vel Mux
    cmd_vel_mux_node = Node(
        package='odesc_hardware',
        executable='cmd_vel_mux',
        name='cmd_vel_mux',
        output='screen'
    )

    # 2.2 Diagnostics Node
    diagnostics_node = Node(
        package='odesc_hardware',
        executable='diagnostics_node',
        name='diagnostics_node',
        output='screen'
    )

    # 2.3 Rosbridge Server
    rosbridge_server = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    # 3. Lidar Node (sllidar_ros2)
    sllidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{'channel_type': 'serial',
                     'serial_port': '/dev/ttyUSB2', # Adjust depending on actual udev rules
                     'serial_baudrate': 115200,
                     'frame_id': 'laser_frame',
                     'inverted': False,
                     'angle_compensate': True}],
        output='screen'
    )

    # 4. Nav2 Bringup (Navigation Stack) - REMOVED from auto-start
    # This will now be launched dynamically via nav2_manager_node
    
    # 5. Nav2 Process Manager
    nav2_manager_node = Node(
        package='odesc_hardware',
        executable='nav2_manager_node',
        name='nav2_manager_node',
        output='screen'
    )

    # 6. Web Dashboard Server
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000'],
        cwd='/home/hospobot/ros2_ws/web_dash/',
        output='screen'
    )

    # 7. IMU Node (BNO086)
    imu_node = Node(
        package='odesc_hardware',
        executable='bno086_node',
        name='bno086_node',
        output='screen',
        parameters=[{'i2c_bus': 1, 'frequency': 50.0, 'frame_id': 'imu_link', 'alpha': 0.2}]
    )

    # 8. Robot Localization (EKF)
    ekf_config_path = os.path.join(get_package_share_directory('rover_bringup'), 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path]
    )

    # 9. SLAM Toolbox (Online Async)
    slam_config_path = os.path.join(get_package_share_directory('rover_bringup'), 'config', 'mapper_params_online_async.yaml')
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher_node,
        odesc_hardware_node,
        cmd_vel_mux_node,
        diagnostics_node,
        rosbridge_server,
        sllidar_node,
        imu_node,
        ekf_node,
        slam_node,
        nav2_manager_node,
        web_server
    ])
