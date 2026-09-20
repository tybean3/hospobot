import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

def generate_launch_description():

    # Package Directories
    hospobot_description_dir = get_package_share_directory('hospobot_description')
    hospobot_bringup_dir = get_package_share_directory('hospobot_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    nav_mode = LaunchConfiguration('nav_mode', default='mapping')
    map_arg = LaunchConfiguration('map', default='')

    global_maps_dir = '/home/hospobot/hospobot_ws/global_maps'
    default_map_file = os.path.join(global_maps_dir, 'hospital_map.yaml')

    # Resolve map yaml file for localization
    map_yaml_file = PythonExpression([
        f"(('{global_maps_dir}/' + '", map_arg, "') if not '", map_arg, "'.startswith('/') else '", map_arg, "') + ('.yaml' if not '", map_arg, "'.endswith('.yaml') else '') if '", map_arg, "' != '' else '", default_map_file, "'"
    ])

    is_mapping = ParameterValue(PythonExpression(["'", nav_mode, "' == 'mapping'"]), value_type=bool)

    # Xacro parsing
    urdf_file = os.path.join(hospobot_description_dir, 'urdf', 'hospobot.urdf.xacro')

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

    # 2. CAN interface bring-up (runs before SocketCAN bridge)
    # Requires passwordless sudo for ip link — add:
    #   hospobot ALL=(ALL) NOPASSWD: /sbin/ip link set can0 up type can bitrate *
    # to /etc/sudoers.d/hospobot-can
    can0_setup = ExecuteProcess(
        cmd=['bash', '-c',
             'sudo ip link set can0 down 2>/dev/null; '
             'if ! ip link show can0 >/dev/null 2>&1; then '
             '  sudo modprobe vcan 2>/dev/null; '
             '  sudo ip link add dev can0 type vcan 2>/dev/null; '
             'fi; '
             '(sudo ip link set can0 up type can bitrate 500000 2>/dev/null || sudo ip link set can0 up 2>/dev/null) && '
             'sudo ip link set can0 txqueuelen 1000 2>/dev/null && '
             'echo "[can0] Interface UP (physical/virtual) at 500kbit/s, txqueuelen=1000" || '
             'echo "[can0] WARNING: Failed to bring up can0"'],
        output='screen',
    )


    # 2.1 ODrive CAN Node (Differential Drive and Odometry over CAN)
    odrive_can_node = Node(
        package='hospobot_can_bridge',
        executable='odrive_can_node',
        name='odrive_can_node',
        output='screen',
        parameters=[{
            'left_node_id': 1,
            'right_node_id': 2,
            'track_width': 0.300,
            'wheel_radius': 0.0625,
            'odom_freq': 20.0,
            'invert_drive_left': True,
            'invert_drive_right': False,
            'invert_odom_left': True,
            'invert_odom_right': False,
            'heartbeat_timeout': 2.0,
            'publish_tf': False,
            'max_linear': PythonExpression(["0.25 if '", nav_mode, "' == 'mapping' else 1.05"]),
            'accel_limit': PythonExpression(["0.6 if '", nav_mode, "' == 'mapping' else 2.0"])
        }],
        remappings=[
            ('/odom', '/odom_can')
        ]
    )

    # 2.1 Cmd Vel Mux
    cmd_vel_mux_node = Node(
        package='odesc_hardware',
        executable='cmd_vel_mux',
        name='cmd_vel_mux',
        output='screen',
        parameters=[{
            'invert_controls': False,
            'max_linear_speed': PythonExpression(["0.25 if '", nav_mode, "' == 'mapping' else 0.0"]),
            'max_angular_speed': PythonExpression(["0.45 if '", nav_mode, "' == 'mapping' else 0.0"])
        }]
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
                     'serial_port': '/dev/ttyUSB0', # Adjust depending on actual udev rules
                     'serial_baudrate': 115200,
                     'frame_id': 'laser_frame',
                     'inverted': False,
                     'angle_compensate': True}],
        remappings=[('/scan', '/scan_raw')],
        output='screen'
    )

    # 3.1 Laser Filter Node
    laser_filter_node = Node(
        package='hospobot_perception',
        executable='laser_filter_node',
        name='laser_filter_node',
        output='screen',
        parameters=[{
            'flip_angles': False
        }]
    )

    # 3b. Laser ICP Odometry (Scan-to-scan matching with Kalman filter & outlier rejection)
    icp_odometry_node = Node(
        package='rtabmap_odom',
        executable='icp_odometry',
        name='icp_odometry',
        output='screen',
        parameters=[{
            'frame_id': 'base_footprint',
            'odom_frame_id': 'odom',
            'publish_tf': True,
            'wait_for_transform': 0.2,
            'expected_update_rate': 10.0,
            'Odom/GuessMotion': 'true',
            'Odom/FilteringStrategy': '1',               # Kalman filter smoothing (absorbs bump shocks & caster jerk)
            'Odom/Holonomic': 'false',                   # Enforce differential-drive constraint (prevents lateral skews)
            'Odom/GuessSmoothingDelay': '0.15',          # Smooth velocity prediction across frames
            'Odom/KalmanProcessNoise': '0.001',
            'Odom/KalmanMeasurementNoise': '0.01',
            'Icp/MaxCorrespondenceDistance': '0.25',     # Tighten correspondence search (rejects floor/ceiling glints)
            'Icp/MaxTranslation': '0.18',                # Clamp translation to 18cm/frame (prevents bump displacement skips)
            'Icp/MaxRotation': '0.22',                   # Clamp rotation to ~12.6 deg/frame (prevents tilt heading skews)
            'Icp/OutlierRatio': '0.75',                  # Discard top 25% worst residual points during tilt
            'Odom/ResetCountdown': '0',                  # Maintain continuous prediction on momentary scan occlusion
            'publish_null_when_lost': False,
            'Reg/Force3DoF': 'true',
        }],
        remappings=[
            ('scan', '/scan'),
            ('odom', '/odom')
        ]
    )

    # 4. Nav2 Bringup (Navigation Stack) - REMOVED from auto-start
    # This will now be launched dynamically via nav2_manager_node
    
    # 5. Nav2 Process Manager (not needed in driving mode)
    nav2_manager_node = Node(
        package='odesc_hardware',
        executable='nav2_manager_node',
        name='nav2_manager_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", nav_mode, "' != 'driving'"]))
    )

    # 5b. RTAB-Map RGB-D Odometry - REMOVED (Using On-Device VIO instead)

    pc_to_laser_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/oakd/points'),
            ('scan', '/scan')
        ],
        parameters=[{
            'target_frame': 'virtual_laser_link',
            'transform_tolerance': 0.5,
            'min_height': 0.3,
            'max_height': 1.0,
            'angle_min': -0.7005,
            'angle_max': 0.7005,
            'angle_increment': 0.01,
            'scan_time': 0.33,
            'range_min': 0.2,
            'range_max': 3.5,
            'use_inf': True
        }],
        output='screen'
    )
    
    # 6. Web Dashboard Server
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000'],
        cwd='/home/hospobot/hospobot_ws/web_dash/',
        output='screen'
    )

    # 6b. Mapping Dashboard Server (Port 8001) - Only starts in mapping mode
    mapping_web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8001'],
        cwd='/home/hospobot/hospobot_ws/mapping_dash/',
        condition=IfCondition(PythonExpression(["'", nav_mode, "' == 'mapping'"])),
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
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        remappings=[
            ('odometry/filtered', '/odom')
        ],
        parameters=[os.path.join(hospobot_bringup_dir, 'config', 'ekf.yaml')]
    )
    # 9. SLAM Toolbox (Online Async) - 2D LiDAR Ceres Scan-Matching Mapping
    slam_config_path = os.path.join(hospobot_bringup_dir, 'config', 'mapper_params_online_async.yaml')
    slam_node = Node(
        condition=IfCondition(PythonExpression(["'", nav_mode, "' == 'mapping'"])),
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[
            slam_config_path,
            {'use_sim_time': use_sim_time}
        ]
    )

    # 9a. SLAM Toolbox Lifecycle Manager (Auto-starts slam_toolbox to publish map frame)
    slam_lifecycle_node = Node(
        condition=IfCondition(PythonExpression(["'", nav_mode, "' == 'mapping'"])),
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_slam',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['slam_toolbox']}
        ]
    )

    # 9b. AMCL & Map Server - For Localization
    localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        condition=IfCondition(PythonExpression(["'", nav_mode, "' == 'localization'"])),
        launch_arguments={'map': map_yaml_file, 'use_sim_time': use_sim_time}.items()
    )

    # 10. OAK-D Pro W Camera running On-Device VIO
    camera_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('depthai_ros_driver_v3'), 'launch', 'vio.launch.py')
        ),
        launch_arguments={
            'parent_frame': 'oakd_frame',
            'params_file': os.path.join(get_package_share_directory('hospobot_bringup'), 'config', 'vio_custom.yaml')
        }.items()
    )

    # 10b. Odometry Republisher (Fixes VIO child_frame_id)
    odom_republisher_node = Node(
        package='hospobot_can_bridge',
        executable='odom_republisher',
        name='odom_republisher',
        output='screen'
    )

    # 11. Object Detector Node (Semantic Obstacle Detection)
    object_detector_node = Node(
        package='hospobot_perception',
        executable='object_detector',
        name='object_detector',
        output='screen'
    )
    
    # 12. Semantic Projection Node
    semantic_projection_node = Node(
        package='hospobot_perception',
        executable='semantic_projection_node',
        name='semantic_projection_node',
        output='screen'
    )
    
    # 13. Semantic Tracker Node
    semantic_tracker_node = Node(
        package='hospobot_perception',
        executable='semantic_tracker_node',
        name='semantic_tracker_node',
        output='screen'
    )

    # 14. SocketCAN Receiver and Sender
    socket_can_receiver_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_socketcan'), 'launch', 'socket_can_receiver.launch.py')
        ),
        launch_arguments={
            'interface': 'can0',
            'interval_sec': '0.01',
            'enable_can_fd': 'false',
            'use_bus_time': 'false',
            'auto_activate': 'true'
        }.items()
    )

    socket_can_sender_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros2_socketcan'), 'launch', 'socket_can_sender.launch.py')
        ),
        launch_arguments={
            'interface': 'can0',
            'timeout_sec': '0.01',
            'enable_can_fd': 'false',
            'enable_frame_loopback': 'false',
            'auto_activate': 'true'
        }.items()
    )

    # 15. System CAN Bridge (our custom node)
    system_can_bridge_node = Node(
        package='hospobot_can_bridge',
        executable='system_can_bridge',
        name='system_can_bridge',
        output='screen'
    )

    # 16. PS5 Controller Joy Node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'deadzone': 0.2,
            'autorepeat_rate': 20.0,
        }]
    )

    # 17. Teleop Twist Joy Node (PS5 mapping)
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[{
            'require_enable_button': True,
            'enable_button': 4, # L1 Bumper
            'axis_linear.x': 1, # Left stick Up/Down
            'scale_linear.x': 0.43,
            'axis_angular.yaw': 0, # Left stick Left/Right
            'scale_angular.yaw': 0.46,
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel_ps5')
        ]
    )

    # 18. Footprint Publisher
    footprint_publisher_node = Node(
        package='hospobot_perception',
        executable='footprint_publisher_node',
        name='footprint_publisher',
        output='screen'
    )

    # 19. Robot Pose Circle & 3-Cone LIDAR FOV Visualizer (RViz)
    robot_fov_indicator_node = Node(
        package='hospobot_perception',
        executable='robot_fov_indicator_node',
        name='robot_fov_indicator',
        output='screen',
        parameters=[{
            'flip_angles': False,
            'cone_range': 2.5,
            'robot_radius': 0.26
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('nav_mode', default_value='mapping', description='Navigation mode: mapping or localization'),
        DeclareLaunchArgument('map', default_value='', description='Map file or name in global_maps for mapping or localization'),
        # Bring up can0 first so SocketCAN bridge finds it ready
        can0_setup,
        robot_state_publisher_node,
        odrive_can_node,
        cmd_vel_mux_node,
        diagnostics_node,
        rosbridge_server,
        sllidar_node,
        laser_filter_node,
        icp_odometry_node, # High-precision 2D scan-matching odometry (replaces slipping wheel TF)
        # imu_node, # Disabled, hardware removed
        slam_node,
        slam_lifecycle_node,
        localization_node,
        nav2_manager_node,
        # pc_to_laser_node, # Disabled to avoid duplicate scan topics
        web_server,
        mapping_web_server,
        camera_node,
        odom_republisher_node,
        # ekf_node, # Disabled: icp_odometry directly broadcasts odom->base_footprint TF
        socket_can_receiver_node,
        socket_can_sender_node,
        system_can_bridge_node,
        joy_node,
        teleop_twist_joy_node,
        footprint_publisher_node,
        robot_fov_indicator_node
    ])

