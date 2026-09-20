import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    # Directories
    hospobot_description_dir = get_package_share_directory('hospobot_description')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')
    
    # Environment variables
    set_env_action = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(get_package_prefix('hospobot_description'), 'share')
    )

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    nav_mode = LaunchConfiguration('nav_mode', default='mapping')
    map_yaml_file = LaunchConfiguration('map', default=os.path.join(get_package_share_directory('hospobot_bringup'), 'maps', 'SimMap-26-8-26.yaml'))

    # URDF configuration
    urdf_file = os.path.join(hospobot_description_dir, 'urdf', 'hospobot_gazebo.urdf.xacro')

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

    # 2. Gazebo Server and Client
    hospital_world = os.path.join(get_package_share_directory('hospobot_bringup'), 'worlds', 'hospital.sdf')
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {hospital_world}'}.items()
    )

    # 2.1 Cmd Vel Mux
    cmd_vel_mux_node = Node(
        package='odesc_hardware',
        executable='cmd_vel_mux',
        name='cmd_vel_mux',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # 2.3 Rosbridge Server
    rosbridge_server = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')
        )
    )

    # 2.4 Web Dashboard Server
    web_server = ExecuteProcess(
        cmd=['python3', '-m', 'http.server', '8000'],
        cwd='/home/hospobot/hospobot_ws/web_dash/',
        output='screen'
    )

    # 3. Spawn entity
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'hospobot', '-x', '-13.0', '-y', '0.0', '-z', '0.1'],
        output='screen'
    )
    
    # 3.5 Bridge ROS and Gazebo Topics
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel_out@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/model/hospobot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            # Camera Bridges
            '/oakd_camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/oakd_camera/depth_image@sensor_msgs/msg/Image[gz.msgs.Image',
            '/oakd_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked',
            '/oakd_camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo'
        ],
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/model/hospobot/tf', '/tf')
        ],
        output='screen'
    )

    # 4. Teleop Control
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{
            'deadzone': 0.2,
            'autorepeat_rate': 20.0,
            'use_sim_time': True
        }]
    )

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
            'use_sim_time': True
        }],
        remappings=[
            ('/cmd_vel', '/cmd_vel_ps5')
        ]
    )

    # 5. RTAB-Map SLAM
    rtabmap_node = Node(
        condition=IfCondition(PythonExpression(["'", nav_mode, "' == 'mapping'"])),
        package='rtabmap_slam',
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[{
            'frame_id': 'base_footprint',
            'subscribe_depth': True,
            'subscribe_rgb': True,
            'subscribe_scan': True,
            'approx_sync': True,
            'map_always_update': True,
            'Grid/FromDepth': 'false',
            'Grid/MaxObstacleHeight': '1.5',
            'Grid/MaxGroundHeight': '0.15',
            'Grid/Sensor': '0',
            'Grid/RangeMax': '12.0',
            'Grid/RayTracing': 'true',
            'RGBD/ProximityBySpace': 'true',
            'RGBD/NeighborLinkRefining': 'true',
            'Reg/Strategy': '1',
            'Reg/Force3DoF': 'true',
            'Optimizer/Slam2D': 'true',
            'Rtabmap/DetectionRate': '2.0',
            'database_path': '~/.ros/rtabmap_sim.db',
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('rgb/image', '/oakd_camera/image'),
            ('depth/image', '/oakd_camera/depth_image'),
            ('rgb/camera_info', '/oakd_camera/camera_info'),
            ('odom', '/odom'),
            ('scan', '/scan'),
            ('grid_map', '/map')
        ]
    )

    # 6. AMCL & Map Server - For Localization
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    localization_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'localization_launch.py')
        ),
        condition=IfCondition(PythonExpression(["'", nav_mode, "' == 'localization'"])),
        launch_arguments={'map': map_yaml_file, 'use_sim_time': use_sim_time}.items()
    )

    # 7. Nav2 Process Manager
    nav2_manager_node = Node(
        package='odesc_hardware',
        executable='nav2_manager_node',
        name='nav2_manager_node',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription([
        set_env_action,
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher_node,
        gazebo_sim,
        cmd_vel_mux_node,
        rosbridge_server,
        web_server,
        spawn_entity,
        bridge_node,
        joy_node,
        teleop_twist_joy_node,
        rtabmap_node,
        localization_node,
        nav2_manager_node
    ])
