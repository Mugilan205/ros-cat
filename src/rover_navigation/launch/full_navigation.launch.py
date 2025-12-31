from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ---------------------------
    # Launch Arguments
    # ---------------------------
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    start_custom_planner_arg = DeclareLaunchArgument(
        'start_custom_planner',
        default_value='false',
        description='Enable custom RRT planner stack (disabled by default)'
    )

    # ---------------------------
    # Robot Description (URDF)
    # ---------------------------
    # Using absolute path as simple_rover_description is not a ROS package
    urdf_path = '/home/caterpillar/ros2_ws/src/simple_rover_description/urdf/simple_rover.urdf.xacro'
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': Command(['xacro', urdf_path])
        }]
    )

    # ---------------------------
    # ESP Serial Odometry (Primary Odom Source)
    # ---------------------------
    # Publishes directly to /odom and broadcasts TF: odom -> base_link
    esp_odom_node = Node(
        package='rover_navigation',
        executable='esp_serial_odometry_node',
        name='esp_serial_to_odom',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'port': '/dev/ttyUSB0',
            'baud': 115200,
        }]
    )

    # ---------------------------
    # RPLIDAR
    # ---------------------------
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 115200,
            'frame_id': 'laser_frame',
            'inverted': False,
            'angle_compensate': True,
            'scan_frequency': 10.0,
        }]
    )

    # ---------------------------
    # SLAM Toolbox (Online Async)
    # ---------------------------
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_params_file': PathJoinSubstitution([
                FindPackageShare('rover_navigation'),
                'config',
                'slam_params.yaml'
            ])
        }.items()
    )

    # ---------------------------
    # Nav2 Bringup (Primary Navigation)
    # ---------------------------
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
            'params_file': PathJoinSubstitution([
                FindPackageShare('rover_navigation'),
                'config',
                'nav2_params.yaml'
            ])
        }.items()
    )

    # ---------------------------
    # Custom Planner Stack (Optional)
    # ---------------------------
    custom_planner_stack = GroupAction(
        condition=IfCondition(LaunchConfiguration('start_custom_planner')),
        actions=[
            Node(
                package='rover_navigation',
                executable='obstacle_detection',
                name='obstacle_detection',
                output='screen'
            ),
            Node(
                package='rover_navigation',
                executable='rrt_planner',
                name='rrt_planner',
                output='screen'
            ),
            Node(
                package='rover_navigation',
                executable='path_executor',
                name='path_executor',
                output='screen'
            ),
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        start_custom_planner_arg,

        robot_state_publisher_node,
        esp_odom_node,
        rplidar_node,
        slam_toolbox,
        nav2_bringup,
        custom_planner_stack
    ])
