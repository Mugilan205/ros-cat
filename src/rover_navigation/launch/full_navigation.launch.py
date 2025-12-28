from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch full navigation stack: obstacle detection, RRT planner, and path executor.
    """
    
    pkg_share = FindPackageShare('rover_navigation')
    
    # Declare launch arguments
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='odom',
        description='Frame ID for planning and control'
    )
    
    lidar_frame_arg = DeclareLaunchArgument(
        'lidar_frame_id',
        default_value='laser',
        description='LiDAR frame ID'
    )
    
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Linear velocity (m/s)'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='1.0',
        description='Angular velocity (rad/s)'
    )
    
    # ============================================================
    # Node 1: Obstacle Detection
    # ============================================================
    obstacle_detection_node = Node(
        package='rover_navigation',
        executable='obstacle_detection',
        name='obstacle_detection',
        output='screen',
        parameters=[
            {'frame_id': LaunchConfiguration('lidar_frame_id')},
            {'obstacle_distance_threshold': 1.5},
            {'max_scan_range': 10.0},
            {'grid_size': 20.0},
            {'grid_resolution': 0.1},
        ],
    )
    
    # ============================================================
    # Node 2: RRT Planner
    # ============================================================
    rrt_planner_node = Node(
        package='rover_navigation',
        executable='rrt_planner',
        name='rrt_planner',
        output='screen',
        parameters=[
            {'frame_id': LaunchConfiguration('frame_id')},
            {'step_size': 0.5},
            {'max_iterations': 5000},
            {'obstacle_clearance': 0.3},
            {'robot_radius': 0.2},
            {'replan_frequency': 5.0},
            {'goal_tolerance': 0.3},
            {'workspace_size': 20.0},
        ],
    )
    
    # ============================================================
    # Node 3: Odometry (Wheel Encoder Integration)
    # ============================================================
    odometry_node = Node(
        package='rover_navigation',
        executable='odometry_node',
        name='odometry_node',
        output='screen',
        parameters=[
            {'wheel_radius': 0.05},
            {'wheel_base': 0.15},
            {'encoder_resolution': 20},
        ],
    )
    
    # ============================================================
    # Node 4: Path Executor
    # ============================================================
    path_executor_node = Node(
        package='rover_navigation',
        executable='path_executor',
        name='path_executor',
        output='screen',
        parameters=[
            {'linear_speed': LaunchConfiguration('linear_speed')},
            {'angular_speed': LaunchConfiguration('angular_speed')},
            {'lookahead_distance': 0.5},
            {'path_tolerance': 0.3},
        ],
    )
    
    return LaunchDescription([
        frame_id_arg,
        lidar_frame_arg,
        linear_speed_arg,
        angular_speed_arg,
        obstacle_detection_node,
        rrt_planner_node,
        odometry_node,
        path_executor_node,
    ])
