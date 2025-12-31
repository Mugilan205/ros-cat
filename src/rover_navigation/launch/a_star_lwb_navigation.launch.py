from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Launch A* global planner + LWB local planner navigation stack.
    """
    
    # Declare launch arguments
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='map',
        description='Global planning frame ID'
    )
    
    robot_frame_arg = DeclareLaunchArgument(
        'robot_frame',
        default_value='base_link',
        description='Robot base frame ID'
    )
    
    odom_frame_arg = DeclareLaunchArgument(
        'odom_frame',
        default_value='odom',
        description='Odometry frame ID'
    )
    
    # A* Global Planner
    a_star_planner_node = Node(
        package='rover_navigation',
        executable='a_star_planner',
        name='a_star_planner',
        output='screen',
        parameters=[
            {'map_frame': LaunchConfiguration('frame_id')},
            {'robot_frame': LaunchConfiguration('robot_frame')},
            {'inflation_radius': 0.5},
            {'allow_unknown': False},
            {'plan_frequency': 1.0},
            {'tolerance_xy': 0.2},
        ],
    )
    
    # LWB Local Planner
    lwb_planner_node = Node(
        package='rover_navigation',
        executable='lwb_planner',
        name='lwb_planner',
        output='screen',
        parameters=[
            {'robot_frame': LaunchConfiguration('robot_frame')},
            {'odom_frame': LaunchConfiguration('odom_frame')},
            {'max_vel_x': 0.5},
            {'min_vel_x': 0.0},
            {'max_vel_theta': 1.0},
            {'acc_lim_x': 0.5},
            {'acc_lim_theta': 1.0},
            {'sim_time': 1.5},
            {'sim_granularity': 0.1},
            {'vx_samples': 20},
            {'vtheta_samples': 40},
            {'robot_radius': 0.2},
            {'local_costmap_size': 4.0},
            {'local_costmap_resolution': 0.05},
            {'inflation_radius': 0.3},
            {'path_distance_bias': 32.0},
            {'goal_distance_bias': 24.0},
            {'obstacle_cost_bias': 1.0},
            {'forward_point_distance': 0.325},
        ],
    )
    
    # Planner Manager
    planner_manager_node = Node(
        package='rover_navigation',
        executable='planner_manager',
        name='planner_manager',
        output='screen',
        parameters=[
            {'goal_tolerance': 0.3},
            {'replan_distance_threshold': 0.5},
            {'monitor_frequency': 5.0},
        ],
    )
    
    # Navigation Dashboard
    dashboard_node = Node(
        package='rover_navigation',
        executable='navigation_dashboard',
        name='navigation_dashboard',
        output='screen',
        parameters=[
            {'publish_frequency': 10.0},
            {'marker_frame': LaunchConfiguration('frame_id')},
        ],
    )
    
    return LaunchDescription([
        frame_id_arg,
        robot_frame_arg,
        odom_frame_arg,
        a_star_planner_node,
        lwb_planner_node,
        planner_manager_node,
        dashboard_node,
    ])



















