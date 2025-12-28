from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch RRT planner node with parameters."""
    
    # Declare arguments
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='odom',
        description='Frame ID for path planning'
    )
    
    step_size_arg = DeclareLaunchArgument(
        'step_size',
        default_value='0.5',
        description='RRT step size (meters)'
    )
    
    max_iterations_arg = DeclareLaunchArgument(
        'max_iterations',
        default_value='5000',
        description='Maximum RRT planning iterations'
    )
    
    clearance_arg = DeclareLaunchArgument(
        'obstacle_clearance',
        default_value='0.3',
        description='Safety clearance around obstacles (meters)'
    )
    
    robot_radius_arg = DeclareLaunchArgument(
        'robot_radius',
        default_value='0.2',
        description='Robot collision radius (meters)'
    )
    
    # RRT Planner Node
    rrt_planner_node = Node(
        package='rover_navigation',
        executable='rrt_planner',
        name='rrt_planner',
        output='screen',
        parameters=[
            {'frame_id': LaunchConfiguration('frame_id')},
            {'step_size': LaunchConfiguration('step_size')},
            {'max_iterations': LaunchConfiguration('max_iterations')},
            {'obstacle_clearance': LaunchConfiguration('obstacle_clearance')},
            {'robot_radius': LaunchConfiguration('robot_radius')},
            {'replan_frequency': 5.0},
            {'goal_tolerance': 0.3},
            {'workspace_size': 20.0},
        ],
    )
    
    return LaunchDescription([
        frame_id_arg,
        step_size_arg,
        max_iterations_arg,
        clearance_arg,
        robot_radius_arg,
        rrt_planner_node,
    ])
