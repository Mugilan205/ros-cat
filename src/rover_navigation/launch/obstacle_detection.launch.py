from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch obstacle detection node with parameters."""
    
    # Declare arguments
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Frame ID for obstacle detection'
    )
    
    obstacle_threshold_arg = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='1.5',
        description='Distance threshold for obstacle detection (meters)'
    )
    
    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='10.0',
        description='Maximum scan range to process (meters)'
    )
    
    # Obstacle Detection Node
    obstacle_detection_node = Node(
        package='rover_navigation',
        executable='obstacle_detection',
        name='obstacle_detection',
        output='screen',
        parameters=[
            {'frame_id': LaunchConfiguration('frame_id')},
            {'obstacle_distance_threshold': LaunchConfiguration('obstacle_threshold')},
            {'max_scan_range': LaunchConfiguration('max_range')},
            {'grid_size': 20.0},
            {'grid_resolution': 0.1},
        ],
    )
    
    return LaunchDescription([
        frame_id_arg,
        obstacle_threshold_arg,
        max_range_arg,
        obstacle_detection_node,
    ])
