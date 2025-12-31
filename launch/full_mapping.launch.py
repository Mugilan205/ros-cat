from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # 1) RPLidar node (adjust to your driver launch/executable if different)
    ld.add_action(Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port': '/dev/ttyUSB1',
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'use_scan_mode': False,
            'force_scan_mode': False,
            'topic_name': 'scan',
            'auto_standby': False
        }]
    ))

    # 2) robot_state_publisher (URDF)
    ld.add_action(Node(package='robot_state_publisher', executable='robot_state_publisher',
                       output='screen', arguments=['/home/caterpillar/ros2_ws/urdf/your_robot.urdf']))

    # 3) EKF after 1.5s
    ld.add_action(TimerAction(period=1.5, actions=[
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=['/home/caterpillar/ros2_ws/config/ekf.yaml'], output='screen')
    ]))

    # 4) Odometry node (for odom → base_link TF)
    ld.add_action(TimerAction(period=1.0, actions=[
        Node(package='rover_navigation', executable='odometry_node', name='odometry_node',
             output='screen')
    ]))

    # 5) SLAM toolbox after 4.0s (needs transforms to be ready)
    ld.add_action(TimerAction(period=4.0, actions=[
        Node(package='slam_toolbox', executable='async_slam_toolbox_node', name='slam_toolbox',
             parameters=['/home/caterpillar/ros2_ws/config/slam_params.yaml'], output='screen')
    ]))

    # 6) RViz after 5.0s
    ld.add_action(TimerAction(period=5.0, actions=[
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen')
    ]))

    return ld
