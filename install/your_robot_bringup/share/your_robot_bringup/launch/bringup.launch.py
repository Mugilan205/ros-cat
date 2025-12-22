from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # RPLIDAR Node
    rplidar_node = Node(
        package="rplidar_ros",
        executable="rplidar_composition",
        parameters=[{
            "serial_port": "/dev/ttyUSB0",
            "serial_baudrate": 115200,
            "frame_id": "laser",
            "scan_mode": "Standard"
        }],
        output="screen"
    )

    # BTS7960 Motor Controller Node
    motor_node = Node(
        package="bts_motor_controller",
        executable="motor_node",
        output="screen"
    )

    return LaunchDescription([
        rplidar_node,
        motor_node
    ])

