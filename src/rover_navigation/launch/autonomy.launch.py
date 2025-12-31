from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():

    return LaunchDescription([

        # 1️⃣ Robot description (URDF)
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{
                "robot_description": ExecuteProcess(
                    cmd=["xacro", "/home/caterpillar/ros2_ws/src/simple_rover_description/urdf/simple_rover.urdf.xacro"],
                    output="screen"
                )
            }]
        ),

        # 2️⃣ RPLIDAR
        Node(
            package="rplidar_ros",
            executable="rplidar_composition",
            parameters=[
                {"serial_port": "/dev/ttyUSB0"},
                {"frame_id": "laser"}
            ],
            output="screen"
        ),

        # 3️⃣ ESP SERIAL ODOMETRY
        Node(
            package="rover_navigation",
            executable="esp_serial_odometry_node",
            output="screen"
        ),

        # 4️⃣ ODOMETRY RELAY + TF
        Node(
            package="rover_navigation",
            executable="odometry_node",
            output="screen"
        ),

        # 5️⃣ SLAM TOOLBOX (LIVE SLAM)
        ExecuteProcess(
            cmd=[
                "ros2", "launch", "slam_toolbox", "online_async_launch.py",
                "slam_params_file:=/home/caterpillar/ros2_ws/config/slam_params.yaml"
            ],
            output="screen"
        ),

        # 6️⃣ NAV2 (AUTONOMOUS NAVIGATION)
        ExecuteProcess(
            cmd=[
                "ros2", "launch", "nav2_bringup", "navigation_launch.py",
                "use_sim_time:=false",
                "slam:=true",
                "enable_docking:=false",
                "params_file:=/home/caterpillar/ros2_ws/config/nav2_params.yaml"
            ],
            output="screen"
        ),

        # 7️⃣ RVIZ (optional but recommended)
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", "/home/caterpillar/ros2_ws/config/rviz_autonomy.rviz"],
            output="screen"
        )
    ])
