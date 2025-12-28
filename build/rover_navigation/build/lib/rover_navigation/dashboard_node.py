"""
Navigation Dashboard Node
========================

Lightweight dashboard for development and debugging of navigation stack.

Visualizes:
- Robot pose (x, y, yaw)
- EKF state
- Global path (A*)
- Local path (LWB)
- Current velocity commands
- Obstacle points
- Planner state

Topics:
  Subscribed:
    - /odom: Robot odometry
    - /global_path: Global path from A* planner
    - /local_trajectory: Local trajectory from LWB planner
    - /cmd_vel: Current velocity commands
    - /scan: Laser scan for obstacle visualization
    - /planner/debug/state: Planner state
  Published:
    - /planner/debug/costs: Trajectory costs (std_msgs/Float32MultiArray)
    - /planner/debug/robot_info: Robot state info (custom message)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32MultiArray, Header
from visualization_msgs.msg import Marker, MarkerArray
import math


class DashboardNode(Node):
    """
    Dashboard node for navigation debugging and visualization.
    """
    
    def __init__(self):
        super().__init__('navigation_dashboard')
        
        # Parameters
        self.declare_parameter('publish_frequency', 10.0)
        self.declare_parameter('marker_frame', 'map')
        
        pub_freq = self.get_parameter('publish_frequency').value
        self.marker_frame = self.get_parameter('marker_frame').value
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            PoseStamped,
            '/odom',
            self.odom_callback,
            sensor_qos
        )
        
        self.global_path_sub = self.create_subscription(
            Path,
            '/global_path',
            self.global_path_callback,
            10
        )
        
        self.local_path_sub = self.create_subscription(
            Path,
            '/local_trajectory',
            self.local_path_callback,
            10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos
        )
        
        self.state_sub = self.create_subscription(
            String,
            '/planner/debug/state',
            self.state_callback,
            10
        )
        
        # Publications
        self.obstacle_markers_pub = self.create_publisher(
            MarkerArray,
            '/planner/debug/obstacles',
            10
        )
        
        self.robot_pose_marker_pub = self.create_publisher(
            Marker,
            '/planner/debug/robot_pose',
            10
        )
        
        # State variables
        self.current_pose = None
        self.current_vel = Twist()
        self.global_path = None
        self.local_path = None
        self.current_scan = None
        self.planner_state = 'UNKNOWN'
        
        # Timer for periodic updates
        self.update_timer = self.create_timer(1.0 / pub_freq, self.update_callback)
        
        self.get_logger().info('Navigation Dashboard Node initialized')
        self.get_logger().info(f'Publishing at {pub_freq} Hz')
    
    def odom_callback(self, msg: PoseStamped):
        """Receive robot odometry."""
        self.current_pose = msg
    
    def global_path_callback(self, msg: Path):
        """Receive global path."""
        self.global_path = msg
    
    def local_path_callback(self, msg: Path):
        """Receive local trajectory."""
        self.local_path = msg
    
    def cmd_vel_callback(self, msg: Twist):
        """Receive velocity commands."""
        self.current_vel = msg
    
    def scan_callback(self, msg: LaserScan):
        """Receive laser scan."""
        self.current_scan = msg
    
    def state_callback(self, msg: String):
        """Receive planner state."""
        self.planner_state = msg.data
    
    def update_callback(self):
        """Periodic update - publish markers and debug info."""
        # Publish robot pose marker
        if self.current_pose:
            self.publish_robot_pose_marker()
        
        # Publish obstacle markers from scan
        if self.current_scan:
            self.publish_obstacle_markers()
        
        # Log current state
        self.log_status()
    
    def publish_robot_pose_marker(self):
        """Publish marker for robot pose and orientation."""
        marker = Marker()
        marker.header.frame_id = self.current_pose.header.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.pose = self.current_pose.pose
        
        # Arrow size
        marker.scale.x = 0.3  # Length
        marker.scale.y = 0.1  # Width
        marker.scale.z = 0.1  # Height
        
        # Color (green)
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        
        self.robot_pose_marker_pub.publish(marker)
    
    def publish_obstacle_markers(self):
        """Publish markers for obstacles from laser scan."""
        marker_array = MarkerArray()
        
        # Delete all previous markers
        delete_marker = Marker()
        delete_marker.header.frame_id = self.current_scan.header.frame_id
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)
        
        # Add obstacle points
        marker_id = 0
        obstacle_threshold = 1.5  # meters
        
        for i, range_val in enumerate(self.current_scan.ranges):
            if (math.isnan(range_val) or math.isinf(range_val) or
                range_val < self.current_scan.range_min or 
                range_val > self.current_scan.range_max):
                continue
            
            if range_val < obstacle_threshold:
                # Create point marker
                angle = self.current_scan.angle_min + i * self.current_scan.angle_increment
                x = range_val * math.cos(angle)
                y = range_val * math.sin(angle)
                
                marker = Marker()
                marker.header.frame_id = self.current_scan.header.frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = marker_id
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = x
                marker.pose.position.y = y
                marker.pose.position.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.1
                marker.scale.y = 0.1
                marker.scale.z = 0.1
                
                # Color (red for obstacles)
                marker.color.a = 0.8
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                
                marker_array.markers.append(marker)
                marker_id += 1
        
        self.obstacle_markers_pub.publish(marker_array)
    
    def log_status(self):
        """Log current navigation status."""
        if self.current_pose:
            x = self.current_pose.pose.position.x
            y = self.current_pose.pose.position.y
            yaw = self.quaternion_to_yaw(self.current_pose.pose.orientation)
            
            self.get_logger().debug(
                f'Robot: ({x:.2f}, {y:.2f}, {math.degrees(yaw):.1f}°), '
                f'Vel: ({self.current_vel.linear.x:.2f}, {self.current_vel.angular.z:.2f}), '
                f'State: {self.planner_state}'
            )
    
    @staticmethod
    def quaternion_to_yaw(quat) -> float:
        """Convert quaternion to yaw angle."""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


