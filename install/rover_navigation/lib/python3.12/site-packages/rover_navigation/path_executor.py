"""
Path Executor with Velocity Control
====================================

This node executes the planned path and controls the rover's velocity.

Functionality:
- Subscribes to planned path (/path)
- Tracks progress along path
- Computes velocity commands for differential drive
- Implements pure pursuit algorithm for path tracking
- Publishes velocity commands on /cmd_vel

Topics:
  Subscribed:
    - /path: Planned path from RRT planner
    - /odom: Current robot odometry
  Published:
    - /cmd_vel: Velocity commands (differential drive)

Parameters:
  - linear_speed: Forward velocity (default: 0.5 m/s)
  - angular_speed: Max rotation speed (default: 1.0 rad/s)
  - lookahead_distance: Pure pursuit distance (default: 0.5m)
  - path_tolerance: Distance to waypoint (default: 0.3m)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
import math
import numpy as np


class PathExecutorNode(Node):
    """
    Executes planned paths and controls rover velocity.
    """

    def __init__(self):
        super().__init__('path_executor')
        
        # Parameters
        self.declare_parameter('linear_speed', 0.5)
        self.declare_parameter('angular_speed', 1.0)
        self.declare_parameter('lookahead_distance', 0.5)
        self.declare_parameter('path_tolerance', 0.3)
        
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.lookahead_dist = self.get_parameter('lookahead_distance').value
        self.path_tolerance = self.get_parameter('path_tolerance').value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriptions
        self.path_sub = self.create_subscription(
            Path,
            '/path',
            self.path_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            PoseStamped,
            '/odom',
            self.odom_callback,
            sensor_qos
        )
        
        # Publications
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # State variables
        self.current_path = None
        self.current_pose = None
        self.current_waypoint_idx = 0
        
        # Execution timer (10 Hz)
        self.execution_timer = self.create_timer(0.1, self.execution_callback)
        
        self.get_logger().info('Path Executor Node initialized')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Lookahead distance: {self.lookahead_dist} m')

    def path_callback(self, msg: Path):
        """
        Receive new path and reset tracking.
        
        Args:
            msg: Path message with waypoints
        """
        if len(msg.poses) > 0:
            self.current_path = msg
            self.current_waypoint_idx = 0
            self.get_logger().info(f'Received path with {len(msg.poses)} waypoints')
        else:
            self.current_path = None
            self.stop_robot()

    def odom_callback(self, msg: PoseStamped):
        """
        Update current robot position.
        
        Args:
            msg: Current odometry pose
        """
        self.current_pose = msg

    def execution_callback(self):
        """
        Main execution loop - compute and publish velocity commands.
        
        Path Tracking Algorithm:
        ========================
        1. If no path or no pose, stop
        2. Get current waypoint index
        3. Find target point using lookahead distance
        4. Compute desired heading toward target
        5. Compute linear and angular velocities
        6. Publish velocity command
        
        Waypoint Progression:
        - Check distance to current waypoint
        - If close enough, move to next waypoint
        - If at final waypoint, stop
        """
        if self.current_path is None or self.current_pose is None:
            self.stop_robot()
            return
        
        # Update waypoint index based on distance
        self.update_waypoint_index()
        
        # Check if path execution is complete
        if self.current_waypoint_idx >= len(self.current_path.poses):
            self.get_logger().info('Path execution complete')
            self.stop_robot()
            return
        
        # Find lookahead point and compute velocity
        target_point = self.find_lookahead_point()
        if target_point is None:
            self.stop_robot()
            return
        
        # Compute velocity command
        cmd_vel = self.compute_velocity_command(target_point)
        self.cmd_vel_pub.publish(cmd_vel)

    def update_waypoint_index(self):
        """
        Update waypoint index based on proximity to current waypoint.
        
        Logic:
        - Calculate distance to current waypoint
        - If distance <= path_tolerance, advance to next waypoint
        - Skip intermediate waypoints if within lookahead distance
        """
        if self.current_path is None:
            return
        
        while self.current_waypoint_idx < len(self.current_path.poses):
            waypoint = self.current_path.poses[self.current_waypoint_idx]
            dist = self.distance_to_pose(self.current_pose, waypoint)
            
            if dist <= self.path_tolerance:
                self.current_waypoint_idx += 1
                if self.current_waypoint_idx < len(self.current_path.poses):
                    self.get_logger().debug(
                        f'Advanced to waypoint {self.current_waypoint_idx}'
                    )
            else:
                break

    def find_lookahead_point(self) -> dict:
        """
        Find target point for pure pursuit control.
        
        Pure Pursuit Algorithm:
        =======================
        1. Start from current waypoint index
        2. Calculate cumulative distance along path
        3. Find point at lookahead_distance ahead
        4. If lookahead distance beyond path end, use goal point
        
        Returns:
            Dictionary with 'x', 'y' keys or None if invalid
        """
        if self.current_waypoint_idx >= len(self.current_path.poses):
            return None
        
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        
        # Simple approach: use waypoint at lookahead distance
        lookahead_idx = self.current_waypoint_idx
        cumulative_dist = 0.0
        
        for i in range(self.current_waypoint_idx, len(self.current_path.poses)):
            waypoint = self.current_path.poses[i]
            wp_x = waypoint.pose.position.x
            wp_y = waypoint.pose.position.y
            
            dist_to_wp = math.sqrt((wp_x - current_x) ** 2 + (wp_y - current_y) ** 2)
            
            if cumulative_dist + dist_to_wp >= self.lookahead_dist:
                lookahead_idx = i
                break
        
        target_waypoint = self.current_path.poses[min(
            lookahead_idx,
            len(self.current_path.poses) - 1
        )]
        
        return {
            'x': target_waypoint.pose.position.x,
            'y': target_waypoint.pose.position.y
        }

    def compute_velocity_command(self, target_point: dict) -> Twist:
        """
        Compute differential drive velocity command.
        
        Control Strategy:
        =================
        1. Calculate angle to target point
        2. Calculate heading error (angle from robot orientation to target)
        3. Use proportional control for angular velocity
        4. Linear velocity depends on alignment:
           - Full speed when aligned
           - Reduced speed when turning sharply
           - Zero when beyond max angular velocity
        
        Args:
            target_point: Dictionary with 'x', 'y' keys
            
        Returns:
            Twist message with linear and angular velocity
        """
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        
        # Calculate angle to target
        dx = target_point['x'] - current_x
        dy = target_point['y'] - current_y
        target_angle = math.atan2(dy, dx)
        
        # Get robot's current orientation (from quaternion)
        robot_angle = self.get_yaw_from_quaternion(
            self.current_pose.pose.orientation
        )
        
        # Calculate heading error
        angle_error = self.normalize_angle(target_angle - robot_angle)
        
        # Command velocities
        cmd_vel = Twist()
        
        # Angular velocity (proportional control)
        # Clamp to max angular speed
        cmd_vel.angular.z = np.clip(
            angle_error * 2.0,  # Proportional gain
            -self.angular_speed,
            self.angular_speed
        )
        
        # Linear velocity (reduced when turning)
        # Full speed when aligned, reduced when turning
        alignment = math.cos(angle_error)
        cmd_vel.linear.x = self.linear_speed * max(0, alignment)
        
        return cmd_vel

    def stop_robot(self):
        """Stop the rover by publishing zero velocity."""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)

    @staticmethod
    def distance_to_pose(pose1: PoseStamped, pose2: PoseStamped) -> float:
        """Calculate distance between two poses."""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx * dx + dy * dy)

    @staticmethod
    def get_yaw_from_quaternion(quat):
        """
        Convert quaternion to yaw angle.
        
        Args:
            quat: Quaternion with x, y, z, w components
            
        Returns:
            Yaw angle in radians
        """
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """
        Normalize angle to [-pi, pi] range.
        
        Args:
            angle: Angle in radians
            
        Returns:
            Normalized angle
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = PathExecutorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
