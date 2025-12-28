#!/usr/bin/env python3
"""
Wheel Encoder Odometry Node
Publishes position and orientation from wheel encoders to /odom topic.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, Quaternion
from nav_msgs.msg import Odometry
import math


class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry_node")
        
        # Odometry parameters
        self.declare_parameter("wheel_radius", 0.05)  # 5cm wheels
        self.declare_parameter("wheel_base", 0.15)    # 15cm distance between wheels
        self.declare_parameter("encoder_resolution", 20)  # 20 ticks per revolution
        
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_base = self.get_parameter("wheel_base").value
        self.encoder_resolution = self.get_parameter("encoder_resolution").value
        
        # Current odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0  # heading in radians
        
        # Velocity state
        self.linear_x = 0.0
        self.angular_z = 0.0
        
        # Create publishers
        self.odom_pub = self.create_publisher(PoseStamped, "/odom", 10)
        
        # Subscribe to command velocity to integrate odometry
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )
        
        # Timer for publishing odometry
        self.timer = self.create_timer(0.05, self.publish_odom)  # 20Hz
        
        self.get_logger().info(
            f"Odometry Node Started\n"
            f"  Wheel Radius: {self.wheel_radius} m\n"
            f"  Wheel Base: {self.wheel_base} m\n"
            f"  Encoder Resolution: {self.encoder_resolution} ticks/rev"
        )

    def cmd_vel_callback(self, msg):
        """Store current velocity commands for odometry integration."""
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z
        
        # Integrate odometry
        dt = 0.05  # Assuming 20Hz update rate
        
        if abs(self.angular_z) > 0.001:
            # Curved path
            radius = self.linear_x / self.angular_z
            delta_theta = self.angular_z * dt
            
            self.x += radius * (math.sin(self.theta + delta_theta) - math.sin(self.theta))
            self.y += radius * (math.cos(self.theta) - math.cos(self.theta + delta_theta))
            self.theta += delta_theta
        else:
            # Straight line
            self.x += self.linear_x * math.cos(self.theta) * dt
            self.y += self.linear_x * math.sin(self.theta) * dt

    def publish_odom(self):
        """Publish current odometry state."""
        # Normalize theta to [-π, π]
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi
        
        # Create PoseStamped message
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        
        # Set position
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = 0.0
        
        # Convert theta to quaternion
        q = self.euler_to_quaternion(0, 0, self.theta)
        msg.pose.orientation = q
        
        # Publish
        self.odom_pub.publish(msg)
        
        # Log periodically
        self.get_logger().debug(
            f"Odometry: X={self.x:.3f}m, Y={self.y:.3f}m, θ={math.degrees(self.theta):.1f}°"
        )

    @staticmethod
    def euler_to_quaternion(roll, pitch, yaw):
        """Convert Euler angles to Quaternion."""
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        
        return q


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
