#!/usr/bin/env python3
"""
Wheel Encoder Odometry Node
Publishes position and orientation from ESP odometry to /odom topic.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import math


class OdometryNode(Node):
    def __init__(self):
        super().__init__("odometry_node")
        
        # ESP odometry input topic
        self.declare_parameter("esp_odom_topic", "/esp/odom")
        esp_odom_topic = self.get_parameter("esp_odom_topic").value
        
        # Store latest ESP odometry data
        self.latest_esp_odom = None
        self.esp_odom_received = False
        
        # Create subscriber for ESP odometry
        self.esp_odom_sub = self.create_subscription(
            Odometry,
            esp_odom_topic,
            self.esp_odom_callback,
            10
        )
        
        # Create publishers
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        
        # Timer for publishing odometry (same as before: 20Hz)
        self.timer = self.create_timer(0.05, self.publish_odom)  # 20Hz
        
        self.get_logger().info(
            f"Odometry Node Started\n"
            f"  Subscribing to ESP odometry: {esp_odom_topic}\n"
            f"  Publishing to: /odom at 20 Hz"
        )

    def esp_odom_callback(self, msg):
        """Store latest ESP odometry data."""
        self.latest_esp_odom = msg
        self.esp_odom_received = True

    def publish_odom(self):
        """Publish current odometry state (same structure as before)."""
        # Wait for ESP odometry before publishing
        if not self.esp_odom_received or self.latest_esp_odom is None:
            self.get_logger().warn("Waiting for ESP odometry data on /esp/odom...", throttle_duration_sec=2.0)
            return
        
        # Create Odometry message (required by EKF) - same structure as before
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"  # Critical: EKF needs this!
        
        # Copy position from ESP
        msg.pose.pose.position.x = self.latest_esp_odom.pose.pose.position.x
        msg.pose.pose.position.y = self.latest_esp_odom.pose.pose.position.y
        msg.pose.pose.position.z = self.latest_esp_odom.pose.pose.position.z
        
        # Copy orientation from ESP
        msg.pose.pose.orientation.x = self.latest_esp_odom.pose.pose.orientation.x
        msg.pose.pose.orientation.y = self.latest_esp_odom.pose.pose.orientation.y
        msg.pose.pose.orientation.z = self.latest_esp_odom.pose.pose.orientation.z
        msg.pose.pose.orientation.w = self.latest_esp_odom.pose.pose.orientation.w
        
        # Copy pose covariance if ESP provides it
        if len(self.latest_esp_odom.pose.covariance) == 36:
            msg.pose.covariance = self.latest_esp_odom.pose.covariance
        
        # Copy twist from ESP (if provided)
        msg.twist.twist.linear.x = self.latest_esp_odom.twist.twist.linear.x
        msg.twist.twist.linear.y = self.latest_esp_odom.twist.twist.linear.y
        msg.twist.twist.linear.z = self.latest_esp_odom.twist.twist.linear.z
        msg.twist.twist.angular.x = self.latest_esp_odom.twist.twist.angular.x
        msg.twist.twist.angular.y = self.latest_esp_odom.twist.twist.angular.y
        msg.twist.twist.angular.z = self.latest_esp_odom.twist.twist.angular.z
        
        # Copy twist covariance if ESP provides it
        if len(self.latest_esp_odom.twist.covariance) == 36:
            msg.twist.covariance = self.latest_esp_odom.twist.covariance
        
        # Publish
        self.odom_pub.publish(msg)
        
        # Log periodically (same format as before)
        self.get_logger().debug(
            f"Odometry: X={msg.pose.pose.position.x:.3f}m, "
            f"Y={msg.pose.pose.position.y:.3f}m, "
            f"θ={math.degrees(self.quaternion_to_yaw(msg.pose.pose.orientation)):.1f}°"
        )

    @staticmethod
    def quaternion_to_yaw(quat):
        """Convert quaternion to yaw angle (for logging)."""
        # Extract yaw from quaternion
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw


def main(args=None):
    rclpy.init(args=args)
    node = OdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
