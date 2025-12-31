#!/usr/bin/env python3
"""
ESP8266 Serial Bridge Node for ROS 2 Jazzy
Bridges ESP8266 Serial Odometry (ODOM,x,y,theta,v,w) to ROS 2 /esp/odom topic.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import serial
import math
import time

class ESP8266SerialBridge(Node):
    def __init__(self):
        super().__init__('esp8266_serial_bridge')
        
        # Parameters
        self.port = '/dev/ttyUSB0'
        self.baudrate = 115200
        self.publish_rate = 20.0  # Hz
        
        # State
        self.ser = None
        self.latest_odom_msg = None
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/esp/odom', 10)
        
        # Timers
        self.create_timer(1.0 / self.publish_rate, self.publish_callback)
        self.create_timer(0.005, self.read_serial_callback)  # Read at 200Hz to keep buffer clear
        self.create_timer(2.0, self.check_connection)        # Reconnect timer
        
        self.get_logger().info(f"ESP8266 Serial Bridge started on {self.port} at {self.baudrate} baud")

    def check_connection(self):
        """Check and maintain serial connection."""
        if self.ser is None or not self.ser.is_open:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0)
                self.get_logger().info(f"Connected to {self.port}")
            except serial.SerialException as e:
                self.get_logger().warn(f"Could not open {self.port}: {e}. Retrying in 2s...", throttle_duration_sec=10.0)
                self.ser = None

    def read_serial_callback(self):
        """Read and parse serial data."""
        if self.ser is None or not self.ser.is_open:
            return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line:
                    continue
                
                self.get_logger().debug(f"Raw Serial: {line}")
                
                parts = line.split(',')
                if len(parts) == 6 and parts[0] == 'ODOM':
                    try:
                        # Parse values
                        x = float(parts[1])
                        y = float(parts[2])
                        theta = float(parts[3])
                        v = float(parts[4])
                        w = float(parts[5])
                        
                        # Create message
                        msg = Odometry()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = 'odom'
                        msg.child_frame_id = 'base_link'
                        
                        # Pose
                        msg.pose.pose.position.x = x
                        msg.pose.pose.position.y = y
                        msg.pose.pose.position.z = 0.0
                        
                        # Yaw to Quaternion
                        msg.pose.pose.orientation = self.yaw_to_quaternion(theta)
                        
                        # Twist
                        msg.twist.twist.linear.x = v
                        msg.twist.twist.angular.z = w
                        
                        # Store as latest
                        self.latest_odom_msg = msg
                        
                    except ValueError:
                        self.get_logger().warn(f"Invalid numeric data: {line}", throttle_duration_sec=5.0)
                else:
                    if parts[0] == 'ODOM':
                        self.get_logger().warn(f"Malformed ODOM line (expected 6 fields): {line}", throttle_duration_sec=5.0)

        except serial.SerialException as e:
            self.get_logger().error(f"Serial error: {e}")
            self.ser.close()
            self.ser = None

    def publish_callback(self):
        """Publish the latest odometry message at fixed rate."""
        if self.latest_odom_msg is not None:
            self.odom_pub.publish(self.latest_odom_msg)
            self.get_logger().debug("Published odometry to /esp/odom")
            # We keep the latest message to continue publishing even if serial drops briefly, 
            # but in a real robot you might want to null it if data is too old.

    def yaw_to_quaternion(self, yaw):
        """Convert yaw angle to geometry_msgs/Quaternion."""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q

def main(args=None):
    rclpy.init(args=args)
    node = ESP8266SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
