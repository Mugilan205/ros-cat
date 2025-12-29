#!/usr/bin/env python3
"""
ESP Serial Odometry Node
Bridge between ESP32 Serial (ASCII) and ROS 2 Odometry.
Parses: ODOM,x,y,theta,v,w
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import serial
import math
import time

class EspSerialOdometryNode(Node):
    def __init__(self):
        super().__init__('esp_serial_odometry_node')
        
        # Configuration
        self.port = '/dev/ttyUSB1'
        self.baudrate = 115200
        self.valid_msg_count = 0
        self.ser = None
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/esp/odom', 10)
        
        # Timer for serial reading (100Hz to ensure we don't miss data)
        self.timer = self.create_timer(0.01, self.read_serial_callback)
        
        # Timer for connection check (every 2 seconds)
        self.conn_timer = self.create_timer(2.0, self.check_connection)
        
        self.get_logger().info(f"ESP Serial Odometry Node started. Target: {self.port} @ {self.baudrate}")

    def check_connection(self):
        """Ensure serial port is open, retry if not."""
        if self.ser is None or not self.ser.is_open:
            try:
                self.ser = serial.Serial(self.port, self.baudrate, timeout=0, write_timeout=0)
                self.get_logger().info(f"Successfully connected to {self.port}")
            except serial.SerialException:
                self.get_logger().warn(f"Failed to open {self.port}. Retrying...", throttle_duration_sec=10.0)
                self.ser = None

    def read_serial_callback(self):
        """Non-blocking read from serial port."""
        if self.ser is None or not self.ser.is_open:
            return

        try:
            while self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self.parse_and_publish(line)
        except serial.SerialException as e:
            self.get_logger().error(f"Serial connection lost: {e}")
            self.ser.close()
            self.ser = None
        except Exception as e:
            self.get_logger().error(f"Unexpected error reading serial: {e}")

    def parse_and_publish(self, line):
        """Parse ASCII line: ODOM,x,y,theta,v,w"""
        parts = line.split(',')
        
        # Validation: Start with ODOM and have 6 fields
        if len(parts) != 6 or parts[0] != 'ODOM':
            self.get_logger().warn(f"Malformed ESP line: {line}", throttle_duration_sec=5.0)
            return

        try:
            # Extract values
            x = float(parts[1])
            y = float(parts[2])
            theta = float(parts[3])
            v = float(parts[4])
            w = float(parts[5])
            
            # Create Odometry message
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"

            # Pose: Position
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = 0.0

            # Pose: Orientation (Yaw to Quaternion)
            msg.pose.pose.orientation.x = 0.0
            msg.pose.pose.orientation.y = 0.0
            msg.pose.pose.orientation.z = math.sin(theta / 2.0)
            msg.pose.pose.orientation.w = math.cos(theta / 2.0)

            # Twist: Velocity
            msg.twist.twist.linear.x = v
            msg.twist.twist.angular.z = w
            
            # Initialize covariances to zero (already zero by default, but explicit for clarity)
            msg.pose.covariance = [0.0] * 36
            msg.twist.covariance = [0.0] * 36

            # Publish
            self.odom_pub.publish(msg)
            
            # Logging
            self.valid_msg_count += 1
            if self.valid_msg_count % 50 == 0:
                self.get_logger().info(f"Published {self.valid_msg_count} messages. Latest: x={x:.3f}, y={y:.3f}, th={theta:.3f}")

        except ValueError:
            self.get_logger().warn(f"Invalid numeric data from ESP: {line}", throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = EspSerialOdometryNode()
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

