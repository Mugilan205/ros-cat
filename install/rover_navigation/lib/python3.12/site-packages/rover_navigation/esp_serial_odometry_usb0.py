#!/usr/bin/env python3
"""ESP8266 serial-to-Odometry bridge for /dev/ttyUSB0 (ROS 2 Jazzy)."""

import math
import serial

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
from geometry_msgs.msg import TransformStamped


class EspSerialOdometryUSB0(Node):
    def __init__(self):
        super().__init__('esp_serial_odometry_usb0')
        self.port = '/dev/ttyUSB2'
        self.baudrate = 115200
        self.ser = None
        self.valid_count = 0

        self.odom_pub = self.create_publisher(Odometry, '/esp/odom', 10)

        # 20 Hz loop for polling serial data
        self.timer = self.create_timer(0.05, self.read_and_publish)

        self.get_logger().info(f"Starting ESP serial bridge on {self.port} @ {self.baudrate}")

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0)
            self.get_logger().info(f"Connected to {self.port}")
        except serial.SerialException as exc:
            self.get_logger().warn(f"Cannot open {self.port}: {exc}", throttle_duration_sec=5.0)
            self.ser = None

    def read_and_publish(self):
        # Ensure connection
        if self.ser is None or not self.ser.is_open:
            self.connect()
            return

        try:
            while self.ser.in_waiting > 0:
                raw = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not raw:
                    continue
                self.get_logger().debug(f"raw: {raw}")
                self.handle_line(raw)
        except serial.SerialException as exc:
            self.get_logger().warn(f"Serial disconnected: {exc}", throttle_duration_sec=5.0)
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        except Exception as exc:  # catch-all to avoid crashing loop
            self.get_logger().error(f"Unexpected error: {exc}")

    def handle_line(self, line: str):
        parts = line.split(',')
        if len(parts) != 6 or parts[0] != 'ODOM':
            self.get_logger().warn(f"Malformed line: {line}", throttle_duration_sec=2.0)
            return

        try:
            x = float(parts[1])
            y = float(parts[2])
            theta = float(parts[3])
            v = float(parts[4])
            w = float(parts[5])
        except ValueError:
            self.get_logger().warn(f"Non-numeric line: {line}", throttle_duration_sec=2.0)
            return

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = 0.0

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.pose.orientation.w = math.cos(theta / 2.0)

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        msg.pose.covariance = [0.0] * 36
        msg.twist.covariance = [0.0] * 36

        self.odom_pub.publish(msg)
        self.valid_count += 1
        self.get_logger().debug(
            f"published #{self.valid_count}: x={x:.3f}, y={y:.3f}, th={theta:.3f}, v={v:.3f}, w={w:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = EspSerialOdometryUSB0()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser:
            try:
                node.ser.close()
            except Exception:
                pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
