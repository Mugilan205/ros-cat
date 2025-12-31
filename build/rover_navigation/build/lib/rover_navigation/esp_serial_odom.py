#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import serial
import math
import time

class ESPSerialOdom(Node):
    def __init__(self):
        super().__init__('esp_serial_odom')

        self.port = '/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTB6SPL3-if00-port0'
        self.baud = 115200

        self.publisher = self.create_publisher(Odometry, '/esp/odom', 10)

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)
            self.get_logger().info(f"Connected to {self.port}")
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
            return

        self.timer = self.create_timer(0.05, self.read_serial)  # 20 Hz

    def yaw_to_quat(self, yaw):
        q = Quaternion()
        q.w = math.cos(yaw / 2.0)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        return q

    def read_serial(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line.startswith("ODOM"):
                return

            self.get_logger().info(f"RAW: {line}")

            _, x, y, theta, lin, ang = line.split(",")

            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"

            msg.pose.pose.position.x = float(x)
            msg.pose.pose.position.y = float(y)
            msg.pose.pose.orientation = self.yaw_to_quat(float(theta))

            msg.twist.twist.linear.x = float(lin)
            msg.twist.twist.angular.z = float(ang)

            self.publisher.publish(msg)

        except Exception as e:
            self.get_logger().warn(f"Parse error: {e}")

def main():
    rclpy.init()
    node = ESPSerialOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
