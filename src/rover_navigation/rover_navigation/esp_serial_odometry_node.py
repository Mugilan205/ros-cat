#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import serial
import math
import time

class ESPSerialToOdom(Node):

    def __init__(self):
        super().__init__('esp_serial_to_odom')

        self.port = '/dev/ttyUSB1'
        self.baud = 115200

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            time.sleep(2)
        except Exception as e:
            self.get_logger().fatal(f"Serial open failed: {e}")
            raise

        self.pub = self.create_publisher(Odometry, '/esp/odom', 10)
        self.timer = self.create_timer(0.05, self.read_serial)  # 20 Hz

        self.get_logger().info("ESP Serial → /esp/odom bridge started")

    def read_serial(self):
        try:
            line = self.ser.readline().decode().strip()
            if not line.startswith("ODOM"):
                return

            _, x, y, th, v, w = line.split(",")

            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'

            msg.pose.pose.position.x = float(x)
            msg.pose.pose.position.y = float(y)

            q = Quaternion()
            q.z = math.sin(float(th) / 2.0)
            q.w = math.cos(float(th) / 2.0)
            msg.pose.pose.orientation = q

            msg.twist.twist.linear.x = float(v)
            msg.twist.twist.angular.z = float(w)

            self.pub.publish(msg)

        except Exception as e:
            self.get_logger().warn(str(e))


def main():
    rclpy.init()
    node = ESPSerialToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
