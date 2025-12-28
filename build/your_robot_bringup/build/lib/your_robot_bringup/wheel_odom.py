#!/usr/bin/env python3
import rclpy
import serial
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

WHEEL_RADIUS = 0.065   # meters
WHEEL_BASE = 0.30     # meters
TICKS_PER_REV = 600

class WheelOdom(Node):
    def __init__(self):
        super().__init__('wheel_odom')

        try:
            self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
            self.get_logger().info("Arduino connected on /dev/ttyACM0")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open Arduino on /dev/ttyACM0: {e}")
            self.get_logger().warn("Odometry will not work. Check Arduino connection and upload test code.")
            self.ser = None

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

    def update(self):
        if self.ser is None:
            return
        
        try:
            line = self.ser.readline().decode().strip()
            if not line:
                return
            left_ticks, right_ticks = map(int, line.split(','))
        except Exception as e:
            self.get_logger().debug(f"Serial read error: {e}")
            return

        dl = 2 * math.pi * WHEEL_RADIUS * (left_ticks / TICKS_PER_REV)
        dr = 2 * math.pi * WHEEL_RADIUS * (right_ticks / TICKS_PER_REV)

        dc = (dl + dr) / 2.0
        dtheta = (dr - dl) / WHEEL_BASE

        self.theta += dtheta
        self.x += dc * math.cos(self.theta)
        self.y += dc * math.sin(self.theta)

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        odom.pose.pose.orientation.z = math.sin(self.theta / 2.0)

        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header = odom.header
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation = odom.pose.pose.orientation

        self.tf_broadcaster.sendTransform(tf)

def main():
    rclpy.init()
    rclpy.spin(WheelOdom())
