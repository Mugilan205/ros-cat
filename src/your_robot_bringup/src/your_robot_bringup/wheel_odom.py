#!/usr/bin/env python3
import rclpy
import serial
import math
import urllib.request
import re
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

        # configuration: try serial by default; if not present, will try HTTP
        self.encoder_source = self.declare_parameter('encoder_source', 'serial').get_parameter_value().string_value
        self.encoder_serial_port = self.declare_parameter('encoder_serial_port', '/dev/ttyACM0').get_parameter_value().string_value
        self.encoder_baud = self.declare_parameter('encoder_baud', 115200).get_parameter_value().integer_value
        self.encoder_http_url = self.declare_parameter('encoder_http_url', 'http://192.168.4.1/').get_parameter_value().string_value

        self.ser = None
        if self.encoder_source == 'serial':
            try:
                self.ser = serial.Serial(self.encoder_serial_port, self.encoder_baud, timeout=1)
                self.get_logger().info(f"Arduino connected on {self.encoder_serial_port}")
            except serial.SerialException as e:
                self.get_logger().error(f"Failed to open serial port {self.encoder_serial_port}: {e}")
                self.get_logger().warn("Will attempt HTTP encoder reads if available.")
                self.ser = None

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # previous tick values to compute deltas
        self.prev_left = 0
        self.prev_right = 0

        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

    def _read_encoders_serial(self):
        try:
            line = self.ser.readline().decode().strip()
            if not line:
                return None
            left, right = map(int, line.split(','))
            return (left, right)
        except Exception as e:
            self.get_logger().debug(f"Serial read error: {e}")
            return None

    def _read_encoders_http(self):
        try:
            with urllib.request.urlopen(self.encoder_http_url, timeout=1.0) as resp:
                text = resp.read().decode('utf-8', errors='ignore')
            m = re.search(r'Left[^\d]*(\d+)[^\d]*Right[^\d]*(\d+)', text, re.IGNORECASE)
            if m:
                left = int(m.group(1))
                right = int(m.group(2))
                return (left, right)
            nums = re.findall(r'(\d+)', text)
            if len(nums) >= 2:
                return (int(nums[0]), int(nums[1]))
            return None
        except Exception as e:
            self.get_logger().debug(f"HTTP read error: {e}")
            return None

    def update(self):
        # read encoder counts from configured source
        counts = None
        if self.encoder_source == 'serial' and self.ser is not None:
            counts = self._read_encoders_serial()
        if counts is None:
            counts = self._read_encoders_http()
        if counts is None:
            return

        left_ticks, right_ticks = counts

        # compute delta ticks since last read
        dl_ticks = left_ticks - self.prev_left
        dr_ticks = right_ticks - self.prev_right
        self.prev_left = left_ticks
        self.prev_right = right_ticks

        dl = 2 * math.pi * WHEEL_RADIUS * (dl_ticks / TICKS_PER_REV)
        dr = 2 * math.pi * WHEEL_RADIUS * (dr_ticks / TICKS_PER_REV)

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
