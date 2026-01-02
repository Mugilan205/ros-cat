#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
import tf2_ros
import serial
import math
import time

class ESPSerialToOdom(Node):

    def __init__(self):
        super().__init__('esp_serial_to_odom')

        # Use persistent device path for ESP (FTDI)
        self.declare_parameter('port', '/dev/serial/by-id/usb-FTDI_USB_Serial_Converter_FTB6SPL3-if00-port0')
        self.declare_parameter('baud', 115200)
        
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=1)
            # Give some time for serial to initialize
            time.sleep(2)
            self.get_logger().info(f"Connected to ESP on {self.port}")
        except Exception as e:
            self.get_logger().fatal(f"Could not open serial port {self.port}: {e}")
            raise e

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # REQUIRED FIX: Create TransformBroadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        self.timer = self.create_timer(0.05, self.read_serial)  # 20 Hz

        self.get_logger().info("ESP Serial → /odom bridge started with dynamic TF")

    def read_serial(self):
        try:
            # Read line from serial
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            
            if not line or not line.startswith("ODOM"):
                return

            parts = line.split(",")
            if len(parts) != 6:
                return

            # Protocol: ODOM,x,y,theta,v,w
            _, x_str, y_str, th_str, v_str, w_str = parts
            x = float(x_str)
            y = float(y_str)
            theta = float(th_str)
            v = float(v_str)
            w = float(w_str)

            now = self.get_clock().now().to_msg()

            # 1. Create and Publish Odometry Message
            odom_msg = Odometry()
            odom_msg.header.stamp = now
            odom_msg.header.frame_id = 'odom'
            odom_msg.child_frame_id = 'base_link'

            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.position.z = 0.0

            # Convert yaw to quaternion (REQUIRED FIX logic)
            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)
            
            odom_msg.pose.pose.orientation.x = 0.0
            odom_msg.pose.pose.orientation.y = 0.0
            odom_msg.pose.pose.orientation.z = qz
            odom_msg.pose.pose.orientation.w = qw

            odom_msg.twist.twist.linear.x = v
            odom_msg.twist.twist.angular.z = w

            self.odom_pub.publish(odom_msg)

            # 2. Broadcast Dynamic TF (odom -> base_link)
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'
            
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().warn(f"Error in read_serial: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ESPSerialToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
