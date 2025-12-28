#!/usr/bin/env python3
"""
ESP8266 Encoder Bridge Node
Reads REAL encoder values from ESP8266 rover via WiFi
Converts encoder ticks to position and publishes to /odom
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion
import socket
import math
import time
import threading


class ESP8266Bridge(Node):
    def __init__(self):
        super().__init__("esp8266_bridge")
        
        # ESP8266 parameters
        self.declare_parameter("esp8266_ip", "192.168.4.1")
        self.declare_parameter("esp8266_port", 80)
        self.declare_parameter("wheel_radius", 0.05)  # 5cm wheels
        self.declare_parameter("wheel_base", 0.15)    # 15cm between wheels
        self.declare_parameter("encoder_ticks_per_rev", 20)  # encoder resolution
        self.declare_parameter("update_frequency", 10.0)  # Hz
        
        self.esp8266_ip = self.get_parameter("esp8266_ip").value
        self.esp8266_port = self.get_parameter("esp8266_port").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_base = self.get_parameter("wheel_base").value
        self.encoder_ticks_per_rev = self.get_parameter("encoder_ticks_per_rev").value
        self.update_frequency = self.get_parameter("update_frequency").value
        
        # Calculate mm per tick for each wheel
        self.mm_per_tick = (2 * math.pi * self.wheel_radius * 1000) / self.encoder_ticks_per_rev
        
        # Odometry state
        self.x = 0.0  # meters
        self.y = 0.0  # meters
        self.theta = 0.0  # radians
        
        # Previous encoder values
        self.prev_left_count = 0
        self.prev_right_count = 0
        self.last_update = time.time()
        
        # Publisher
        self.odom_pub = self.create_publisher(PoseStamped, "/odom", 10)
        
        # Timer for reading from ESP8266
        update_period = 1.0 / self.update_frequency
        self.timer = self.create_timer(update_period, self.read_and_publish)
        
        # Connection status
        self.connected = False
        
        self.get_logger().info(
            f"ESP8266 Bridge Node Started\n"
            f"  WiFi SSID: RC_ROVER\n"
            f"  IP: {self.esp8266_ip}:{self.esp8266_port}\n"
            f"  Wheel Radius: {self.wheel_radius} m\n"
            f"  Wheel Base: {self.wheel_base} m\n"
            f"  Encoder Resolution: {self.encoder_ticks_per_rev} ticks/rev\n"
            f"  MM per tick: {self.mm_per_tick:.4f}"
        )

    def read_encoder_values(self):
        """
        Read encoder values from ESP8266 web interface.
        Returns: (left_count, right_count) or (None, None) if failed
        """
        try:
            # Create socket and connect to ESP8266
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(2.0)
            sock.connect((self.esp8266_ip, self.esp8266_port))
            
            # Send HTTP GET request
            request = b"GET / HTTP/1.1\r\nHost: " + self.esp8266_ip.encode() + b"\r\nConnection: close\r\n\r\n"
            sock.sendall(request)
            
            # Receive response
            response = b""
            while True:
                try:
                    chunk = sock.recv(1024)
                    if not chunk:
                        break
                    response += chunk
                except socket.timeout:
                    break
            
            sock.close()
            
            # Parse response to extract encoder values
            response_str = response.decode('utf-8', errors='ignore')
            
            left_count = None
            right_count = None
            
            # Look for "Left Encoder: X" and "Right Encoder: Y"
            lines = response_str.split('\n')
            for line in lines:
                if 'Left Encoder:' in line:
                    # Extract number from "Left Encoder: 123<br>"
                    parts = line.split(':')
                    if len(parts) > 1:
                        value_str = parts[1].strip().split('<')[0].strip()
                        try:
                            left_count = int(value_str)
                        except ValueError:
                            pass
                
                if 'Right Encoder:' in line:
                    parts = line.split(':')
                    if len(parts) > 1:
                        value_str = parts[1].strip().split('<')[0].strip()
                        try:
                            right_count = int(value_str)
                        except ValueError:
                            pass
            
            if left_count is not None and right_count is not None:
                if not self.connected:
                    self.connected = True
                    self.get_logger().info("✅ Connected to ESP8266!")
                return left_count, right_count
            else:
                if self.connected:
                    self.connected = False
                    self.get_logger().warn("❌ Could not parse encoder values from ESP8266")
                return None, None
                
        except Exception as e:
            if self.connected:
                self.connected = False
                self.get_logger().error(f"❌ Failed to read from ESP8266: {e}")
            return None, None

    def update_odometry(self, left_count, right_count):
        """Update odometry based on encoder ticks using differential drive kinematics."""
        
        # Calculate delta ticks
        delta_left = left_count - self.prev_left_count
        delta_right = right_count - self.prev_right_count
        
        self.prev_left_count = left_count
        self.prev_right_count = right_count
        
        # Convert ticks to distance in mm
        dist_left_mm = delta_left * self.mm_per_tick
        dist_right_mm = delta_right * self.mm_per_tick
        
        # Convert to meters
        dist_left = dist_left_mm / 1000.0
        dist_right = dist_right_mm / 1000.0
        
        # Calculate movement using differential drive equations
        dist_center = (dist_left + dist_right) / 2.0
        delta_theta = (dist_right - dist_left) / self.wheel_base
        
        # Update pose
        self.x += dist_center * math.cos(self.theta)
        self.y += dist_center * math.sin(self.theta)
        self.theta += delta_theta
        
        # Normalize theta to [-π, π]
        while self.theta > math.pi:
            self.theta -= 2 * math.pi
        while self.theta < -math.pi:
            self.theta += 2 * math.pi

    def publish_odom(self):
        """Publish current odometry state."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        
        # Position in meters
        msg.pose.position.x = self.x
        msg.pose.position.y = self.y
        msg.pose.position.z = 0.0
        
        # Orientation as quaternion from theta
        q = self.euler_to_quaternion(0, 0, self.theta)
        msg.pose.orientation = q
        
        self.odom_pub.publish(msg)

    def read_and_publish(self):
        """Main callback: read encoders and publish odometry."""
        left_count, right_count = self.read_encoder_values()
        
        if left_count is not None and right_count is not None:
            self.update_odometry(left_count, right_count)
            self.publish_odom()
            
            # Debug log every 10 updates
            if int(time.time() * 10) % 10 == 0:
                self.get_logger().debug(
                    f"Encoders: L={left_count}, R={right_count} | "
                    f"Odom: X={self.x:.3f}m, Y={self.y:.3f}m, θ={math.degrees(self.theta):.1f}°"
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
    node = ESP8266Bridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
