#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial, time

class CmdVelBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('max_speed', 0.3)  # m/s
        port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        try:
            self.ser = serial.Serial(port, baud, timeout=0.1)
            time.sleep(0.2)
            self.get_logger().info(f"Serial opened {port}@{baud}")
        except Exception as e:
            self.get_logger().error(f"Cannot open serial: {e}")
            raise
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cb, 10)

    def cb(self, msg: Twist):
        linear = msg.linear.x
        angular = msg.angular.z
        wheel_base = 0.30  # change to your robot
        left = linear - angular * wheel_base / 2.0
        right = linear + angular * wheel_base / 2.0
        # normalize
        maxv = max(abs(left), abs(right), self.max_speed)
        if maxv > self.max_speed:
            left *= self.max_speed / maxv
            right *= self.max_speed / maxv
        left_norm = max(-1.0, min(1.0, left / self.max_speed))
        right_norm = max(-1.0, min(1.0, right / self.max_speed))
        line = f"M,{left_norm:.3f},{right_norm:.3f}\n"
        try:
            self.ser.write(line.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

def main():
    rclpy.init()
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
