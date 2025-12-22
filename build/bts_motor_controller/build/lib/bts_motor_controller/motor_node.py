#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pigpio

# BTS7960 Pins
L_PWM = 12   # GPIO12
L_DIR = 5    # GPIO5
R_PWM = 13   # GPIO13
R_DIR = 6    # GPIO6

class MotorController(Node):
    def __init__(self):
        super().__init__("motor_controller")

        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("pigpio daemon not running!")
            exit(1)

        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_callback, 10
        )

        self.get_logger().info("Motor Controller Started")

    def set_motor(self, pwm_pin, dir_pin, speed):
        if speed >= 0:
            self.pi.write(dir_pin, 1)
            duty = min(int(speed * 255), 255)
        else:
            self.pi.write(dir_pin, 0)
            duty = min(int(abs(speed) * 255), 255)

        self.pi.set_PWM_dutycycle(pwm_pin, duty)

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Differential Drive
        left = linear - angular
        right = linear + angular

        self.set_motor(L_PWM, L_DIR, left)
        self.set_motor(R_PWM, R_DIR, right)

def main(args=None):
    rclpy.init(args=args)
    node = MotorController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
