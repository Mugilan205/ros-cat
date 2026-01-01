#!/usr/bin/env python3
"""Dry-run node converting /cmd_vel to wheel RPMs (no hardware output)."""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelToWheels(Node):
    def __init__(self) -> None:
        super().__init__('cmdvel_to_wheels')
        self.wheel_base = 0.45  # meters
        self.wheel_radius = 0.075  # meters
        self.rpm_scale = 60.0 / (2.0 * math.pi * self.wheel_radius)  # m/s to RPM per wheel

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.get_logger().info('cmdvel_to_wheels node started (dry-run)')

    def _on_cmd_vel(self, msg: Twist) -> None:
        v = msg.linear.x
        w = msg.angular.z

        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        rpm_left = v_left * self.rpm_scale
        rpm_right = v_right * self.rpm_scale

        print(f"L RPM: {rpm_left:.2f} | R RPM: {rpm_right:.2f}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelToWheels()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
