import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelToWheels(Node):
    def __init__(self):
        super().__init__('cmdvel_to_wheels')

        self.wheel_base = 0.45   # meters (adjust later)
        self.wheel_radius = 0.075  # meters

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.get_logger().info("CMD_VEL → Wheel converter started (DRY RUN)")

    def cmd_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        v_left = v - (w * self.wheel_base / 2.0)
        v_right = v + (w * self.wheel_base / 2.0)

        rpm_left = (v_left / (2 * 3.1416 * self.wheel_radius)) * 60
        rpm_right = (v_right / (2 * 3.1416 * self.wheel_radius)) * 60

        self.get_logger().info(
            f"L RPM: {rpm_left:.2f} | R RPM: {rpm_right:.2f}"
        )

def main():
    rclpy.init()
    node = CmdVelToWheels()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()