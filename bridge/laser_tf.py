#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class LaserTF(Node):
    def __init__(self):
        super().__init__('laser_tf_broadcaster')

        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.broadcast)

    def broadcast(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "laser"

        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0

        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        self.get_logger().info("Broadcasting laser -> base_link TF")
        self.br.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = LaserTF()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
