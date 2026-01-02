#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

import math


class OdometryNode(Node):

    def __init__(self):
        super().__init__('odometry_node')

        # Parameters
        self.declare_parameter('esp_odom_topic', '/esp/odom')
        esp_topic = self.get_parameter('esp_odom_topic').value

        # State
        self.latest_esp_odom = None
        self.esp_odom_received = False

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber
        self.create_subscription(
            Odometry,
            esp_topic,
            self.esp_odom_callback,
            10
        )

        # Publisher
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        # Timer (20 Hz)
        self.timer = self.create_timer(0.05, self.publish_odom)

        self.get_logger().info(
            "Odometry Node Started (ESP Relay Mode)\n"
            f"  Subscribing to: {esp_topic}\n"
            "  Publishing: /odom @ 20 Hz\n"
            "  Broadcasting TF: odom -> base_link"
        )

    def esp_odom_callback(self, msg: Odometry):
        self.latest_esp_odom = msg
        self.esp_odom_received = True

    def publish_odom(self):
        if not self.esp_odom_received:
        # publish zero pose so TF exists
          dummy = Odometry()
          dummy.pose.pose.orientation.w = 1.0
          self.latest_esp_odom = dummy


        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Copy pose
        msg.pose.pose = self.latest_esp_odom.pose.pose
        msg.twist.twist = self.latest_esp_odom.twist.twist

        msg.pose.covariance = self.latest_esp_odom.pose.covariance
        msg.twist.covariance = self.latest_esp_odom.twist.covariance

        self.odom_pub.publish(msg)

        # TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)

    node = OdometryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
