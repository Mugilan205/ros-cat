import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros

class StaticOdom(Node):
    def __init__(self):
        super().__init__('static_odom')

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.timer = self.create_timer(0.05, self.publish_odom)

    def publish_odom(self):
        # STATIC ODOM (robot not moving)
        x = 0.0
        y = 0.0
        theta = 0.0

        # Odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.w = 1.0

        self.odom_pub.publish(odom)

        # TF: odom -> base_link
        tf = TransformStamped()
        tf.header.stamp = odom.header.stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"

        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(tf)

        self.get_logger().info("STATIC ODOM → base_link @ (0,0,0)")

def main(args=None):
    rclpy.init(args=args)
    node = StaticOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
