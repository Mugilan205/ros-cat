#!/usr/bin/env python3
import rclpy
import math
import random
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

# -------- RRT PARAMETERS --------
GOAL_X = 3.0
GOAL_Y = 0.0
STEP_SIZE = 0.3
GOAL_THRESHOLD = 0.4
MAX_ITER = 300

# -------- RRT NODE CLASS --------
class RRTNode:
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

class RRTAuto(Node):
    def __init__(self):
        super().__init__('rrt_autonomous')

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.obstacles = []

        self.timer = self.create_timer(0.1, self.control_loop)

    # ---------- CALLBACKS ----------
    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        self.yaw = math.atan2(
            2 * (q.w * q.z),
            1 - 2 * (q.z * q.z)
        )

    def scan_cb(self, msg):
        self.obstacles.clear()
        angle = msg.angle_min

        for r in msg.ranges:
            if 0.1 < r < 2.5:
                ox = self.x + r * math.cos(angle + self.yaw)
                oy = self.y + r * math.sin(angle + self.yaw)
                self.obstacles.append((ox, oy))
            angle += msg.angle_increment

    # ---------- RRT CORE ----------
    def build_rrt(self):
        start = RRTNode(self.x, self.y)
        tree = [start]

        for _ in range(MAX_ITER):
            rand_x = random.uniform(self.x - 3, self.x + 3)
            rand_y = random.uniform(self.y - 3, self.y + 3)

            nearest = min(
                tree,
                key=lambda n: math.hypot(n.x - rand_x, n.y - rand_y)
            )

            theta = math.atan2(rand_y - nearest.y, rand_x - nearest.x)
            new_x = nearest.x + STEP_SIZE * math.cos(theta)
            new_y = nearest.y + STEP_SIZE * math.sin(theta)

            if not self.collision(new_x, new_y):
                new_node = RRTNode(new_x, new_y, nearest)
                tree.append(new_node)

                if math.hypot(new_x - GOAL_X, new_y - GOAL_Y) < GOAL_THRESHOLD:
                    return new_node
        return None

    def collision(self, x, y):
        for ox, oy in self.obstacles:
            if math.hypot(x - ox, y - oy) < 0.25:
                return True
        return False

    # ---------- CONTROL ----------
    def control_loop(self):
        goal_dist = math.hypot(GOAL_X - self.x, GOAL_Y - self.y)
        cmd = Twist()

        if goal_dist < GOAL_THRESHOLD:
            self.get_logger().info("✅ Goal reached")
            self.cmd_pub.publish(cmd)
            return

        path_node = self.build_rrt()
        if not path_node:
            cmd.angular.z = 0.5
            self.cmd_pub.publish(cmd)
            return

        target_x = path_node.x
        target_y = path_node.y

        angle_to_target = math.atan2(
            target_y - self.y,
            target_x - self.x
        )

        angle_error = angle_to_target - self.yaw
        cmd.linear.x = 0.25
        cmd.angular.z = 1.5 * angle_error

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(RRTAuto())
