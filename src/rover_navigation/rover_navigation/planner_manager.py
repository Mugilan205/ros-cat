"""
Planner Manager
===============

Coordinates global and local planners, handles goal reception and replanning logic.

This node acts as a coordinator that:
- Receives goals and forwards to global planner
- Monitors global path execution
- Triggers replanning when needed
- Manages state transitions

Topics:
  Subscribed:
    - /goal: Goal pose (geometry_msgs/PoseStamped)
    - /global_path: Global path from A* planner
    - /odom: Current robot pose
  Published:
    - /planner/debug/state: Planner state (std_msgs/String)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String
import math


class PlannerManagerNode(Node):
    """
    Manages coordination between global and local planners.
    """
    
    def __init__(self):
        super().__init__('planner_manager')
        
        # Parameters
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('replan_distance_threshold', 0.5)
        self.declare_parameter('monitor_frequency', 5.0)
        
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.replan_distance_threshold = self.get_parameter('replan_distance_threshold').value
        monitor_freq = self.get_parameter('monitor_frequency').value
        
        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriptions
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal',
            self.goal_callback,
            10
        )
        
        self.global_path_sub = self.create_subscription(
            Path,
            '/global_path',
            self.global_path_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            PoseStamped,
            '/odom',
            self.odom_callback,
            sensor_qos
        )
        
        # Publications
        self.state_pub = self.create_publisher(String, '/planner/debug/state', 10)
        
        # State
        self.current_goal = None
        self.current_path = None
        self.current_pose = None
        self.state = 'IDLE'
        
        # Monitoring timer
        self.monitor_timer = self.create_timer(1.0 / monitor_freq, self.monitor_callback)
        
        self.get_logger().info('Planner Manager Node initialized')
        self.publish_state('IDLE')
    
    def goal_callback(self, msg: PoseStamped):
        """Receive goal pose."""
        self.current_goal = msg
        self.get_logger().info(
            f'Received goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        self.publish_state('PLANNING')
    
    def global_path_callback(self, msg: Path):
        """Receive global path from A* planner."""
        self.current_path = msg
        if len(msg.poses) > 0:
            self.get_logger().debug(f'Received global path with {len(msg.poses)} waypoints')
            self.publish_state('ACTIVE')
        else:
            self.publish_state('FAILED')
    
    def odom_callback(self, msg: PoseStamped):
        """Receive current robot pose."""
        self.current_pose = msg
    
    def monitor_callback(self):
        """Monitor path execution and check if goal reached."""
        if self.current_goal is None or self.current_pose is None:
            return
        
        # Check if goal reached
        dist_to_goal = self.distance_between_poses(self.current_pose, self.current_goal)
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info('Goal reached!')
            self.publish_state('GOAL_REACHED')
            self.current_goal = None
            return
        
        # Check if path is valid
        if self.current_path is None or len(self.current_path.poses) == 0:
            if self.state != 'FAILED':
                self.publish_state('WAITING_FOR_PATH')
            return
        
        # Check if we need replanning (robot deviated significantly from path)
        if self.current_path and len(self.current_path.poses) > 0:
            min_dist_to_path = float('inf')
            for pose in self.current_path.poses:
                dist = self.distance_between_poses(self.current_pose, pose)
                min_dist_to_path = min(min_dist_to_path, dist)
            
            if min_dist_to_path > self.replan_distance_threshold:
                self.get_logger().warn(
                    f'Robot deviated from path ({min_dist_to_path:.2f}m), may need replanning'
                )
                # Note: Actual replanning is handled by global planner
    
    def publish_state(self, state: str):
        """Publish current planner state."""
        self.state = state
        msg = String()
        msg.data = state
        self.state_pub.publish(msg)
    
    @staticmethod
    def distance_between_poses(pose1: PoseStamped, pose2: PoseStamped) -> float:
        """Calculate distance between two poses."""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx * dx + dy * dy)


def main(args=None):
    rclpy.init(args=args)
    node = PlannerManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()









