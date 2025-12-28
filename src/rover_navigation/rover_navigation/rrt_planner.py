"""
RRT Path Planner with Obstacle Avoidance
=========================================

This node implements a Rapidly-exploring Random Tree (RRT) algorithm
with dynamic obstacle avoidance.

Algorithm Overview:
-------------------

RRT (Rapidly-exploring Random Tree):
1. Start with root node at current position
2. Sample random configurations in the workspace
3. Find nearest tree node to sample point
4. Extend tree toward sample point if collision-free
5. Repeat until goal is within extension distance
6. Return path from root to goal

Obstacle Avoidance:
- Check for collision using circle-based approximation
- Maintain clearance distance around obstacles
- Re-plan if new obstacles detected within path

Dynamic Re-planning:
- Monitor detected obstacles
- Check if current path collides with new obstacles
- Trigger re-planning if collision detected
- Implement incremental RRT* for online planning

Topics:
  Subscribed:
    - /obstacles: Detected obstacle positions
    - /odom: Current robot odometry
    - /goal: Target goal position
  Published:
    - /path: Planned path (nav_msgs/Path)
    - /plan_status: Planning status messages

Parameters:
  - step_size: Max distance per RRT extension (default: 0.5m)
  - max_iterations: Maximum planning iterations (default: 5000)
  - obstacle_clearance: Safety margin around obstacles (default: 0.3m)
  - robot_radius: Collision radius of robot (default: 0.2m)
  - replan_frequency: How often to check for replanning (default: 5 Hz)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped, Point, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String
import numpy as np
import math
import time
from typing import List, Tuple, Optional


class TreeNode:
    """Represents a node in the RRT tree."""
    
    def __init__(self, x: float, y: float, parent=None):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = 0.0 if parent is None else parent.cost + math.sqrt(
            (x - parent.x) ** 2 + (y - parent.y) ** 2
        )
    
    def distance_to(self, x: float, y: float) -> float:
        """Calculate Euclidean distance to point."""
        return math.sqrt((self.x - x) ** 2 + (self.y - y) ** 2)
    
    def to_pose(self, timestamp, frame_id: str = 'odom') -> PoseStamped:
        """Convert node to PoseStamped message."""
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = timestamp
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.orientation.w = 1.0
        return pose


class RRTPlannerNode(Node):
    """
    Implements RRT path planning with obstacle avoidance.
    """

    def __init__(self):
        super().__init__('rrt_planner')
        
        # Parameters
        self.declare_parameter('step_size', 0.5)
        self.declare_parameter('max_iterations', 5000)
        self.declare_parameter('obstacle_clearance', 0.3)
        self.declare_parameter('robot_radius', 0.2)
        self.declare_parameter('replan_frequency', 5.0)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('workspace_size', 20.0)
        self.declare_parameter('frame_id', 'odom')
        
        self.step_size = self.get_parameter('step_size').value
        self.max_iterations = self.get_parameter('max_iterations').value
        self.obstacle_clearance = self.get_parameter('obstacle_clearance').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.replan_freq = self.get_parameter('replan_frequency').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.workspace = self.get_parameter('workspace_size').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriptions
        self.obstacles_sub = self.create_subscription(
            PoseStamped,
            '/obstacles',
            self.obstacles_callback,
            sensor_qos
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal',
            self.goal_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            PoseStamped,
            '/odom',
            self.odom_callback,
            sensor_qos
        )
        
        # Publications
        self.path_pub = self.create_publisher(Path, '/path', 10)
        self.status_pub = self.create_publisher(String, '/plan_status', 10)
        
        # State variables
        self.current_pose = None
        self.goal_pose = None
        self.obstacles = []  # List of (x, y, radius) tuples
        self.current_path = None
        self.tree = []
        
        # Planning timer
        self.planning_timer = self.create_timer(
            1.0 / self.replan_freq,
            self.planning_callback
        )
        
        self.get_logger().info('RRT Planner Node initialized')
        self.get_logger().info(f'Step size: {self.step_size}m')
        self.get_logger().info(f'Obstacle clearance: {self.obstacle_clearance}m')

    def obstacles_callback(self, msg: PoseStamped):
        """
        Update obstacle list from detected obstacles.
        
        Args:
            msg: PoseStamped message with obstacle position
        """
        obstacle_pos = (msg.pose.position.x, msg.pose.position.y)
        
        # Check if this obstacle is already tracked (within clustering distance)
        clustering_distance = 0.5  # meters
        for i, (obs_x, obs_y, _) in enumerate(self.obstacles):
            if math.sqrt((obstacle_pos[0] - obs_x) ** 2 + 
                        (obstacle_pos[1] - obs_y) ** 2) < clustering_distance:
                # Update existing obstacle
                self.obstacles[i] = (obstacle_pos[0], obstacle_pos[1], 
                                    self.robot_radius + self.obstacle_clearance)
                return
        
        # Add new obstacle
        self.obstacles.append((
            obstacle_pos[0],
            obstacle_pos[1],
            self.robot_radius + self.obstacle_clearance
        ))

    def goal_callback(self, msg: PoseStamped):
        """
        Receive new goal and trigger planning.
        
        Args:
            msg: Goal pose
        """
        self.goal_pose = msg
        self.get_logger().info(
            f'Received goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

    def odom_callback(self, msg: PoseStamped):
        """
        Update current robot position.
        
        Args:
            msg: Current odometry pose
        """
        self.current_pose = msg

    def planning_callback(self):
        """
        Main planning loop - plan path if goal is set.
        
        Decision logic:
        1. If no goal, do nothing
        2. If no current pose, do nothing
        3. If current path exists, check for collisions
        4. If collision detected, trigger replanning
        5. If no path exists or replanning needed, run RRT
        """
        if self.goal_pose is None or self.current_pose is None:
            return
        
        # Check if current path is still valid
        if self.current_path is not None:
            if not self.is_path_valid(self.current_path):
                self.get_logger().warn('Current path collides with new obstacle - replanning...')
                status_msg = String()
                status_msg.data = 'REPLANNING'
                self.status_pub.publish(status_msg)
                self.current_path = None
        
        # Plan new path if needed
        if self.current_path is None:
            self.plan_path()

    def plan_path(self):
        """
        Execute RRT algorithm to plan collision-free path.
        
        RRT Algorithm:
        ===============
        1. Initialize tree with current position
        2. Loop for max_iterations:
           a. Generate random sample (90% random, 10% goal biasing)
           b. Find nearest node in tree
           c. Create new node by extending toward sample
           d. Check collision on line segment
           e. If collision-free, add to tree
           f. If new node is close to goal, connect and return path
        3. Return best path found or None
        """
        start_time = time.time()
        
        start_x = self.current_pose.pose.position.x
        start_y = self.current_pose.pose.position.y
        goal_x = self.goal_pose.pose.position.x
        goal_y = self.goal_pose.pose.position.y
        
        # Initialize tree
        self.tree = [TreeNode(start_x, start_y)]
        path_found = False
        goal_node = None
        
        for iteration in range(self.max_iterations):
            # Sample random point
            if np.random.random() < 0.1:  # 10% goal biasing
                rand_x, rand_y = goal_x, goal_y
            else:
                rand_x = np.random.uniform(-self.workspace / 2, self.workspace / 2)
                rand_y = np.random.uniform(-self.workspace / 2, self.workspace / 2)
            
            # Find nearest node
            nearest_node = self.find_nearest_node(rand_x, rand_y)
            if nearest_node is None:
                continue
            
            # Extend toward random point
            distance = nearest_node.distance_to(rand_x, rand_y)
            if distance < 1e-6:
                continue
            
            # Calculate new node position
            new_x = nearest_node.x + self.step_size * (rand_x - nearest_node.x) / distance
            new_y = nearest_node.y + self.step_size * (rand_y - nearest_node.y) / distance
            
            # Check collision
            if not self.is_collision_free(nearest_node.x, nearest_node.y, new_x, new_y):
                continue
            
            # Add new node
            new_node = TreeNode(new_x, new_y, nearest_node)
            self.tree.append(new_node)
            
            # Check if goal reached
            dist_to_goal = new_node.distance_to(goal_x, goal_y)
            if dist_to_goal < self.goal_tolerance:
                # Try to connect directly to goal
                if self.is_collision_free(new_x, new_y, goal_x, goal_y):
                    goal_node = TreeNode(goal_x, goal_y, new_node)
                    path_found = True
                    break
            
            # Early exit if we have a reasonable path
            if iteration > 100 and path_found:
                break
        
        elapsed = time.time() - start_time
        
        if path_found and goal_node:
            self.current_path = self.reconstruct_path(goal_node)
            self.get_logger().info(
                f'Path found in {elapsed:.3f}s ({iteration} iterations), '
                f'length: {len(self.current_path.poses)} poses'
            )
            
            status_msg = String()
            status_msg.data = 'SUCCESS'
            self.status_pub.publish(status_msg)
        else:
            self.get_logger().warn(
                f'No path found after {self.max_iterations} iterations ({elapsed:.3f}s)'
            )
            status_msg = String()
            status_msg.data = 'FAILED'
            self.status_pub.publish(status_msg)
        
        # Publish path
        self.path_pub.publish(self.current_path if self.current_path else Path())

    def find_nearest_node(self, x: float, y: float) -> Optional[TreeNode]:
        """
        Find nearest node in tree to given point using Euclidean distance.
        
        Complexity: O(n) where n is tree size
        Note: For large trees, use KD-tree for O(log n) performance
        
        Args:
            x: Target x coordinate
            y: Target y coordinate
            
        Returns:
            Nearest tree node or None if tree is empty
        """
        if not self.tree:
            return None
        
        nearest = self.tree[0]
        min_dist = nearest.distance_to(x, y)
        
        for node in self.tree[1:]:
            dist = node.distance_to(x, y)
            if dist < min_dist:
                nearest = node
                min_dist = dist
        
        return nearest

    def is_collision_free(self, x1: float, y1: float, x2: float, y2: float) -> bool:
        """
        Check if line segment from (x1,y1) to (x2,y2) collides with obstacles.
        
        Collision Detection Method:
        - For each obstacle, compute distance from line segment to obstacle center
        - If distance < obstacle_radius, collision detected
        - Use closest point on line segment to obstacle center
        
        Args:
            x1, y1: Start point
            x2, y2: End point
            
        Returns:
            True if collision-free, False if collision detected
        """
        for obs_x, obs_y, obs_radius in self.obstacles:
            if self.segment_circle_collision(x1, y1, x2, y2, obs_x, obs_y, obs_radius):
                return False
        return True

    def segment_circle_collision(self, x1: float, y1: float, x2: float, y2: float,
                                cx: float, cy: float, radius: float) -> bool:
        """
        Detect collision between line segment and circle.
        
        Geometry:
        1. Find closest point on line segment to circle center
        2. Compute distance from that point to circle center
        3. If distance < radius, collision occurs
        
        Args:
            x1, y1: Line segment start
            x2, y2: Line segment end
            cx, cy: Circle center
            radius: Circle radius
            
        Returns:
            True if collision, False otherwise
        """
        # Vector from start to end
        dx = x2 - x1
        dy = y2 - y1
        
        # Vector from start to circle center
        fx = x1 - cx
        fy = y1 - cy
        
        # If segment has zero length
        if dx == 0 and dy == 0:
            return math.sqrt(fx * fx + fy * fy) < radius
        
        # Parameter t for closest point on segment
        t = -(fx * dx + fy * dy) / (dx * dx + dy * dy)
        t = max(0, min(1, t))  # Clamp to segment
        
        # Closest point on segment
        closest_x = x1 + t * dx
        closest_y = y1 + t * dy
        
        # Distance to circle center
        dist = math.sqrt(
            (closest_x - cx) ** 2 + (closest_y - cy) ** 2
        )
        
        return dist < radius

    def is_path_valid(self, path: Path) -> bool:
        """
        Check if existing path is still collision-free.
        
        Re-planning Trigger:
        - Sample path at regular intervals
        - Check collision for each segment
        - Return False if any collision detected
        
        Args:
            path: Path message to validate
            
        Returns:
            True if path is still valid, False if replanning needed
        """
        if len(path.poses) < 2:
            return False
        
        for i in range(len(path.poses) - 1):
            x1 = path.poses[i].pose.position.x
            y1 = path.poses[i].pose.position.y
            x2 = path.poses[i + 1].pose.position.x
            y2 = path.poses[i + 1].pose.position.y
            
            if not self.is_collision_free(x1, y1, x2, y2):
                return False
        
        return True

    def reconstruct_path(self, goal_node: TreeNode) -> Path:
        """
        Reconstruct path from start to goal by traversing parent pointers.
        
        Args:
            goal_node: Goal node in tree
            
        Returns:
            Path message with all waypoints from start to goal
        """
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Traverse from goal back to start
        current = goal_node
        poses = []
        while current is not None:
            poses.append(current)
            current = current.parent
        
        # Reverse to get start -> goal order
        poses.reverse()
        
        # Convert to PoseStamped messages
        for node in poses:
            path.poses.append(node.to_pose(path.header.stamp, self.frame_id))
        
        return path


def main(args=None):
    rclpy.init(args=args)
    node = RRTPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
