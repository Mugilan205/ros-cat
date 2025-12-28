"""
Local Weighted Band (LWB) Planner
==================================

DWB-style local planner for reactive obstacle avoidance and path tracking.

Algorithm Overview:
-------------------
- Samples velocity space (v, w)
- Forward-simulates trajectories
- Scores trajectories using multiple cost functions:
  - Obstacle cost
  - Path alignment cost
  - Goal distance cost
  - Smoothness cost
- Selects best velocity command

Topics:
  Subscribed:
    - /global_path: Global path from A* planner
    - /scan: LaserScan for local obstacle detection
    - /odom: Current robot pose (or use TF)
  Published:
    - /cmd_vel: Velocity commands (geometry_msgs/Twist)
    - /local_trajectory: Best trajectory (nav_msgs/Path)
    - /planner/debug/vel_samples: Debug velocity samples

Parameters:
  - max_vel_x: Maximum linear velocity (default: 0.5 m/s)
  - min_vel_x: Minimum linear velocity (default: 0.0 m/s)
  - max_vel_theta: Maximum angular velocity (default: 1.0 rad/s)
  - acc_lim_x: Linear acceleration limit (default: 0.5 m/s²)
  - acc_lim_theta: Angular acceleration limit (default: 1.0 rad/s²)
  - sim_time: Forward simulation time (default: 1.5 s)
  - sim_granularity: Simulation step size (default: 0.1 s)
  - vx_samples: Number of linear velocity samples (default: 20)
  - vtheta_samples: Number of angular velocity samples (default: 40)
  - robot_radius: Robot collision radius (default: 0.2 m)
  - local_costmap_size: Size of local costmap in meters (default: 4.0)
  - local_costmap_resolution: Costmap resolution (default: 0.05 m)
  - inflation_radius: Obstacle inflation radius (default: 0.3 m)
  - path_distance_bias: Weight for path alignment cost (default: 32.0)
  - goal_distance_bias: Weight for goal distance cost (default: 24.0)
  - obstacle_cost_bias: Weight for obstacle cost (default: 1.0)
  - forward_point_distance: Distance ahead on path to track (default: 0.325)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
import numpy as np
import math
from typing import List, Tuple, Optional, Dict
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs

from rover_navigation.costmap import LocalCostmap


class Trajectory:
    """Represents a forward-simulated trajectory."""
    
    def __init__(self, vx: float, vy: float, vtheta: float):
        self.vx = vx
        self.vy = vy  # Always 0 for differential drive
        self.vtheta = vtheta
        self.poses: List[PoseStamped] = []
        self.total_cost = float('inf')
        self.cost_breakdown = {}
    
    def add_pose(self, pose: PoseStamped):
        """Add pose to trajectory."""
        self.poses.append(pose)


class LWBPlannerNode(Node):
    """
    Local Weighted Band planner for reactive navigation.
    """
    
    def __init__(self):
        super().__init__('lwb_planner')
        
        # Velocity parameters
        self.declare_parameter('max_vel_x', 0.5)
        self.declare_parameter('min_vel_x', 0.0)
        self.declare_parameter('max_vel_theta', 1.0)
        self.declare_parameter('acc_lim_x', 0.5)
        self.declare_parameter('acc_lim_theta', 1.0)
        
        # Simulation parameters
        self.declare_parameter('sim_time', 1.5)
        self.declare_parameter('sim_granularity', 0.1)
        self.declare_parameter('vx_samples', 20)
        self.declare_parameter('vtheta_samples', 40)
        
        # Robot parameters
        self.declare_parameter('robot_radius', 0.2)
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('odom_frame', 'odom')
        
        # Costmap parameters
        self.declare_parameter('local_costmap_size', 4.0)
        self.declare_parameter('local_costmap_resolution', 0.05)
        self.declare_parameter('inflation_radius', 0.3)
        
        # Cost weights
        self.declare_parameter('path_distance_bias', 32.0)
        self.declare_parameter('goal_distance_bias', 24.0)
        self.declare_parameter('obstacle_cost_bias', 1.0)
        self.declare_parameter('forward_point_distance', 0.325)
        
        # Get parameters
        self.max_vel_x = self.get_parameter('max_vel_x').value
        self.min_vel_x = self.get_parameter('min_vel_x').value
        self.max_vel_theta = self.get_parameter('max_vel_theta').value
        self.acc_lim_x = self.get_parameter('acc_lim_x').value
        self.acc_lim_theta = self.get_parameter('acc_lim_theta').value
        self.sim_time = self.get_parameter('sim_time').value
        self.sim_granularity = self.get_parameter('sim_granularity').value
        self.vx_samples = self.get_parameter('vx_samples').value
        self.vtheta_samples = self.get_parameter('vtheta_samples').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.path_distance_bias = self.get_parameter('path_distance_bias').value
        self.goal_distance_bias = self.get_parameter('goal_distance_bias').value
        self.obstacle_cost_bias = self.get_parameter('obstacle_cost_bias').value
        self.forward_point_distance = self.get_parameter('forward_point_distance').value
        
        # Costmap
        self.local_costmap = LocalCostmap(
            self.get_parameter('local_costmap_size').value,
            self.get_parameter('local_costmap_resolution').value,
            self.get_parameter('inflation_radius').value
        )
        
        # QoS profiles
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriptions
        self.global_path_sub = self.create_subscription(
            Path,
            '/global_path',
            self.global_path_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            sensor_qos
        )
        
        self.odom_sub = self.create_subscription(
            PoseStamped,
            '/odom',
            self.odom_callback,
            sensor_qos
        )
        
        # Publications
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.local_trajectory_pub = self.create_publisher(Path, '/local_trajectory', 10)
        self.vel_samples_pub = self.create_publisher(
            Float32MultiArray,
            '/planner/debug/vel_samples',
            10
        )
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # State variables
        self.global_path = None
        self.current_scan = None
        self.current_pose = None
        self.current_vel = Twist()  # Current velocity command
        
        # Planning timer (15 Hz)
        self.planning_timer = self.create_timer(1.0 / 15.0, self.planning_callback)
        
        self.get_logger().info('LWB Planner Node initialized')
        self.get_logger().info(f'Max velocity: {self.max_vel_x} m/s, {self.max_vel_theta} rad/s')
        self.get_logger().info(f'Planning frequency: 15 Hz')
    
    def global_path_callback(self, msg: Path):
        """Receive global path from A* planner."""
        self.global_path = msg
        if len(msg.poses) > 0:
            self.get_logger().debug(f'Received global path with {len(msg.poses)} waypoints')
    
    def scan_callback(self, msg: LaserScan):
        """Receive laser scan for local obstacle detection."""
        self.current_scan = msg
        # Update local costmap
        if self.current_pose:
            self.local_costmap.update_from_scan(msg, self.current_pose)
    
    def odom_callback(self, msg: PoseStamped):
        """Receive current robot pose."""
        self.current_pose = msg
    
    def planning_callback(self):
        """
        Main planning loop - compute velocity command.
        """
        if self.current_pose is None or self.global_path is None:
            return
        
        if len(self.global_path.poses) == 0:
            # No path, stop
            self.stop_robot()
            return
        
        # Update local costmap if we have a scan
        if self.current_scan:
            self.local_costmap.update_from_scan(self.current_scan, self.current_pose)
        
        # Sample velocities
        trajectories = self.sample_velocities()
        
        # Score trajectories
        self.score_trajectories(trajectories)
        
        # Select best trajectory
        best_trajectory = self.select_best_trajectory(trajectories)
        
        if best_trajectory and best_trajectory.total_cost < float('inf'):
            # Publish velocity command
            cmd_vel = Twist()
            cmd_vel.linear.x = float(best_trajectory.vx)
            cmd_vel.angular.z = float(best_trajectory.vtheta)
            self.cmd_vel_pub.publish(cmd_vel)
            self.current_vel = cmd_vel
            
            # Publish local trajectory
            local_path = Path()
            local_path.header.frame_id = self.odom_frame
            local_path.header.stamp = self.get_clock().now().to_msg()
            local_path.poses = best_trajectory.poses
            self.local_trajectory_pub.publish(local_path)
        else:
            # No valid trajectory, stop
            self.get_logger().warn('No valid trajectory found, stopping')
            self.stop_robot()
    
    def sample_velocities(self) -> List[Trajectory]:
        """
        Sample velocity space and generate trajectories.
        
        Returns:
            List of forward-simulated trajectories
        """
        trajectories = []
        
        # Sample linear velocities
        if self.min_vel_x >= self.max_vel_x:
            vx_values = [self.max_vel_x]
        else:
            vx_values = np.linspace(self.min_vel_x, self.max_vel_x, self.vx_samples)
        
        # Sample angular velocities
        vtheta_values = np.linspace(-self.max_vel_theta, self.max_vel_theta, self.vtheta_samples)
        
        # Current pose state
        current_x = self.current_pose.pose.position.x
        current_y = self.current_pose.pose.position.y
        current_yaw = self.quaternion_to_yaw(self.current_pose.pose.orientation)
        
        # Sample and simulate
        for vx in vx_values:
            for vtheta in vtheta_values:
                # Apply acceleration limits
                vx = self.apply_acceleration_limits(vx, self.current_vel.linear.x)
                vtheta = self.apply_angular_acceleration_limits(
                    vtheta, self.current_vel.angular.z
                )
                
                # Simulate trajectory
                trajectory = self.simulate_trajectory(
                    current_x, current_y, current_yaw, vx, vtheta
                )
                trajectories.append(trajectory)
        
        return trajectories
    
    def simulate_trajectory(self, start_x: float, start_y: float, start_yaw: float,
                           vx: float, vtheta: float) -> Trajectory:
        """
        Forward simulate a trajectory given initial pose and velocities.
        
        Args:
            start_x, start_y, start_yaw: Initial pose
            vx: Linear velocity
            vtheta: Angular velocity
            
        Returns:
            Simulated trajectory
        """
        trajectory = Trajectory(vx, 0.0, vtheta)
        
        x = start_x
        y = start_y
        yaw = start_yaw
        
        # Create initial pose
        pose = PoseStamped()
        pose.header.frame_id = self.odom_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation = self.yaw_to_quaternion(yaw)
        trajectory.add_pose(pose)
        
        # Simulate for sim_time seconds
        t = 0.0
        while t < self.sim_time:
            # Differential drive kinematics
            dt = min(self.sim_granularity, self.sim_time - t)
            
            # Update pose
            if abs(vtheta) < 1e-6:
                # Straight line
                x += vx * math.cos(yaw) * dt
                y += vx * math.sin(yaw) * dt
            else:
                # Arc
                radius = vx / vtheta
                dtheta = vtheta * dt
                x += radius * (math.sin(yaw + dtheta) - math.sin(yaw))
                y += radius * (math.cos(yaw) - math.cos(yaw + dtheta))
                yaw += dtheta
            
            # Normalize yaw
            yaw = self.normalize_angle(yaw)
            
            # Add pose
            pose = PoseStamped()
            pose.header.frame_id = self.odom_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation = self.yaw_to_quaternion(yaw)
            trajectory.add_pose(pose)
            
            t += dt
        
        return trajectory
    
    def score_trajectories(self, trajectories: List[Trajectory]):
        """
        Score all trajectories using multiple cost functions.
        
        Args:
            trajectories: List of trajectories to score
        """
        for trajectory in trajectories:
            # Compute individual costs
            obstacle_cost = self.compute_obstacle_cost(trajectory)
            path_cost = self.compute_path_distance_cost(trajectory)
            goal_cost = self.compute_goal_distance_cost(trajectory)
            smoothness_cost = self.compute_smoothness_cost(trajectory)
            
            # Weighted sum
            total_cost = (
                self.obstacle_cost_bias * obstacle_cost +
                self.path_distance_bias * path_cost +
                self.goal_distance_bias * goal_cost +
                smoothness_cost  # Smoothness is usually unweighted or has weight 1.0
            )
            
            trajectory.total_cost = total_cost
            trajectory.cost_breakdown = {
                'obstacle': obstacle_cost,
                'path': path_cost,
                'goal': goal_cost,
                'smoothness': smoothness_cost,
                'total': total_cost
            }
    
    def compute_obstacle_cost(self, trajectory: Trajectory) -> float:
        """
        Compute obstacle cost (penalty for trajectories near obstacles).
        
        Returns:
            Obstacle cost (0 = no obstacles, higher = closer to obstacles)
        """
        max_cost = 0.0
        
        for pose in trajectory.poses:
            # Get cost at this pose (in local costmap coordinates)
            # Costmap origin is at robot, so we need to compute relative position
            x = pose.pose.position.x - self.current_pose.pose.position.x
            y = pose.pose.position.y - self.current_pose.pose.position.y
            
            cost = self.local_costmap.get_cost_at_world(x, y)
            
            # Convert costmap cost (0-254) to normalized cost
            if cost >= 250:  # Lethal
                return float('inf')
            elif cost > 0:
                normalized_cost = cost / 254.0
                max_cost = max(max_cost, normalized_cost)
        
        return max_cost
    
    def compute_path_distance_cost(self, trajectory: Trajectory) -> float:
        """
        Compute path alignment cost (penalty for deviating from global path).
        
        Returns:
            Path distance cost (lower is better)
        """
        if not trajectory.poses:
            return float('inf')
        
        # Use forward point on trajectory
        forward_idx = min(
            int(self.forward_point_distance / 
                (trajectory.vx * self.sim_granularity)),
            len(trajectory.poses) - 1
        )
        
        if forward_idx >= len(trajectory.poses):
            forward_idx = len(trajectory.poses) - 1
        
        traj_pose = trajectory.poses[forward_idx]
        
        # Find closest point on global path
        min_dist = float('inf')
        for path_pose in self.global_path.poses:
            dist = self.distance_between_poses(traj_pose, path_pose)
            min_dist = min(min_dist, dist)
        
        return min_dist
    
    def compute_goal_distance_cost(self, trajectory: Trajectory) -> float:
        """
        Compute goal distance cost (penalty for distance to goal).
        
        Returns:
            Goal distance cost (lower is better)
        """
        if not self.global_path.poses:
            return 0.0
        
        # Use end of trajectory
        traj_end = trajectory.poses[-1]
        goal = self.global_path.poses[-1]
        
        return self.distance_between_poses(traj_end, goal)
    
    def compute_smoothness_cost(self, trajectory: Trajectory) -> float:
        """
        Compute smoothness cost (penalty for sudden velocity changes).
        
        Returns:
            Smoothness cost
        """
        # Compare with current velocity
        dvx = abs(trajectory.vx - self.current_vel.linear.x)
        dvtheta = abs(trajectory.vtheta - self.current_vel.angular.z)
        
        # Normalize and combine
        smoothness = (dvx / self.max_vel_x) + (dvtheta / self.max_vel_theta)
        return smoothness
    
    def select_best_trajectory(self, trajectories: List[Trajectory]) -> Optional[Trajectory]:
        """
        Select trajectory with lowest cost.
        
        Args:
            trajectories: List of scored trajectories
            
        Returns:
            Best trajectory or None
        """
        if not trajectories:
            return None
        
        best = min(trajectories, key=lambda t: t.total_cost)
        return best
    
    def apply_acceleration_limits(self, target_vx: float, current_vx: float) -> float:
        """Apply linear acceleration limits."""
        max_dv = self.acc_lim_x * self.sim_granularity
        dv = target_vx - current_vx
        dv = np.clip(dv, -max_dv, max_dv)
        return current_vx + dv
    
    def apply_angular_acceleration_limits(self, target_vtheta: float, 
                                         current_vtheta: float) -> float:
        """Apply angular acceleration limits."""
        max_dv = self.acc_lim_theta * self.sim_granularity
        dv = target_vtheta - current_vtheta
        dv = np.clip(dv, -max_dv, max_dv)
        return current_vtheta + dv
    
    def stop_robot(self):
        """Stop the robot."""
        cmd_vel = Twist()
        self.cmd_vel_pub.publish(cmd_vel)
        self.current_vel = cmd_vel
    
    @staticmethod
    def distance_between_poses(pose1: PoseStamped, pose2: PoseStamped) -> float:
        """Calculate distance between two poses."""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx * dx + dy * dy)
    
    @staticmethod
    def quaternion_to_yaw(quat) -> float:
        """Convert quaternion to yaw."""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw
    
    @staticmethod
    def yaw_to_quaternion(yaw: float):
        """Convert yaw to quaternion."""
        from geometry_msgs.msg import Quaternion
        quat = Quaternion()
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2.0)
        quat.w = math.cos(yaw / 2.0)
        return quat
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None):
    rclpy.init(args=args)
    node = LWBPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

