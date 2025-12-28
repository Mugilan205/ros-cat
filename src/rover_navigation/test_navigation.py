#!/usr/bin/env python3
"""
Test Script for Rover Navigation System
========================================

This script tests the rover navigation system by:
1. Publishing simulated LiDAR scans with obstacles
2. Sending goal positions
3. Monitoring the planner and executor nodes

Usage:
    python3 test_navigation.py [options]

Options:
    --scenario [1|2|3]  Choose test scenario
    --verbose          Enable debug output
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path
from std_msgs.msg import String, Header
import math
import time
import argparse
import numpy as np


class NavigationTester(Node):
    """Test rover navigation with simulated LiDAR data."""
    
    def __init__(self, scenario=1, verbose=False):
        super().__init__('navigation_tester')
        self.scenario = scenario
        self.verbose = verbose
        
        # Publishers
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal', 10)
        self.odom_pub = self.create_publisher(PoseStamped, '/odom', 10)
        
        # Subscribers
        self.path_sub = self.create_subscription(Path, '/path', self.path_callback, 10)
        self.status_sub = self.create_subscription(String, '/plan_status', self.status_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        
        # State
        self.current_path = None
        self.plan_status = None
        self.last_cmd_vel = None
        
        # Test parameters
        self.robot_x = 0.0
        self.robot_y = 0.0
        
        # Timer for publishing data
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.timer_counter = 0
        
        self.get_logger().info(f'Navigation Tester initialized (Scenario {scenario})')

    def path_callback(self, msg: Path):
        """Receive planned path."""
        self.current_path = msg
        if msg.poses and self.verbose:
            self.get_logger().info(f'Received path with {len(msg.poses)} waypoints')

    def status_callback(self, msg: String):
        """Receive planning status."""
        self.plan_status = msg.data
        self.get_logger().info(f'Plan Status: {msg.data}')

    def cmd_vel_callback(self, msg: Twist):
        """Monitor velocity commands."""
        self.last_cmd_vel = msg
        if self.verbose:
            self.get_logger().debug(
                f'Cmd Vel: linear={msg.linear.x:.2f}, angular={msg.angular.z:.2f}'
            )

    def timer_callback(self):
        """Main test loop."""
        if self.scenario == 1:
            self.scenario_1_simple_goal()
        elif self.scenario == 2:
            self.scenario_2_obstacle_avoidance()
        elif self.scenario == 3:
            self.scenario_3_dynamic_obstacles()

    def scenario_1_simple_goal(self):
        """Test 1: Simple path to goal without obstacles."""
        self.get_logger().info('=== Scenario 1: Simple Goal ===')
        
        # Phase 1: Publish goal (first 20 iterations)
        if self.timer_counter < 20:
            self.publish_goal(5.0, 0.0)
            self.publish_lidar_scan(no_obstacles=True)
            self.publish_odometry(self.robot_x, self.robot_y)
            
            if self.timer_counter == 0:
                self.get_logger().info('Sending goal: (5.0, 0.0)')
        
        # Phase 2: Monitor path execution
        elif self.timer_counter < 40:
            self.publish_lidar_scan(no_obstacles=True)
            self.publish_odometry(self.robot_x, self.robot_y)
            
            if self.current_path:
                self.get_logger().info(f'Path available, {len(self.current_path.poses)} waypoints')
        
        # Phase 3: Complete
        else:
            self.get_logger().info('Scenario 1 complete')
            self.timer_counter = -1
        
        self.timer_counter += 1

    def scenario_2_obstacle_avoidance(self):
        """Test 2: Path planning with static obstacles."""
        self.get_logger().info('=== Scenario 2: Obstacle Avoidance ===')
        
        # Publish goal
        if self.timer_counter < 20:
            self.publish_goal(8.0, 0.0)
            self.publish_lidar_scan(obstacle_x=2.0, obstacle_y=0.0)
            self.publish_odometry(self.robot_x, self.robot_y)
            
            if self.timer_counter == 0:
                self.get_logger().info('Sending goal: (8.0, 0.0) with obstacle at (2.0, 0.0)')
        
        # Monitor planning
        elif self.timer_counter < 40:
            self.publish_lidar_scan(obstacle_x=2.0, obstacle_y=0.0)
            self.publish_odometry(self.robot_x, self.robot_y)
            
            if self.current_path:
                # Check if path avoids obstacle
                min_dist = self.min_distance_to_obstacle(self.current_path, 2.0, 0.0)
                self.get_logger().info(
                    f'Path generated, min distance to obstacle: {min_dist:.2f}m'
                )
        
        else:
            self.get_logger().info('Scenario 2 complete')
            self.timer_counter = -1
        
        self.timer_counter += 1

    def scenario_3_dynamic_obstacles(self):
        """Test 3: Dynamic re-planning when obstacles appear."""
        self.get_logger().info('=== Scenario 3: Dynamic Obstacles ===')
        
        # Phase 1: Send goal
        if self.timer_counter < 10:
            self.publish_goal(10.0, 0.0)
            self.publish_lidar_scan(no_obstacles=True)
            self.publish_odometry(self.robot_x, self.robot_y)
            
            if self.timer_counter == 0:
                self.get_logger().info('Phase 1: Sending goal (10.0, 0.0) - no obstacles')
        
        # Phase 2: Wait for initial path
        elif self.timer_counter < 25:
            self.publish_lidar_scan(no_obstacles=True)
            self.publish_odometry(self.robot_x, self.robot_y)
            
            if self.timer_counter == 10 and self.current_path:
                self.get_logger().info(f'Phase 1 Complete: Initial path has {len(self.current_path.poses)} waypoints')
        
        # Phase 3: Introduce obstacles (should trigger replanning)
        elif self.timer_counter < 40:
            if self.timer_counter == 25:
                self.get_logger().info('Phase 2: Introducing obstacle at (5.0, 0.0) - triggering replanning')
            
            self.publish_lidar_scan(obstacle_x=5.0, obstacle_y=0.0)
            self.publish_odometry(self.robot_x, self.robot_y)
            
            if self.plan_status == 'REPLANNING':
                self.get_logger().warn('Replanning detected!')
        
        else:
            self.get_logger().info('Scenario 3 complete')
            self.timer_counter = -1
        
        self.timer_counter += 1

    def publish_goal(self, x: float, y: float):
        """Publish goal position."""
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.goal_pub.publish(goal)

    def publish_odometry(self, x: float, y: float):
        """Publish robot odometry."""
        odom = PoseStamped()
        odom.header.frame_id = 'odom'
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.pose.position.x = x
        odom.pose.position.y = y
        odom.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)

    def publish_lidar_scan(self, no_obstacles=False, obstacle_x=None, obstacle_y=None):
        """
        Publish simulated LiDAR scan.
        
        Simulates a 270-degree LiDAR with obstacles.
        """
        scan = LaserScan()
        scan.header.frame_id = 'laser'
        scan.header.stamp = self.get_clock().now().to_msg()
        
        # LiDAR parameters
        scan.angle_min = -math.pi / 2
        scan.angle_max = math.pi / 2
        scan.angle_increment = 0.01
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.1
        scan.range_max = 10.0
        
        # Generate ranges
        num_beams = int((scan.angle_max - scan.angle_min) / scan.angle_increment) + 1
        ranges = []
        
        for i in range(num_beams):
            angle = scan.angle_min + i * scan.angle_increment
            
            if no_obstacles:
                # No obstacles, return max range
                ranges.append(scan.range_max)
            else:
                # Check distance to obstacle
                range_val = scan.range_max
                
                if obstacle_x is not None and obstacle_y is not None:
                    # Distance from LiDAR origin to obstacle
                    obs_dist = math.sqrt(obstacle_x ** 2 + obstacle_y ** 2)
                    obs_angle = math.atan2(obstacle_y, obstacle_x)
                    
                    # Check if beam points toward obstacle (within beam width)
                    angle_diff = abs(angle - obs_angle)
                    if angle_diff < 0.1:  # 10-degree field of view
                        range_val = obs_dist
                
                ranges.append(range_val)
        
        scan.ranges = ranges
        self.scan_pub.publish(scan)

    @staticmethod
    def min_distance_to_obstacle(path: Path, obs_x: float, obs_y: float) -> float:
        """Calculate minimum distance from path to obstacle."""
        min_dist = float('inf')
        
        for pose in path.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            dist = math.sqrt((x - obs_x) ** 2 + (y - obs_y) ** 2)
            min_dist = min(min_dist, dist)
        
        return min_dist


def main():
    parser = argparse.ArgumentParser(description='Test rover navigation system')
    parser.add_argument('--scenario', type=int, default=1, choices=[1, 2, 3],
                       help='Test scenario')
    parser.add_argument('--verbose', action='store_true', help='Enable debug output')
    args = parser.parse_args()
    
    rclpy.init()
    tester = NavigationTester(scenario=args.scenario, verbose=args.verbose)
    
    print(f'\n{"="*60}')
    print(f'Starting Navigation Test - Scenario {args.scenario}')
    print(f'{"="*60}')
    print('Press Ctrl+C to stop\n')
    
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        print('\nTest stopped by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
