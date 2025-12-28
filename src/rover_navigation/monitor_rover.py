#!/usr/bin/env python3
"""
Real-Time Rover Movement Monitor
==================================

Shows live: Position, Orientation, Velocity, Path Status
Updates every 0.5 seconds with nice formatting

Usage:
    python3 monitor_rover.py
    
Press Ctrl+C to stop
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String
import math
import sys
import os
from datetime import datetime


class RoverMonitor(Node):
    def __init__(self):
        super().__init__('rover_monitor')
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            PoseStamped, '/odom', self.odom_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10
        )
        self.path_sub = self.create_subscription(
            Path, '/path', self.path_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, '/plan_status', self.status_callback, 10
        )
        
        # Data storage
        self.odom_data = None
        self.cmd_vel_data = None
        self.path_data = None
        self.status_data = None
        
        # Display timer (every 0.5 seconds)
        self.display_timer = self.create_timer(0.5, self.display_callback)
        self.update_count = 0

    def odom_callback(self, msg):
        """Receive odometry updates"""
        self.odom_data = msg

    def cmd_vel_callback(self, msg):
        """Receive velocity commands"""
        self.cmd_vel_data = msg

    def path_callback(self, msg):
        """Receive planned path"""
        self.path_data = msg

    def status_callback(self, msg):
        """Receive planning status"""
        self.status_data = msg.data

    def display_callback(self):
        """Update display every 0.5 seconds"""
        self.update_count += 1
        
        # Clear screen
        os.system('clear' if os.name == 'posix' else 'cls')
        
        # Header
        print("╔" + "═" * 58 + "╗")
        print("║" + " " * 12 + "🚗 ROVER MOVEMENT MONITOR" + " " * 21 + "║")
        print("╠" + "═" * 58 + "╣")
        
        # Position and Orientation
        if self.odom_data:
            self._print_position_section()
        else:
            print("║  ⏳ Waiting for odometry data..." + " " * 24 + "║")
        
        print("╠" + "═" * 58 + "╣")
        
        # Velocity Commands
        if self.cmd_vel_data:
            self._print_velocity_section()
        else:
            print("║  ⏳ Waiting for velocity commands..." + " " * 21 + "║")
        
        print("╠" + "═" * 58 + "╣")
        
        # Path Status
        if self.path_data or self.status_data:
            self._print_status_section()
        else:
            print("║  ⏳ Waiting for planning..." + " " * 29 + "║")
        
        print("╚" + "═" * 58 + "╝")
        
        # Footer
        print(f"\n  Updates: {self.update_count}  |  Time: {datetime.now().strftime('%H:%M:%S')}  |  Press Ctrl+C to stop\n")

    def _print_position_section(self):
        """Print position and orientation"""
        pos = self.odom_data.pose.position
        ori = self.odom_data.pose.orientation
        
        # Convert quaternion to yaw angle
        yaw = self.get_yaw_from_quat(ori)
        yaw_deg = math.degrees(yaw)
        
        # Distance from origin
        distance = math.sqrt(pos.x**2 + pos.y**2)
        
        print("║  📍 POSITION & ORIENTATION:")
        print(f"║     X Position:  {pos.x:8.3f} m")
        print(f"║     Y Position:  {pos.y:8.3f} m")
        print(f"║     Z Position:  {pos.z:8.3f} m")
        print(f"║     Heading:     {yaw_deg:6.1f}° ({yaw:7.3f} rad)")
        print(f"║     Distance from origin: {distance:6.3f} m")

    def _print_velocity_section(self):
        """Print velocity commands"""
        lin_x = self.cmd_vel_data.linear.x
        lin_y = self.cmd_vel_data.linear.y
        lin_z = self.cmd_vel_data.linear.z
        ang_x = self.cmd_vel_data.angular.x
        ang_y = self.cmd_vel_data.angular.y
        ang_z = self.cmd_vel_data.angular.z
        
        # Linear velocity magnitude
        lin_mag = math.sqrt(lin_x**2 + lin_y**2 + lin_z**2)
        
        # Angular velocity magnitude
        ang_mag = math.sqrt(ang_x**2 + ang_y**2 + ang_z**2)
        
        print("║  ⚙️  VELOCITY COMMANDS:")
        print(f"║     Linear X:  {lin_x:7.3f} m/s")
        print(f"║     Linear Y:  {lin_y:7.3f} m/s")
        print(f"║     Angular Z: {ang_z:7.3f} rad/s ({math.degrees(ang_z):6.1f}°/s)")
        print(f"║     Total speed: {lin_mag:6.3f} m/s  |  Rotation: {ang_mag:6.3f} rad/s")

    def _print_status_section(self):
        """Print planning and path status"""
        status_symbol = '❓'
        status_text = 'UNKNOWN'
        
        if self.status_data:
            if self.status_data == 'SUCCESS':
                status_symbol = '✅'
                status_text = 'SUCCESS - Path planned'
            elif self.status_data == 'FAILED':
                status_symbol = '❌'
                status_text = 'FAILED - No path found'
            elif self.status_data == 'REPLANNING':
                status_symbol = '🔄'
                status_text = 'REPLANNING - Obstacles detected'
        
        waypoints = 0
        if self.path_data:
            waypoints = len(self.path_data.poses)
        
        print("║  📊 PLANNING STATUS:")
        print(f"║     Status:     {status_symbol} {status_text}")
        print(f"║     Waypoints:  {waypoints} waypoints in current path")

    @staticmethod
    def get_yaw_from_quat(quat):
        """
        Convert quaternion to yaw angle (rotation around Z axis).
        
        Args:
            quat: Quaternion with x, y, z, w components
            
        Returns:
            Yaw angle in radians
        """
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )
        return yaw


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    print("\n🚀 Starting Rover Monitor...")
    print("   Connecting to ROS 2 topics...")
    print("   Press Ctrl+C to stop\n")
    
    try:
        monitor = RoverMonitor()
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n\n👋 Monitor stopped by user")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
