"""
Obstacle Detection Node
========================

This node processes LiDAR scan data and detects obstacles.

Functionality:
- Subscribes to /scan (sensor_msgs/LaserScan)
- Detects obstacles based on distance threshold
- Converts scan points to 2D Cartesian coordinates
- Publishes detected obstacles on /obstacles (custom message)
- Maintains obstacle map in occupancy grid

Topics:
  Subscribed:
    - /scan: LaserScan data from LiDAR
  Published:
    - /obstacles: List of detected obstacle positions
    - /occupancy_grid: 2D grid representation of obstacles

Parameters:
  - obstacle_distance_threshold: Minimum distance to consider as obstacle (default: 1.5m)
  - max_scan_range: Maximum range to process (default: 10.0m)
  - grid_size: Size of occupancy grid (default: 20m x 20m)
  - grid_resolution: Resolution in meters per cell (default: 0.1m)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
import numpy as np
import math


class ObstacleDetectionNode(Node):
    """
    Detects obstacles from LiDAR scan data and publishes obstacle positions.
    """

    def __init__(self):
        super().__init__('obstacle_detection')
        
        # Parameters
        self.declare_parameter('obstacle_distance_threshold', 1.5)
        self.declare_parameter('max_scan_range', 10.0)
        self.declare_parameter('grid_size', 20.0)
        self.declare_parameter('grid_resolution', 0.1)
        self.declare_parameter('frame_id', 'laser')
        
        self.threshold = self.get_parameter('obstacle_distance_threshold').value
        self.max_range = self.get_parameter('max_scan_range').value
        self.grid_size = self.get_parameter('grid_size').value
        self.grid_res = self.get_parameter('grid_resolution').value
        self.frame_id = self.get_parameter('frame_id').value
        
        # QoS Profile for sensor data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriptions
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )
        
        # Publications
        self.obstacles_pub = self.create_publisher(
            PoseStamped,
            '/obstacles',
            10
        )
        
        self.occupancy_grid_pub = self.create_publisher(
            OccupancyGrid,
            '/occupancy_grid',
            10
        )
        
        # Storage for obstacle data
        self.obstacles = []
        
        # Initialize occupancy grid
        self.grid_width = int(self.grid_size / self.grid_res)
        self.occupancy_grid_data = np.zeros(
            (self.grid_width, self.grid_width),
            dtype=np.int8
        )
        
        self.get_logger().info('Obstacle Detection Node initialized')
        self.get_logger().info(f'Obstacle threshold: {self.threshold}m')
        self.get_logger().info(f'Max scan range: {self.max_range}m')

    def scan_callback(self, scan_msg: LaserScan):
        """
        Process incoming LiDAR scan data.
        
        Algorithm:
        1. Filter scan points based on distance threshold
        2. Convert polar coordinates to Cartesian (x, y)
        3. Filter points beyond max_range
        4. Update occupancy grid
        5. Publish detected obstacles
        
        Args:
            scan_msg: LaserScan message from LiDAR
        """
        self.obstacles = []
        
        # Reset occupancy grid
        self.occupancy_grid_data.fill(0)
        
        # Process each scan angle
        for i, range_val in enumerate(scan_msg.ranges):
            # Skip invalid measurements
            if math.isnan(range_val) or math.isinf(range_val):
                continue
            
            # Skip if beyond max range or below threshold (too close is invalid)
            if range_val < scan_msg.range_min or range_val > self.max_range:
                continue
            
            # Calculate angle for this beam
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            
            # Convert to Cartesian coordinates (LiDAR frame)
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            
            # Detect obstacle if within threshold
            if range_val < self.threshold:
                self.obstacles.append((x, y, range_val))
                self._update_occupancy_grid(x, y, occupied=True)
            else:
                # Mark as free space if not an obstacle
                self._update_occupancy_grid(x, y, occupied=False)
        
        # Publish occupancy grid
        self._publish_occupancy_grid(scan_msg.header.stamp)
        
        # Publish detected obstacles
        self._publish_obstacles(scan_msg.header)

    def _update_occupancy_grid(self, x: float, y: float, occupied: bool):
        """
        Update occupancy grid with point measurement.
        
        Grid coordinate system:
        - Origin at center of grid
        - x increases to the right
        - y increases upward
        
        Args:
            x: X coordinate in LiDAR frame
            y: Y coordinate in LiDAR frame
            occupied: True if obstacle, False for free space
        """
        # Convert to grid indices
        # Offset by half grid size since origin is at center
        grid_x = int((x + self.grid_size / 2) / self.grid_res)
        grid_y = int((y + self.grid_size / 2) / self.grid_res)
        
        # Check bounds
        if 0 <= grid_x < self.grid_width and 0 <= grid_y < self.grid_width:
            if occupied:
                self.occupancy_grid_data[grid_y, grid_x] = 100  # Occupied
            else:
                # Only update to free if not already occupied
                if self.occupancy_grid_data[grid_y, grid_x] != 100:
                    self.occupancy_grid_data[grid_y, grid_x] = 0  # Free

    def _publish_occupancy_grid(self, timestamp):
        """
        Publish occupancy grid for visualization and planning.
        
        Args:
            timestamp: ROS timestamp for the message
        """
        grid_msg = OccupancyGrid()
        grid_msg.header = Header(
            stamp=timestamp,
            frame_id=self.frame_id
        )
        
        # Set metadata
        grid_msg.info = MapMetaData()
        grid_msg.info.map_load_time = timestamp
        grid_msg.info.resolution = self.grid_res
        grid_msg.info.width = self.grid_width
        grid_msg.info.height = self.grid_width
        
        # Origin at center of grid (bottom-left in standard occupancy grid)
        origin = PoseStamped()
        origin.header = grid_msg.header
        origin.pose.position.x = -self.grid_size / 2
        origin.pose.position.y = -self.grid_size / 2
        origin.pose.orientation.w = 1.0
        grid_msg.info.origin = origin.pose
        
        # Flatten grid data (occupancy grid uses row-major order)
        grid_msg.data = self.occupancy_grid_data.flatten().tolist()
        
        self.occupancy_grid_pub.publish(grid_msg)

    def _publish_obstacles(self, header: Header):
        """
        Publish detected obstacles as point cloud.
        
        Args:
            header: Header with frame_id and timestamp
        """
        for x, y, distance in self.obstacles:
            obstacle_msg = PoseStamped()
            obstacle_msg.header = header
            obstacle_msg.pose.position.x = float(x)
            obstacle_msg.pose.position.y = float(y)
            obstacle_msg.pose.position.z = 0.0
            obstacle_msg.pose.orientation.w = 1.0
            
            self.obstacles_pub.publish(obstacle_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
