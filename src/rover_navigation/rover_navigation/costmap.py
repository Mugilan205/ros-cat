"""
Costmap Utilities
=================

Utilities for creating and managing costmaps for local planning.
Supports both global costmaps (from OccupancyGrid) and local rolling costmaps (from LaserScan).
"""

import numpy as np
import math
from typing import Tuple, Optional
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped


class Costmap:
    """
    Base costmap class for obstacle representation.
    """
    
    # Cost values
    FREE_SPACE = 0
    LETHAL_OBSTACLE = 254
    INSCRIBED_OBSTACLE = 253
    INFLATED_OBSTACLE = 128
    UNKNOWN = -1
    
    def __init__(self, width: int, height: int, resolution: float, 
                 origin_x: float, origin_y: float):
        """
        Initialize costmap.
        
        Args:
            width: Width in cells
            height: Height in cells
            resolution: Resolution in meters per cell
            origin_x: Origin x coordinate in world frame
            origin_y: Origin y coordinate in world frame
        """
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        
        # Initialize cost grid (int8, -1 to 255)
        self.costmap = np.full((height, width), self.FREE_SPACE, dtype=np.int8)
    
    def world_to_map(self, wx: float, wy: float) -> Tuple[int, int]:
        """Convert world coordinates to map indices."""
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        return mx, my
    
    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """Convert map indices to world coordinates."""
        wx = mx * self.resolution + self.origin_x
        wy = my * self.resolution + self.origin_y
        return wx, wy
    
    def is_valid_index(self, mx: int, my: int) -> bool:
        """Check if map indices are valid."""
        return 0 <= mx < self.width and 0 <= my < self.height
    
    def get_cost(self, mx: int, my: int) -> int:
        """Get cost at map coordinates."""
        if not self.is_valid_index(mx, my):
            return self.LETHAL_OBSTACLE
        return int(self.costmap[my, mx])
    
    def set_cost(self, mx: int, my: int, cost: int):
        """Set cost at map coordinates."""
        if self.is_valid_index(mx, my):
            self.costmap[my, mx] = np.clip(cost, -1, 255)
    
    def inflate_obstacles(self, inflation_radius: float, inscribed_radius: float = 0.0):
        """
        Inflate obstacles in the costmap.
        
        Args:
            inflation_radius: Radius for inflation in meters
            inscribed_radius: Radius of inscribed circle (robot radius)
        """
        inflation_cells = int(math.ceil(inflation_radius / self.resolution))
        inscribed_cells = int(math.ceil(inscribed_radius / self.resolution))
        
        # Find all obstacle cells
        obstacle_cells = []
        for y in range(self.height):
            for x in range(self.width):
                if self.costmap[y, x] >= self.LETHAL_OBSTACLE:
                    obstacle_cells.append((x, y))
        
        # Create temporary grid for inflated costs
        inflated = self.costmap.copy()
        
        # Inflate each obstacle
        for obs_x, obs_y in obstacle_cells:
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    mx = obs_x + dx
                    my = obs_y + dy
                    
                    if not self.is_valid_index(mx, my):
                        continue
                    
                    # Calculate distance
                    dist = math.sqrt((dx * self.resolution) ** 2 + 
                                    (dy * self.resolution) ** 2)
                    
                    if dist <= inscribed_radius:
                        # Lethal zone
                        inflated[my, mx] = self.LETHAL_OBSTACLE
                    elif dist <= inflation_radius:
                        # Inflated zone - cost decreases with distance
                        cost = int(self.INFLATED_OBSTACLE * 
                                  (1.0 - (dist - inscribed_radius) / 
                                   (inflation_radius - inscribed_radius)))
                        if inflated[my, mx] < cost:
                            inflated[my, mx] = cost
        
        self.costmap = inflated


class GlobalCostmap(Costmap):
    """
    Global costmap from OccupancyGrid (typically from SLAM).
    """
    
    def __init__(self, occupancy_grid: OccupancyGrid, inflation_radius: float = 0.5):
        """
        Create global costmap from OccupancyGrid.
        
        Args:
            occupancy_grid: OccupancyGrid message
            inflation_radius: Inflation radius in meters
        """
        width = occupancy_grid.info.width
        height = occupancy_grid.info.height
        resolution = occupancy_grid.info.resolution
        origin_x = occupancy_grid.info.origin.position.x
        origin_y = occupancy_grid.info.origin.position.y
        
        super().__init__(width, height, resolution, origin_x, origin_y)
        
        # Copy occupancy data to costmap
        grid_data = np.array(occupancy_grid.data, dtype=np.int8).reshape((height, width))
        
        for y in range(height):
            for x in range(width):
                val = grid_data[y, x]
                if val >= 50:  # Occupied
                    self.costmap[y, x] = self.LETHAL_OBSTACLE
                elif val == -1:  # Unknown
                    self.costmap[y, x] = self.UNKNOWN
                else:  # Free
                    self.costmap[y, x] = self.FREE_SPACE
        
        # Inflate obstacles
        if inflation_radius > 0:
            self.inflate_obstacles(inflation_radius)


class LocalCostmap(Costmap):
    """
    Rolling window costmap from LaserScan data.
    """
    
    def __init__(self, size: float, resolution: float, inflation_radius: float = 0.3):
        """
        Create local rolling costmap.
        
        Args:
            size: Size of costmap in meters (square)
            resolution: Resolution in meters per cell
            inflation_radius: Inflation radius in meters
        """
        width = height = int(size / resolution)
        # Origin at center so robot (0,0) maps to costmap center
        origin_x = -size / 2.0
        origin_y = -size / 2.0
        
        super().__init__(width, height, resolution, origin_x, origin_y)
        
        self.inflation_radius = inflation_radius
        self.size = size
    
    def update_from_scan(self, scan: LaserScan, robot_pose: Optional[PoseStamped] = None):
        """
        Update costmap from LaserScan data.
        
        Args:
            scan: LaserScan message
            robot_pose: Current robot pose (optional, for frame transforms)
        """
        # Clear costmap (mark as free)
        self.costmap.fill(self.FREE_SPACE)
        
        # Process scan points
        for i, range_val in enumerate(scan.ranges):
            # Skip invalid measurements
            if (math.isnan(range_val) or math.isinf(range_val) or
                range_val < scan.range_min or range_val > scan.range_max):
                continue
            
            # Calculate angle
            angle = scan.angle_min + i * scan.angle_increment
            
            # Convert to Cartesian (in laser frame)
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)
            
            # Laser points are in laser frame, which should align with robot/base_link frame
            # For local costmap, robot is at (0, 0), so laser points map directly
            # Costmap origin is at (-size/2, -size/2), so (0,0) maps to center
            mx, my = self.world_to_map(x, y)
            
            # Mark obstacle cell
            if self.is_valid_index(mx, my):
                self.costmap[my, mx] = self.LETHAL_OBSTACLE
        
        # Inflate obstacles
        if self.inflation_radius > 0:
            self.inflate_obstacles(self.inflation_radius)
    
    def get_cost_at_world(self, wx: float, wy: float) -> int:
        """
        Get cost at world coordinates (relative to costmap origin).
        
        Args:
            wx, wy: World coordinates
            
        Returns:
            Cost value
        """
        mx, my = self.world_to_map(wx, wy)
        return self.get_cost(mx, my)
    
    def get_cost_along_line(self, x1: float, y1: float, x2: float, y2: float, 
                           num_samples: int = 20) -> int:
        """
        Get maximum cost along a line segment.
        
        Args:
            x1, y1: Start point
            x2, y2: End point
            num_samples: Number of samples along line
            
        Returns:
            Maximum cost along line
        """
        max_cost = self.FREE_SPACE
        
        for i in range(num_samples + 1):
            t = i / num_samples
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            cost = self.get_cost_at_world(x, y)
            max_cost = max(max_cost, cost)
        
        return max_cost

