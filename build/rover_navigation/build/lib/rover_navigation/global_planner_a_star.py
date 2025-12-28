"""
A* Global Path Planner
======================

Grid-based A* path planner for ROS2 navigation stack.

Algorithm Overview:
-------------------
- Uses 8-connected grid search on OccupancyGrid from SLAM Toolbox
- Cost function: g(n) = distance, h(n) = Euclidean heuristic
- Supports obstacle inflation for safety margins
- Replans on new goals or significant map updates

Topics:
  Subscribed:
    - /map: OccupancyGrid from SLAM Toolbox
    - /goal: Target pose (geometry_msgs/PoseStamped)
  Published:
    - /global_path: Planned path (nav_msgs/Path)

Parameters:
  - inflation_radius: Obstacle inflation radius in meters (default: 0.5)
  - allow_unknown: Allow planning through unknown cells (default: False)
  - plan_frequency: Planning frequency in Hz (default: 1.0)
  - tolerance_xy: Position tolerance for goal reaching (default: 0.2)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import String
import numpy as np
import math
import heapq
from typing import List, Tuple, Optional, Set
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs


class GridCell:
    """Represents a cell in the grid."""
    
    def __init__(self, x: int, y: int, parent=None, g_cost=float('inf'), h_cost=0.0):
        self.x = x
        self.y = y
        self.parent = parent
        self.g_cost = g_cost  # Cost from start
        self.h_cost = h_cost  # Heuristic to goal
        self.f_cost = g_cost + h_cost  # Total cost
    
    def __lt__(self, other):
        """Comparison for priority queue."""
        return self.f_cost < other.f_cost
    
    def __eq__(self, other):
        """Equality check."""
        return self.x == other.x and self.y == other.y
    
    def __hash__(self):
        """Hash for set operations."""
        return hash((self.x, self.y))


class AStarPlannerNode(Node):
    """
    A* global path planner using grid-based search.
    """
    
    # 8-connected neighbors: [dx, dy, cost]
    NEIGHBORS = [
        (0, 1, 1.0),    # North
        (1, 0, 1.0),    # East
        (0, -1, 1.0),   # South
        (-1, 0, 1.0),   # West
        (1, 1, 1.414),  # Northeast
        (1, -1, 1.414), # Southeast
        (-1, -1, 1.414), # Southwest
        (-1, 1, 1.414), # Northwest
    ]
    
    def __init__(self):
        super().__init__('a_star_planner')
        
        # Parameters
        self.declare_parameter('inflation_radius', 0.5)
        self.declare_parameter('allow_unknown', False)
        self.declare_parameter('plan_frequency', 1.0)
        self.declare_parameter('tolerance_xy', 0.2)
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        
        self.inflation_radius = self.get_parameter('inflation_radius').value
        self.allow_unknown = self.get_parameter('allow_unknown').value
        self.plan_freq = self.get_parameter('plan_frequency').value
        self.tolerance = self.get_parameter('tolerance_xy').value
        self.robot_frame = self.get_parameter('robot_frame').value
        self.map_frame = self.get_parameter('map_frame').value
        
        # QoS for map (latched)
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscriptions
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            map_qos
        )
        
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal',
            self.goal_callback,
            10
        )
        
        # Publications
        self.path_pub = self.create_publisher(Path, '/global_path', 10)
        self.status_pub = self.create_publisher(String, '/planner/debug/state', 10)
        
        # TF for getting robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # State variables
        self.occupancy_grid = None
        self.grid_metadata = None  # width, height, resolution, origin
        self.inflated_grid = None
        self.current_goal = None
        self.current_path = None
        
        # Planning timer
        self.planning_timer = self.create_timer(
            1.0 / self.plan_freq,
            self.planning_callback
        )
        
        self.get_logger().info('A* Global Planner Node initialized')
        self.get_logger().info(f'Inflation radius: {self.inflation_radius}m')
        self.get_logger().info(f'Planning frequency: {self.plan_freq}Hz')
    
    def map_callback(self, msg: OccupancyGrid):
        """
        Process incoming map from SLAM Toolbox.
        
        Args:
            msg: OccupancyGrid message
        """
        self.get_logger().info(
            f'Received map: {msg.info.width}x{msg.info.height}, '
            f'resolution: {msg.info.resolution}m/cell'
        )
        
        self.occupancy_grid = np.array(msg.data, dtype=np.int8).reshape(
            (msg.info.height, msg.info.width)
        )
        
        self.grid_metadata = {
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'origin_yaw': self.quaternion_to_yaw(msg.info.origin.orientation)
        }
        
        # Inflate obstacles
        self.inflated_grid = self.inflate_obstacles(
            self.occupancy_grid,
            self.inflation_radius,
            msg.info.resolution
        )
        
        # Trigger replanning if we have a goal
        if self.current_goal:
            self.plan_path()
    
    def goal_callback(self, msg: PoseStamped):
        """
        Receive new goal pose.
        
        Args:
            msg: Goal pose
        """
        self.current_goal = msg
        self.get_logger().info(
            f'Received goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )
        
        # Trigger planning
        if self.occupancy_grid is not None:
            self.plan_path()
    
    def planning_callback(self):
        """
        Periodic planning callback.
        Checks if replanning is needed and plans if necessary.
        """
        if self.current_goal is None or self.occupancy_grid is None:
            return
        
        # For now, just trigger replanning if we have a goal
        # More sophisticated replanning logic can be added here
        pass
    
    def plan_path(self):
        """
        Execute A* algorithm to find path from current pose to goal.
        """
        if self.current_goal is None or self.occupancy_grid is None:
            return
        
        # Get current robot pose
        current_pose = self.get_robot_pose()
        if current_pose is None:
            self.get_logger().warn('Cannot get robot pose, skipping planning')
            self.publish_status('WAITING_FOR_POSE')
            return
        
        # Convert world coordinates to grid indices
        start_x, start_y = self.world_to_grid(
            current_pose.pose.position.x,
            current_pose.pose.position.y
        )
        goal_x, goal_y = self.world_to_grid(
            self.current_goal.pose.position.x,
            self.current_goal.pose.position.y
        )
        
        # Validate start and goal
        if not self.is_valid_cell(start_x, start_y):
            self.get_logger().error(f'Start cell ({start_x}, {start_y}) is invalid')
            self.publish_status('INVALID_START')
            return
        
        if not self.is_valid_cell(goal_x, goal_y):
            self.get_logger().error(f'Goal cell ({goal_x}, {goal_y}) is invalid')
            self.publish_status('INVALID_GOAL')
            return
        
        # Run A* algorithm
        path_cells = self.a_star_search(start_x, start_y, goal_x, goal_y)
        
        if path_cells is None or len(path_cells) == 0:
            self.get_logger().warn('A* search failed to find path')
            self.publish_status('FAILED')
            self.current_path = None
            self.path_pub.publish(Path())  # Publish empty path
            return
        
        # Convert grid path to world coordinates
        path = self.grid_path_to_world_path(path_cells)
        self.current_path = path
        self.get_logger().info(
            f'Path found: {len(path.poses)} waypoints, '
            f'length: {self.compute_path_length(path):.2f}m'
        )
        
        # Publish path
        self.path_pub.publish(path)
        self.publish_status('ACTIVE')
    
    def a_star_search(self, start_x: int, start_y: int, goal_x: int, goal_y: int) -> Optional[List[Tuple[int, int]]]:
        """
        A* search algorithm.
        
        Args:
            start_x, start_y: Start cell coordinates
            goal_x, goal_y: Goal cell coordinates
            
        Returns:
            List of (x, y) cell coordinates from start to goal, or None if no path
        """
        # Initialize open and closed sets
        open_set = []  # Priority queue (min heap)
        closed_set: Set[Tuple[int, int]] = set()
        
        # Initialize start cell
        start_cell = GridCell(start_x, start_y, None, 0.0, 
                             self.euclidean_heuristic(start_x, start_y, goal_x, goal_y))
        heapq.heappush(open_set, (start_cell.f_cost, start_cell))
        
        # Dictionary to track best costs for each cell
        cell_map = {(start_x, start_y): start_cell}
        
        while open_set:
            # Get cell with lowest f_cost
            _, current = heapq.heappop(open_set)
            
            # Check if goal reached
            if current.x == goal_x and current.y == goal_y:
                # Reconstruct path
                path = []
                cell = current
                while cell is not None:
                    path.append((cell.x, cell.y))
                    cell = cell.parent
                path.reverse()
                return path
            
            # Add to closed set
            closed_set.add((current.x, current.y))
            
            # Explore neighbors
            for dx, dy, move_cost in self.NEIGHBORS:
                nx = current.x + dx
                ny = current.y + dy
                
                # Skip if invalid or in closed set
                if not self.is_valid_cell(nx, ny) or (nx, ny) in closed_set:
                    continue
                
                # Calculate tentative g_cost
                tentative_g = current.g_cost + move_cost * self.grid_metadata['resolution']
                
                # Check if we've seen this cell before
                if (nx, ny) in cell_map:
                    neighbor_cell = cell_map[(nx, ny)]
                    if tentative_g >= neighbor_cell.g_cost:
                        continue  # Not a better path
                else:
                    # Create new cell
                    neighbor_cell = GridCell(nx, ny)
                    cell_map[(nx, ny)] = neighbor_cell
                
                # Update cell
                neighbor_cell.parent = current
                neighbor_cell.g_cost = tentative_g
                neighbor_cell.h_cost = self.euclidean_heuristic(nx, ny, goal_x, goal_y)
                neighbor_cell.f_cost = neighbor_cell.g_cost + neighbor_cell.h_cost
                
                # Add to open set
                heapq.heappush(open_set, (neighbor_cell.f_cost, neighbor_cell))
        
        # No path found
        return None
    
    def is_valid_cell(self, x: int, y: int) -> bool:
        """
        Check if grid cell is valid (within bounds and not occupied).
        
        Args:
            x, y: Grid cell coordinates
            
        Returns:
            True if valid, False otherwise
        """
        if self.inflated_grid is None or self.grid_metadata is None:
            return False
        
        # Check bounds
        if x < 0 or x >= self.grid_metadata['width']:
            return False
        if y < 0 or y >= self.grid_metadata['height']:
            return False
        
        # Check occupancy (in inflated grid)
        # Occupied = 100, Free = 0, Unknown = -1
        cell_value = self.inflated_grid[y, x]
        
        if cell_value >= 50:  # Occupied or inflated
            return False
        
        if cell_value == -1 and not self.allow_unknown:  # Unknown
            return False
        
        return True
    
    def euclidean_heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        """
        Euclidean distance heuristic.
        
        Args:
            x1, y1: First cell coordinates
            x2, y2: Second cell coordinates
            
        Returns:
            Euclidean distance in meters
        """
        dx = (x2 - x1) * self.grid_metadata['resolution']
        dy = (y2 - y1) * self.grid_metadata['resolution']
        return math.sqrt(dx * dx + dy * dy)
    
    def world_to_grid(self, world_x: float, world_y: float) -> Tuple[int, int]:
        """
        Convert world coordinates to grid cell indices.
        
        Args:
            world_x, world_y: World coordinates in meters
            
        Returns:
            Grid cell (x, y) indices
        """
        # Transform to map frame (accounting for origin)
        dx = world_x - self.grid_metadata['origin_x']
        dy = world_y - self.grid_metadata['origin_y']
        
        # Rotate if origin has rotation (simplified - assumes no rotation)
        # grid_x = int((dx * math.cos(-origin_yaw) - dy * math.sin(-origin_yaw)) / resolution)
        # grid_y = int((dx * math.sin(-origin_yaw) + dy * math.cos(-origin_yaw)) / resolution)
        
        # Simple version (assuming no rotation)
        grid_x = int(dx / self.grid_metadata['resolution'])
        grid_y = int(dy / self.grid_metadata['resolution'])
        
        return grid_x, grid_y
    
    def grid_to_world(self, grid_x: int, grid_y: int) -> Tuple[float, float]:
        """
        Convert grid cell indices to world coordinates.
        
        Args:
            grid_x, grid_y: Grid cell indices
            
        Returns:
            World coordinates (x, y) in meters
        """
        world_x = grid_x * self.grid_metadata['resolution'] + self.grid_metadata['origin_x']
        world_y = grid_y * self.grid_metadata['resolution'] + self.grid_metadata['origin_y']
        return world_x, world_y
    
    def inflate_obstacles(self, grid: np.ndarray, radius: float, resolution: float) -> np.ndarray:
        """
        Inflate obstacles in the grid by a given radius.
        
        Args:
            grid: Original occupancy grid
            radius: Inflation radius in meters
            resolution: Grid resolution in meters per cell
            
        Returns:
            Inflated occupancy grid
        """
        inflated = grid.copy()
        radius_cells = int(math.ceil(radius / resolution))
        
        # Find all obstacle cells
        height, width = grid.shape
        obstacle_cells = []
        for y in range(height):
            for x in range(width):
                if grid[y, x] >= 50:  # Occupied
                    obstacle_cells.append((x, y))
        
        # Inflate each obstacle
        for obs_x, obs_y in obstacle_cells:
            # Check all cells within inflation radius
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    nx = obs_x + dx
                    ny = obs_y + dy
                    
                    if not (0 <= nx < width and 0 <= ny < height):
                        continue
                    
                    # Calculate distance from obstacle center
                    dist = math.sqrt((dx * resolution) ** 2 + (dy * resolution) ** 2)
                    
                    if dist <= radius:
                        # Mark as inflated (use 50 for inflated obstacles)
                        if inflated[ny, nx] < 50:
                            inflated[ny, nx] = 50
        
        return inflated
    
    def grid_path_to_world_path(self, path_cells: List[Tuple[int, int]]) -> Path:
        """
        Convert grid cell path to world coordinate Path message.
        
        Args:
            path_cells: List of (x, y) grid cell coordinates
            
        Returns:
            Path message with world coordinates
        """
        path = Path()
        path.header.frame_id = self.map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        
        for grid_x, grid_y in path_cells:
            world_x, world_y = self.grid_to_world(grid_x, grid_y)
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0  # Default orientation
            path.poses.append(pose)
        
        return path
    
    def compute_path_length(self, path: Path) -> float:
        """Compute total length of path in meters."""
        if len(path.poses) < 2:
            return 0.0
        
        total_length = 0.0
        for i in range(len(path.poses) - 1):
            p1 = path.poses[i].pose.position
            p2 = path.poses[i + 1].pose.position
            dx = p2.x - p1.x
            dy = p2.y - p1.y
            total_length += math.sqrt(dx * dx + dy * dy)
        
        return total_length
    
    def get_robot_pose(self) -> Optional[PoseStamped]:
        """
        Get current robot pose from TF.
        
        Returns:
            PoseStamped in map frame, or None if transform unavailable
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.robot_frame,
                rclpy.time.Time()
            )
            
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
        except Exception as e:
            self.get_logger().debug(f'TF lookup failed: {str(e)}')
            return None
    
    def quaternion_to_yaw(self, quat) -> float:
        """Convert quaternion to yaw angle."""
        x, y, z, w = quat.x, quat.y, quat.z, quat.w
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw
    
    def publish_status(self, status: str):
        """Publish planner status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

