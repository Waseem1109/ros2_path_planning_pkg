"""
Path Planner node
"""

import heapq
from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Tuple
import os
import time

import numpy as np
import numpy.typing as npt
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


@dataclass
class Point2D:
    """2D point matching controller's Point2D structure"""

    x: float
    y: float


@dataclass
class Trajectory:
    """Consistent trajectory format with controller"""

    path: List[Point2D]
    visited_index: int = 0


class PathPlanner(Node):
    """A* path planner with strict typing matching controller conventions"""

    def __init__(self) -> None:
        super().__init__("path_planner")

        self.declare_parameter("start_x", 0.5)
        self.declare_parameter("start_y", 0.5)
        self.declare_parameter("goal_x", 5.0)
        self.declare_parameter("goal_y", 5.0)
        self.declare_parameter("map_file", "maps/blank_map.pgm")
        self.declare_parameter("grid_resolution", 0.05)
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("circle_radius", 1.0)
        self.declare_parameter("num_loops", 1)
        self.declare_parameter("circle_points", 32)

        # Type-annotated member variables
        self.last_compute_time: float = 0.0
        self.grid: npt.NDArray[np.uint8] = np.array([])
        self.origin: Tuple[float, float] = (0.0, 0.0)
        self.resolution: float = self.get_parameter("grid_resolution").value
        self.current_trajectory: Optional[Trajectory] = None

        # ALL Publishers should be created in __init__
        self.path_pub = self.create_publisher(
            Path,
            "/planned_path",
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
        )
        self.metrics_pub = self.create_publisher(
            DiagnosticArray, "/planning_metrics", 10
        )
        self.path_points_pub = self.create_publisher(PoseArray, "/path_points", 10)

        # Add marker publishers for start/goal visualization
        self.marker_pub = self.create_publisher(MarkerArray, "/path_markers", 10)

        # Create timer for continuous publishing
        self.timer = self.create_timer(
            1.0 / self.get_parameter("publish_rate").value, self.timer_callback
        )

        # Load map with proper path handling
        map_file = self.get_parameter("map_file").value

        # Handle relative path for map file
        if not os.path.isabs(map_file):
            # Get the package share directory
            from ament_index_python.packages import get_package_share_directory

            package_share_directory = get_package_share_directory("path_planning")
            map_file = os.path.join(package_share_directory, map_file)

        self.load_map(map_file)

        # Execute planning once on startup
        self.execute_planning()

    def timer_callback(self):
        """Continuously publish the path for PlotJuggler"""
        if self.current_trajectory is not None:
            self.publish_path(self.current_trajectory)
            self.publish_metrics(self.current_trajectory)
            self.publish_path_points(self.current_trajectory)
            self.publish_markers()  # NEW
            self.get_logger().debug("Republished path data")

    def publish_markers(self):
        """Publish start and goal markers for visualization"""
        marker_array = MarkerArray()

        # Start marker (green sphere)
        start_marker = Marker()
        start_marker.header.frame_id = "my_bot/odom"
        start_marker.header.stamp = self.get_clock().now().to_msg()
        start_marker.ns = "path_planning"
        start_marker.id = 0
        start_marker.type = Marker.SPHERE
        start_marker.action = Marker.ADD
        start_marker.pose.position.x = self.get_parameter("start_x").value
        start_marker.pose.position.y = self.get_parameter("start_y").value
        start_marker.pose.position.z = 0.0
        start_marker.pose.orientation.w = 1.0
        start_marker.scale.x = 0.2
        start_marker.scale.y = 0.2
        start_marker.scale.z = 0.2
        start_marker.color.a = 1.0
        start_marker.color.r = 0.0
        start_marker.color.g = 1.0
        start_marker.color.b = 0.0
        marker_array.markers.append(start_marker)

        # Start text label
        start_text = Marker()
        start_text.header.frame_id = "my_bot/odom"
        start_text.header.stamp = self.get_clock().now().to_msg()
        start_text.ns = "path_planning"
        start_text.id = 1
        start_text.type = Marker.TEXT_VIEW_FACING
        start_text.action = Marker.ADD
        start_text.pose.position.x = self.get_parameter("start_x").value
        start_text.pose.position.y = self.get_parameter("start_y").value + 0.3
        start_text.pose.position.z = 0.0
        start_text.text = "START"
        start_text.scale.z = 0.3
        start_text.color.a = 1.0
        start_text.color.r = 0.0
        start_text.color.g = 1.0
        start_text.color.b = 0.0
        marker_array.markers.append(start_text)

        # Goal marker (red sphere)
        goal_marker = Marker()
        goal_marker.header.frame_id = "my_bot/odom"
        goal_marker.header.stamp = self.get_clock().now().to_msg()
        goal_marker.ns = "path_planning"
        goal_marker.id = 2
        goal_marker.type = Marker.SPHERE
        goal_marker.action = Marker.ADD
        goal_marker.pose.position.x = self.get_parameter("goal_x").value
        goal_marker.pose.position.y = self.get_parameter("goal_y").value
        goal_marker.pose.position.z = 0.0
        goal_marker.pose.orientation.w = 1.0
        goal_marker.scale.x = 0.2
        goal_marker.scale.y = 0.2
        goal_marker.scale.z = 0.2
        goal_marker.color.a = 1.0
        goal_marker.color.r = 1.0
        goal_marker.color.g = 0.0
        goal_marker.color.b = 0.0
        marker_array.markers.append(goal_marker)

        # Goal text label
        goal_text = Marker()
        goal_text.header.frame_id = "my_bot/odom"
        goal_text.header.stamp = self.get_clock().now().to_msg()
        goal_text.ns = "path_planning"
        goal_text.id = 3
        goal_text.type = Marker.TEXT_VIEW_FACING
        goal_text.action = Marker.ADD
        goal_text.pose.position.x = self.get_parameter("goal_x").value
        goal_text.pose.position.y = self.get_parameter("goal_y").value + 0.3
        goal_text.pose.position.z = 0.0
        goal_text.text = "GOAL"
        goal_text.scale.z = 0.3
        goal_text.color.a = 1.0
        goal_text.color.r = 1.0
        goal_text.color.g = 0.0
        goal_text.color.b = 0.0
        marker_array.markers.append(goal_text)

        self.marker_pub.publish(marker_array)

    def execute_planning(self):
        """Plan path to circle, then follow circular path"""
        start = Point2D(
            self.get_parameter("start_x").value, self.get_parameter("start_y").value
        )
        goal_center = Point2D(
            self.get_parameter("goal_x").value, self.get_parameter("goal_y").value
        )

        # Get parameters
        circle_radius = self.get_parameter("circle_radius").value
        num_loops = self.get_parameter("num_loops").value
        circle_points_per_loop = self.get_parameter("circle_points").value

        # **STEP 1: Generate circle points around target**
        circle_points = self.generate_circle_points(
            goal_center, circle_radius, circle_points_per_loop
        )

        # **STEP 2: Find closest valid circle point to start**
        best_circle_point = self.find_closest_valid_circle_point(start, circle_points)

        if best_circle_point is None:
            self.get_logger().error("No valid circle points found")
            return

        # **STEP 3: Plan A* path to the circle entry point**
        if trajectory := self.a_star(start, best_circle_point):

            # **STEP 4: Add circular path starting from entry point**
            # Rotate circle to start from the entry point
            rotated_circle = self.rotate_circle_to_start(
                circle_points, best_circle_point
            )

            # Add multiple loops
            for loop in range(num_loops):
                trajectory.path.extend(rotated_circle)

            self.current_trajectory = trajectory
            self.get_logger().info(
                f"Planned path: A* to circle + {num_loops} loops (radius {circle_radius})"
            )
        else:
            self.get_logger().error("A* planning to circle failed")

    def load_map(self, filename: str) -> None:
        """Load PGM map with obstacle data"""
        try:
            with open(filename, "rb") as f:
                # Read PGM header
                if f.readline().decode().strip() != "P5":
                    raise ValueError("Invalid PGM format")

                # Skip comments
                while True:
                    line = f.readline()
                    if not line.startswith(b"#"):
                        f.seek(f.tell() - len(line))
                        break

                # Read dimensions
                width, height = map(int, f.readline().decode().split())
                f.readline()  # Skip max value

                # Load grid data
                self.grid = np.frombuffer(f.read(), dtype=np.uint8)
                self.grid = self.grid.reshape((height, width))

                self.get_logger().info(f"Loaded map: {width}x{height} from {filename}")

        except FileNotFoundError:
            self.get_logger().error(f"Map file {filename} not found")
            raise
        except Exception as e:
            self.get_logger().error(f"Error loading map {filename}: {str(e)}")
            raise

    def is_obstacle(self, x: int, y: int) -> bool:
        """Check grid cell for obstacles using standard threshold"""
        if 0 <= x < self.grid.shape[1] and 0 <= y < self.grid.shape[0]:
            return self.grid[y, x] > 127  # OccupancyGrid convention
        return True  # Out of bounds is considered an obstacle

    def valid_position(self, pos: Tuple[int, int]) -> bool:
        """Check if position is within bounds and obstacle-free"""
        x, y = pos
        return (
            0 <= x < self.grid.shape[1]
            and 0 <= y < self.grid.shape[0]
            and not self.is_obstacle(x, y)
        )

    def world_to_grid(self, point: Point2D) -> Tuple[int, int]:
        """Convert world coordinates to grid indices"""
        return (
            int((point.x - self.origin[0]) / self.resolution),
            int((point.y - self.origin[1]) / self.resolution),
        )

    def grid_to_world(self, x: int, y: int) -> Point2D:
        """Convert grid indices to world coordinates"""
        return Point2D(
            x=(x * self.resolution) + self.origin[0],
            y=(y * self.resolution) + self.origin[1],
        )

    def calculate_clearance(self, trajectory: Trajectory) -> float:
        """Calculate minimum distance to nearest obstacle along path"""
        if len(trajectory.path) == 0:
            return 0.0

        min_clearance = float("inf")

        for point in trajectory.path:
            grid_x, grid_y = self.world_to_grid(point)

            # Check 3x3 area around each path point
            for dx in [-1, 0, 1]:
                for dy in [-1, 0, 1]:
                    nx = grid_x + dx
                    ny = grid_y + dy
                    if self.is_obstacle(nx, ny):
                        distance = math.hypot(
                            dx * self.resolution, dy * self.resolution
                        )
                        min_clearance = min(min_clearance, distance)

        return min_clearance if min_clearance != float("inf") else -1.0

    def find_closest_valid_circle_point(
        self, start: Point2D, circle_points: List[Point2D]
    ) -> Optional[Point2D]:
        """Find the closest valid circle point to start position"""
        best_point = None
        min_distance = float("inf")

        for point in circle_points:
            grid_pos = self.world_to_grid(point)
            if self.valid_position(grid_pos):
                distance = math.hypot(point.x - start.x, point.y - start.y)
                if distance < min_distance:
                    min_distance = distance
                    best_point = point

        return best_point

    def rotate_circle_to_start(
        self, circle_points: List[Point2D], start_point: Point2D
    ) -> List[Point2D]:
        """Rotate circle to start from the point closest to start_point"""
        if not circle_points:
            return circle_points

        # Find the exact point in circle_points that matches start_point
        best_index = 0
        min_distance = float("inf")

        for i, point in enumerate(circle_points):
            distance = math.hypot(point.x - start_point.x, point.y - start_point.y)
            if distance < min_distance:
                min_distance = distance
                best_index = i

        # Return circle starting from best_index
        return circle_points[best_index:] + circle_points[:best_index]

    def generate_circle_points(
        self, center: Point2D, radius: float, num_points: int = 32
    ) -> List[Point2D]:
        """Generate points on a circle around the center"""
        circle_points = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center.x + radius * math.cos(angle)
            y = center.y + radius * math.sin(angle)
            circle_points.append(Point2D(x, y))
        return circle_points

    def a_star(self, start: Point2D, goal: Point2D) -> Optional[Trajectory]:
        start_time = self.get_clock().now()

        # Initialize trajectory as None
        trajectory: Optional[Trajectory] = None

        # Convert to grid coordinates
        start_grid = self.world_to_grid(start)
        goal_grid = self.world_to_grid(goal)

        if not self.valid_position(start_grid):
            self.get_logger().error(f"Start position invalid: {start_grid}")
            return None
        if not self.valid_position(goal_grid):
            self.get_logger().error(f"Goal position invalid: {goal_grid}")
            return None

        # 8-direction movements with costs
        movements: List[Tuple[int, int, float]] = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (1, 1, 1.414),
            (1, -1, 1.414),
            (-1, 1, 1.414),
            (-1, -1, 1.414),
        ]

        open_set: List[Tuple[float, Tuple[int, int]]] = []
        heapq.heappush(open_set, (0.0, start_grid))

        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]] = {}
        g_score: Dict[Tuple[int, int], float] = {start_grid: 0.0}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal_grid:
                path = self.reconstruct_path(came_from, current)
                trajectory = Trajectory(path=path)
                break  # Exit loop when path found

            for dx, dy, cost in movements:
                neighbor = (current[0] + dx, current[1] + dy)

                if not self.valid_position(neighbor):
                    continue

                tentative_g = g_score[current] + cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score, neighbor))

        end_time = self.get_clock().now()
        self.last_compute_time = (end_time - start_time).nanoseconds / 1e9

        if trajectory:  # Now properly defined in all cases
            return trajectory
        else:
            self.get_logger().warn("No valid path found")
            return None

    def heuristic(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Euclidean distance heuristic"""
        return np.hypot(a[0] - b[0], a[1] - b[1])

    def reconstruct_path(
        self,
        came_from: Dict[Tuple[int, int], Optional[Tuple[int, int]]],
        current: Tuple[int, int],
    ) -> List[Point2D]:
        """Rebuild path from came_from map"""
        path: List[Tuple[int, int]] = []
        while current in came_from:
            path.append(current)
            current = came_from[current]  # type: ignore
        path.reverse()

        return [self.grid_to_world(x, y) for x, y in path]

    def publish_path(self, trajectory: Trajectory) -> None:
        """Convert to ROS Path message for controller compatibility"""
        msg = Path()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "my_bot/odom"

        for point in trajectory.path:
            pose = PoseStamped()
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = 0.0  # Add z coordinate
            msg.poses.append(pose)

        self.path_pub.publish(msg)

    def publish_metrics(self, trajectory: Trajectory):
        """Publish planning metrics for PlotJuggler"""
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "path_planning_metrics"
        status.level = DiagnosticStatus.OK

        # Add key metrics
        status.values.append(
            KeyValue(key="path_length", value=str(len(trajectory.path)))
        )
        status.values.append(
            KeyValue(key="computation_time", value=str(self.last_compute_time))
        )
        status.values.append(
            KeyValue(
                key="obstacle_clearance",
                value=str(self.calculate_clearance(trajectory)),
            )
        )

        msg.status.append(status)
        self.metrics_pub.publish(msg)

    def publish_path_points(self, trajectory: Trajectory):
        """Publish path coordinates for XY plotting"""
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "my_bot/odom"

        for point in trajectory.path:
            pose = PoseStamped()
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            pose.pose.position.z = 0.0  # Add z coordinate
            pose_array.poses.append(pose.pose)

        self.path_points_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)

    planner = PathPlanner()

    # Keep the node alive for PlotJuggler
    rclpy.spin(planner)

    planner.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
