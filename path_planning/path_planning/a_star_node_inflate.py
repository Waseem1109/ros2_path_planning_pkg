import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import time
import tf2_ros
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped

class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('a_star_planner_node')
        # Declare parameters
        self.declare_parameter('robot_radius', 0.5)  
        self.declare_parameter('base_frame', 'base_link') 
        self.declare_parameter('map_frame', 'map')  
        self.declare_parameter('planning_frequency', 1.0)
        # Get parameters
        self.robot_radius = self.get_parameter('robot_radius').get_parameter_value().double_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.planning_frequency = self.get_parameter('planning_frequency').get_parameter_value().double_value
        self.resolution = 2.0  
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, '/inflated_map', 10)
        self.timer = self.create_timer(1.0/self.planning_frequency, self.plan_path)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.map_data = None
        self.goal_x = None   
        self.goal_y = None
        self.goal_yaw = None  
        self.min_x = 0
        self.min_y = 0
        self.max_x = 0
        self.max_y = 0
        self.x_width = 0
        self.y_width = 0
        self.obstacle_map = None
        self.inflated_map = None
        self.get_logger().info(f'A* Planner Node initialized with robot_radius={self.robot_radius}, '
                              f'base_frame={self.base_frame}, map_frame={self.map_frame}')

    class Node:
        def __init__(self, x, y, cost, parent_index):
            self.x = x  
            self.y = y  
            self.cost = cost
            self.parent_index = parent_index

        def __str__(self):
            return f"{self.x},{self.y},{self.cost},{self.parent_index}"

    def map_callback(self, msg):
        """Process incoming map data and create inflated map"""
        self.map_data = msg
        self.resolution = msg.info.resolution
        self.min_x = msg.info.origin.position.x
        self.min_y = msg.info.origin.position.y
        self.x_width = msg.info.width
        self.y_width = msg.info.height
        self.max_x = self.min_x + self.x_width * self.resolution
        self.max_y = self.min_y + self.y_width * self.resolution
        self.calc_obstacle_map(msg.data, msg.info.width, msg.info.height)
        self.calc_inflated_map()
        self.publish_inflated_map()
        self.get_logger().info('Map received, processed, and inflated')

    def goal_callback(self, msg):
        """Process incoming goal pose"""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y

        q = msg.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.goal_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.get_logger().info(f'Goal received: x={self.goal_x}, y={self.goal_y}, yaw={self.goal_yaw}')

    def calc_obstacle_map(self, map_data, width, height):
        """Convert occupancy grid to obstacle map"""
        self.obstacle_map = [[False for _ in range(self.y_width)] for _ in range(self.x_width)]
        for ix in range(self.x_width):
            for iy in range(self.y_width):
                idx = iy * width + ix
                if idx < len(map_data) and map_data[idx] > 50:  
                    self.obstacle_map[ix][iy] = True

    def calc_inflated_map(self):
        """Create inflated map based on robot radius with lower occupancy for inflated areas"""
        self.inflated_map = [[0 for _ in range(self.y_width)] for _ in range(self.x_width)]
        inflation_cells = int(math.ceil(self.robot_radius / self.resolution))

        for ix in range(self.x_width):
            for iy in range(self.y_width):
                if self.obstacle_map[ix][iy]:
                    self.inflated_map[ix][iy] = 100  
                    for dx in range(-inflation_cells, inflation_cells + 1):
                        for dy in range(-inflation_cells, inflation_cells + 1):
                            nx, ny = ix + dx, iy + dy
                            if 0 <= nx < self.x_width and 0 <= ny < self.y_width:
                                distance = math.hypot(dx * self.resolution, dy * self.resolution)
                                if distance <= self.robot_radius and self.inflated_map[nx][ny] < 100:
                                    self.inflated_map[nx][ny] = 50 

    def publish_inflated_map(self):
        """Publish the inflated map"""
        if self.map_data is None or self.inflated_map is None:
            return

        inflated_map_msg = OccupancyGrid()
        inflated_map_msg.header = self.map_data.header
        inflated_map_msg.info = self.map_data.info
        map_data = []
        for iy in range(self.y_width):
            for ix in range(self.x_width):
                map_data.append(self.inflated_map[ix][iy])
        inflated_map_msg.data = map_data
        self.inflated_map_pub.publish(inflated_map_msg)
        self.get_logger().info('Published inflated map')

    def get_base_link_position(self):
        """Get the current position of base_link in the map frame"""
        try:
            # Lookup transform from map_frame to base_frame
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
            start_x = transform.transform.translation.x
            start_y = transform.transform.translation.y
            return start_x, start_y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get transform from {self.map_frame} to {self.base_frame}: {e}')
            return None, None

    def plan_path(self):
        """Plan path using A* algorithm and publish"""
        if self.map_data is None or self.inflated_map is None:
            self.get_logger().warn('No map data available for planning')
            return

        if self.goal_x is None or self.goal_y is None:
            self.get_logger().warn('No goal pose available for planning')
            return

        # Get current base_link position
        start_x, start_y = self.get_base_link_position()
        if start_x is None or start_y is None:
            self.get_logger().warn(f'Cannot plan path: {self.base_frame} position unavailable')
            return

        rx, ry = self.planning(start_x, start_y, self.goal_x, self.goal_y)
        if rx and ry:
            path_msg = self.create_path_msg(rx, ry)
            self.path_pub.publish(path_msg)
            self.get_logger().info('Published planned path')

    def planning(self, sx, sy, gx, gy):
        """A* path planning using inflated map"""
        start_node = self.Node(self.calc_xy_index(sx, self.min_x),
                              self.calc_xy_index(sy, self.min_y), 0.0, -1)
        goal_node = self.Node(self.calc_xy_index(gx, self.min_x),
                             self.calc_xy_index(gy, self.min_y), 0.0, -1)

        open_set, closed_set = {}, {}
        open_set[self.calc_grid_index(start_node)] = start_node

        while True:
            if not open_set:
                self.get_logger().warn('Open set is empty')
                return [], []

            c_id = min(open_set, key=lambda o: open_set[o].cost + self.calc_heuristic(goal_node, open_set[o]))
            current = open_set[c_id]

            if current.x == goal_node.x and current.y == goal_node.y:
                self.get_logger().info('Goal reached')
                goal_node.parent_index = current.parent_index
                goal_node.cost = current.cost
                break

            del open_set[c_id]
            closed_set[c_id] = current

            for motion in self.get_motion_model():
                node = self.Node(current.x + motion[0], current.y + motion[1],
                                current.cost + motion[2], c_id)
                n_id = self.calc_grid_index(node)

                if not self.verify_node(node):
                    continue

                if n_id in closed_set:
                    continue

                if n_id not in open_set:
                    open_set[n_id] = node
                elif open_set[n_id].cost > node.cost:
                    open_set[n_id] = node

        rx, ry = self.calc_final_path(goal_node, closed_set)
        return rx, ry

    def calc_final_path(self, goal_node, closed_set):
        """Generate final path from goal to start"""
        rx = [self.calc_grid_position(goal_node.x, self.min_x)]
        ry = [self.calc_grid_position(goal_node.y, self.min_y)]
        parent_index = goal_node.parent_index
        while parent_index != -1:
            n = closed_set[parent_index]
            rx.append(self.calc_grid_position(n.x, self.min_x))
            ry.append(self.calc_grid_position(n.y, self.min_y))
            parent_index = n.parent_index
        return rx, ry

    def create_path_msg(self, rx, ry):
        """Convert path to nav_msgs/Path message"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame
        for x, y in zip(rx, ry):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg

    @staticmethod
    def calc_heuristic(n1, n2):
        """Calculate heuristic (Euclidean distance)"""
        w = 1.0
        d = w * math.hypot(n1.x - n2.x, n1.y - n2.y)
        return d

    def calc_grid_position(self, index, min_position):
        """Convert grid index to position"""
        return index * self.resolution + min_position

    def calc_xy_index(self, position, min_pos):
        """Convert position to grid index"""
        return round((position - min_pos) / self.resolution)

    def calc_grid_index(self, node):
        """Calculate unique grid index"""
        return (node.y - self.min_y) * self.x_width + (node.x - self.min_x)

    def verify_node(self, node):
        """Check if node is valid (within bounds and not in obstacle or inflated area)"""
        px = self.calc_grid_position(node.x, self.min_x)
        py = self.calc_grid_position(node.y, self.min_y)

        if px < self.min_x or py < self.min_y or px >= self.max_x or py >= self.max_y:
            return False

        if self.inflated_map and self.inflated_map[node.x][node.y] >= 50:  # Treat both obstacles (100) and inflated areas (50) as invalid
            return False

        return True

    @staticmethod
    def get_motion_model():
        """Define motion model (8-directional movement)"""
        return [
            [1, 0, 1],
            [0, 1, 1],
            [-1, 0, 1],
            [0, -1, 1],
            [-1, -1, math.sqrt(2)],
            [-1, 1, math.sqrt(2)],
            [1, -1, math.sqrt(2)],
            [1, 1, math.sqrt(2)]
        ]

def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()