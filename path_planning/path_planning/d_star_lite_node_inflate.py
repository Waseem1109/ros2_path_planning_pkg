import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import math
import numpy as np
import tf2_ros
import heapq

class DStarLitePlannerNode(Node):
    class Node:
        def __init__(self, x: int = 0, y: int = 0, cost: float = 0.0):
            self.x = x
            self.y = y
            self.cost = cost

    def __init__(self):
        super().__init__('d_star_lite_planner_node')
        
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
        
        # ROS2 Subscriptions and Publishers
        qos = rclpy.qos.QoSProfile(depth=10, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT)
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.inflated_map_pub = self.create_publisher(OccupancyGrid, '/inflated_map', 10)
        self.timer = self.create_timer(1.0/self.planning_frequency, self.plan_path)

        # TF2 for robot position
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # D* Lite specific variables
        self.motions = [
            self.Node(1, 0, 1),
            self.Node(0, 1, 1),
            self.Node(-1, 0, 1),
            self.Node(0, -1, 1),
            self.Node(1, 1, math.sqrt(2)),
            self.Node(1, -1, math.sqrt(2)),
            self.Node(-1, 1, math.sqrt(2)),
            self.Node(-1, -1, math.sqrt(2))
        ]
        
        self.map_data = None
        self.start = self.Node(0, 0)
        self.goal = self.Node(0, 0)
        self.goal_x = None
        self.goal_y = None
        self.U = []  # Priority queue using heapq
        self.km = 0.0
        self.kold = (0.0, 0.0)
        self.rhs = None
        self.g = None
        self.detected_obstacles_xy = set()
        self.initialized = False
        self.last = None
        self.spoofed_obstacles = []
        self.min_x = 0
        self.min_y = 0
        self.x_width = 0
        self.y_width = 0
        self.resolution = 2.0
        self.inflated_map = None
        self.get_logger().info(f'D* Lite Planner Node initialized with robot_radius={self.robot_radius}, '
                              f'base_frame={self.base_frame}, map_frame={self.map_frame}')

    def add_coordinates(self, node1: Node, node2: Node):
        new_node = self.Node()
        new_node.x = node1.x + node2.x
        new_node.y = node1.y + node2.y
        new_node.cost = node1.cost + node2.cost
        return new_node

    def compare_coordinates(self, node1: Node, node2: Node):
        return node1.x == node2.x and node1.y == node2.y

    def create_grid(self, val: float):
        return np.full((self.x_width, self.y_width), val)

    def is_obstacle(self, node: Node):
        if self.inflated_map is None:
            return False
        return self.inflated_map[node.x][node.y] >= 50

    def c(self, node1: Node, node2: Node):
        if self.is_obstacle(node2):
            return math.inf
        new_node = self.Node(node1.x - node2.x, node1.y - node2.y)
        detected_motion = next((motion for motion in self.motions
                               if self.compare_coordinates(motion, new_node)), None)
        return detected_motion.cost if detected_motion else math.inf

    def h(self, s: Node):
        return max(abs(self.start.x - s.x), abs(self.start.y - s.y))

    def calculate_key(self, s: Node):
        return (min(self.g[s.x][s.y], self.rhs[s.x][s.y]) + self.h(s) + self.km,
                min(self.g[s.x][s.y], self.rhs[s.x][s.y]))

    def is_valid(self, node: Node):
        return 0 <= node.x < self.x_width and 0 <= node.y < self.y_width

    def get_neighbours(self, u: Node):
        return [self.add_coordinates(u, motion) for motion in self.motions
                if self.is_valid(self.add_coordinates(u, motion))]

    def pred(self, u: Node):
        return self.get_neighbours(u)

    def succ(self, u: Node):
        return self.get_neighbours(u)

    def initialize(self):
        self.initialized = True
        self.U = []
        self.km = 0.0
        self.kold = (0.0, 0.0)
        self.rhs = self.create_grid(math.inf)
        self.g = self.create_grid(math.inf)
        self.rhs[self.goal.x][self.goal.y] = 0
        heapq.heappush(self.U, (self.calculate_key(self.goal), id(self.goal), self.goal))
        self.detected_obstacles_xy = set()
        self.last = self.start

    def update_vertex(self, u: Node):
        if not self.compare_coordinates(u, self.goal):
            self.rhs[u.x][u.y] = min(self.c(u, sprime) + self.g[sprime.x][sprime.y]
                                     for sprime in self.succ(u))
        # Remove u from U by filtering and re-heapifying
        self.U = [(key, node_id, node) for key, node_id, node in self.U
                  if not self.compare_coordinates(node, u)]
        heapq.heapify(self.U)
        if self.g[u.x][u.y] != self.rhs[u.x][u.y]:
            heapq.heappush(self.U, (self.calculate_key(u), id(u), u))

    def compare_keys(self, key_pair1: tuple[float, float], key_pair2: tuple[float, float]):
        return key_pair1[0] < key_pair2[0] or \
               (key_pair1[0] == key_pair2[0] and key_pair1[1] < key_pair2[1])

    def compute_shortest_path(self):
        while self.U:
            key, node_id, u = self.U[0]
            if not (self.compare_keys(key, self.calculate_key(self.start)) or
                    self.rhs[self.start.x][self.start.y] != self.g[self.start.x][self.start.y]):
                break
            self.kold = key
            heapq.heappop(self.U)
            if self.compare_keys(self.kold, self.calculate_key(u)):
                heapq.heappush(self.U, (self.calculate_key(u), id(u), u))
            elif self.g[u.x][u.y] > self.rhs[u.x][u.y]:
                self.g[u.x][u.y] = self.rhs[u.x][u.y]
                for s in self.pred(u):
                    self.update_vertex(s)
            else:
                self.g[u.x][u.y] = math.inf
                for s in self.pred(u) + [u]:
                    self.update_vertex(s)

    def detect_changes(self):
        changed_vertices = []
        if self.spoofed_obstacles:
            for spoofed_obstacle in self.spoofed_obstacles[0]:
                if self.compare_coordinates(spoofed_obstacle, self.start) or \
                   self.compare_coordinates(spoofed_obstacle, self.goal):
                    continue
                changed_vertices.append(spoofed_obstacle)
                self.detected_obstacles_xy.add((spoofed_obstacle.x, spoofed_obstacle.y))
            self.spoofed_obstacles.pop(0)
        return changed_vertices

    def compute_current_path(self):
        path = []
        current_point = self.Node(self.start.x, self.start.y)
        visited = set()  # To prevent infinite loops
        while not self.compare_coordinates(current_point, self.goal):
            if (current_point.x, current_point.y) in visited:
                self.get_logger().warn("Cycle detected in path computation")
                break
            visited.add((current_point.x, current_point.y))
            path.append(current_point)
            successors = self.succ(current_point)
            if not successors:
                self.get_logger().warn("No valid successors in path computation")
                break
            current_point = min(successors,
                              key=lambda sprime: self.c(current_point, sprime) + self.g[sprime.x][sprime.y])
        path.append(self.goal)
        return path

    def map_callback(self, msg):
        """Process incoming map data and create inflated map"""
        self.map_data = msg
        self.resolution = msg.info.resolution
        self.min_x = msg.info.origin.position.x
        self.min_y = msg.info.origin.position.y
        self.x_width = msg.info.width
        self.y_width = msg.info.height
        self.calc_inflated_map(msg.data)
        self.publish_inflated_map()
        self.get_logger().info('Map received, processed, and inflated')

    def calc_inflated_map(self, map_data):
        """Create inflated map based on robot radius"""
        self.inflated_map = [[0 for _ in range(self.y_width)] for _ in range(self.x_width)]
        inflation_cells = int(math.ceil(self.robot_radius / self.resolution))

        for ix in range(self.x_width):
            for iy in range(self.y_width):
                idx = iy * self.x_width + ix
                if idx < len(map_data) and map_data[idx] > 50:
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

    def goal_callback(self, msg):
        """Process incoming goal pose and reset planner state"""
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        new_goal = self.Node(self.calc_xy_index(self.goal_x, self.min_x),
                            self.calc_xy_index(self.goal_y, self.min_y))
        
        # Reset planner state if goal changes
        if not self.compare_coordinates(new_goal, self.goal):
            self.goal = new_goal
            self.initialized = False  # Force reinitialization
            self.U = []
            self.km = 0.0
            self.kold = (0.0, 0.0)
            self.rhs = None
            self.g = None
            self.detected_obstacles_xy = set()
            self.last = None
            self.get_logger().info(f'New goal received: x={self.goal_x}, y={self.goal_y}, planner state reset')

    def get_base_link_position(self):
        """Get the current position of base_link in the map frame"""
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame,
                                                      rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
            start_x = transform.transform.translation.x
            start_y = transform.transform.translation.y
            return start_x, start_y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(f'Could not get transform from {self.map_frame} to {self.base_frame}: {e}')
            return None, None

    def calc_xy_index(self, position, min_pos):
        """Convert position to grid index"""
        return round((position - min_pos) / self.resolution)

    def plan_path(self):
        """Plan path using D* Lite algorithm and publish"""
        if self.map_data is None or self.inflated_map is None:
            self.get_logger().warn('No map data available for planning')
            return

        if self.goal_x is None or self.goal_y is None:
            self.get_logger().warn('No goal pose available for planning')
            return

        start_x, start_y = self.get_base_link_position()
        if start_x is None or start_y is None:
            self.get_logger().warn(f'Cannot plan path: {self.base_frame} position unavailable')
            return

        self.start = self.Node(self.calc_xy_index(start_x, self.min_x),
                             self.calc_xy_index(start_y, self.min_y))
        
        success, pathx, pathy = self.main()
        if success:
            path_msg = self.create_path_msg(pathx, pathy)
            self.path_pub.publish(path_msg)
            self.get_logger().info('Published planned path')

    def create_path_msg(self, rx, ry):
        """Convert path to nav_msgs/Path message"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.map_frame
        for x, y in zip(rx, ry):
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = x * self.resolution + self.min_x
            pose.pose.position.y = y * self.resolution + self.min_y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        return path_msg

    def main(self):
        pathx = [self.start.x]
        pathy = [self.start.y]
        if not self.initialized:
            self.initialize()
        self.compute_shortest_path()

        while not self.compare_coordinates(self.goal, self.start):
            if self.g[self.start.x][self.start.y] == math.inf:
                self.get_logger().warn("No path possible")
                return False, pathx, pathy
            successors = self.succ(self.start)
            if not successors:
                self.get_logger().warn("No valid successors")
                return False, pathx, pathy
            self.start = min(successors,
                           key=lambda sprime: self.c(self.start, sprime) + self.g[sprime.x][sprime.y])
            pathx.append(self.start.x)
            pathy.append(self.start.y)
            changed_vertices = self.detect_changes()
            if changed_vertices:
                self.get_logger().info("New obstacle detected")
                self.km += self.h(self.last)
                self.last = self.start
                for u in changed_vertices:
                    if self.compare_coordinates(u, self.start):
                        continue
                    self.rhs[u.x][u.y] = math.inf
                    self.g[u.x][u.y] = math.inf
                    self.update_vertex(u)
                self.compute_shortest_path()
        self.get_logger().info("Path found")
        return True, pathx, pathy

def main(args=None):
    rclpy.init(args=args)
    node = DStarLitePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()