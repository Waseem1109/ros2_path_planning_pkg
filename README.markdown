# ROS2 Path Planning with A* and D* Lite

This repository contains ROS2 nodes implementing **A*** and **D* Lite** path planning algorithms for 2D grid-based navigation. The nodes subscribe to map and goal pose topics, compute paths using the respective algorithms, and publish planned paths and inflated maps for robot navigation.

## Overview

- **A* Node**: Implements the A* algorithm for static path planning, suitable for environments where the map does not change frequently.
- **D* Lite Node**: Implements the D* Lite algorithm for dynamic path planning, optimized for environments with changing obstacles (e.g., detected via sensors).
- **ROS2 Integration**: Both nodes integrate with ROS2, subscribing to `/map` (nav_msgs/OccupancyGrid) and `/goal_pose` (geometry_msgs/PoseStamped), and publishing to `/planned_path` (nav_msgs/Path) and `/inflated_map` (nav_msgs/OccupancyGrid).

## Prerequisites

- **ROS2**: Humble (or compatible version) installed on Ubuntu (e.g., 22.04).
- **Python**: 3.10 or later (included with ROS2 Humble).
- **Dependencies**:
  - `rclpy`
  - `nav_msgs`
  - `geometry_msgs`
  - `tf2_ros`
  - `numpy`
- **Colcon**: ROS2 build tool.
- **Workspace**: A ROS2 workspace (e.g., `ros2_ws_jjsugar12`) with the `path_planning` package.

## Setup

1. **Clone the Repository**:
   ```bash
   mkdir -p ~/ros2_ws_jjsugar12/src
   cd ~/ros2_ws_jjsugar12/src
   git clone <repository-url>
   ```

2. **Build the Workspace**:
   ```bash
   cd ~/ros2_ws_jjsugar12
   colcon build --packages-select path_planning
   ```

3. **Source the Workspace**:
   ```bash
   source ~/ros2_ws_jjsugar12/install/setup.bash
   ```

## Running the Nodes

### 1. Launch the Map Server
Start the map server to publish the map to the `/map` topic:
```bash
ros2 launch path_planning map_server.launch.py
```
Ensure your map file is correctly configured in the `map_server.launch.py` file.

### 2. Set Up the Static Transform
Publish a static transform from `map` to `base_link` (adjust the transform values as needed):
```bash
ros2 run tf2_ros static_transform_publisher 1 0 0 0 0 0 map base_link
```
This provides the robot's position in the map frame. In a real system, replace this with a dynamic transform from a localization system (e.g., AMCL or SLAM).

### 3. Run the Path Planning Nodes
- **For A***:
  ```bash
  ros2 launch path_planning a_star.launch.py
  ```
- **For D* Lite**:
  ```bash
  ros2 launch path_planning d_star_lite.launch.py
  ```

### 4. Publish a Goal Pose
Use a tool like `rviz2` or a command-line publisher to send a goal pose to `/goal_pose`. For example:
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped "{header: {frame_id: 'map'}, pose: {position: {x: 5.0, y: 5.0, z: 0.0}, orientation: {w: 1.0}}}"
```

### 5. Monitor Outputs
- **Published Topics**:
  - `/planned_path` (nav_msgs/Path): The computed path from the robot's current position to the goal.
  - `/inflated_map` (nav_msgs/OccupancyGrid): The map with inflated obstacles based on the robot's radius.
- Use `rviz2` to visualize the map, path, and inflated map.
- Check terminal logs for debugging information (e.g., "Path found", "No path possible").

## Project Structure

- `path_planning/`: ROS2 package directory.
  - `path_planning/a_star.py`: A* path planning node implementation.
  - `path_planning/d_star_node_inflate.py`: D* Lite path planning node implementation.
  - `launch/a_star.launch.py`: Launch file for the A* node.
  - `launch/d_star_lite.launch.py`: Launch file for the D* Lite node.
  - `launch/map_server.launch.py`: Launch file for the map server.
- `README.md`: This file.

## Notes

- **A***: Best for static environments where obstacles do not change. It computes a path from scratch for each goal or map update.
- **D* Lite**: Optimized for dynamic environments with changing obstacles (e.g., detected via `spoofed_obstacles` or sensors). It incrementally updates the path, making it faster for replanning.
- **Parameters**:
  - `robot_radius`: Radius for inflating obstacles (default: 0.5 meters).
  - `base_frame`: Robot's base frame (default: `base_link`).
  - `map_frame`: Map frame (default: `map`).
  - `planning_frequency`: Frequency of path planning updates (default: 1.0 Hz).
- **Performance**: The D* Lite node uses `heapq` for efficient priority queue operations and resets its state when a new goal is received to prevent hangs.

## Troubleshooting

- **Node Hangs on Second Goal (D* Lite)**:
  - Ensure you are using the latest version of `d_star_node_inflate.py`, which resets the planner state in `goal_callback`.
  - Check logs for warnings like "No path possible" or "No valid successors".
- **TF2 Errors**:
  - Verify the `map` to `base_link` transform is published and valid.
  - Check the `map_frame` and `base_frame` parameters.
- **No Path Published**:
  - Confirm that `/map` and `/goal_pose` topics are receiving valid data.
  - Ensure the goal is in a free space (not inside an obstacle or inflated region).
- **Performance Issues**:
  - For large maps, reduce `planning_frequency` or map resolution.
  - Profile the code with `python3 -m cProfile -s time d_star_node_inflate.py` to identify bottlenecks.

## Contributing

Contributions are welcome! Please submit a pull request or open an issue for bugs, improvements, or feature requests.

## License

This project is licensed under the MIT License.