from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Declare launch arguments (these can be overridden from command line)
            DeclareLaunchArgument(
                "start_x", default_value="0.5", description="Start X coordinate"
            ),
            DeclareLaunchArgument(
                "start_y", default_value="0.5", description="Start Y coordinate"
            ),
            DeclareLaunchArgument(
                "goal_x",
                default_value="4.0",
                description="Goal X coordinate (center of circle)",
            ),
            DeclareLaunchArgument(
                "goal_y",
                default_value="4.0",
                description="Goal Y coordinate (center of circle)",
            ),
            DeclareLaunchArgument(
                "circle_radius",
                default_value="1.0",
                description="Radius of circular path around goal",
            ),
            DeclareLaunchArgument(
                "num_loops",
                default_value="2",
                description="Number of times to circle around goal",
            ),
            DeclareLaunchArgument(
                "circle_points",
                default_value="32",
                description="Number of points per circle (higher = smoother)",
            ),
            DeclareLaunchArgument(
                "publish_rate",
                default_value="2.0",
                description="Rate to republish path (Hz)",
            ),
            DeclareLaunchArgument(
                "map_file",
                default_value="maps/blank_map.pgm",
                description="Path to map file",
            ),
            DeclareLaunchArgument(
                "grid_resolution",
                default_value="0.05",
                description="Grid resolution for planning",
            ),
            # Launch the path planner node with parameters
            Node(
                package="path_planning",
                executable="path_planner",
                name="path_planner",
                output="screen",
                parameters=[
                    {
                        "start_x": LaunchConfiguration("start_x"),
                        "start_y": LaunchConfiguration("start_y"),
                        "goal_x": LaunchConfiguration("goal_x"),
                        "goal_y": LaunchConfiguration("goal_y"),
                        "circle_radius": LaunchConfiguration("circle_radius"),
                        "num_loops": LaunchConfiguration("num_loops"),
                        "circle_points": LaunchConfiguration("circle_points"),
                        "publish_rate": LaunchConfiguration("publish_rate"),
                        "map_file": LaunchConfiguration("map_file"),
                        "grid_resolution": LaunchConfiguration("grid_resolution"),
                    }
                ],
            ),
        ]
    )
