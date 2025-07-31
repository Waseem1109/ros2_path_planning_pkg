import os
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, LogInfo
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Declare the map_name argument
    map_name = LaunchConfiguration("map_name")
    ld.add_action(
        DeclareLaunchArgument(
            "map_name",
            default_value="test_map.yaml",
            description="Default map_name value is 'map.yaml'"
        )
    )

    # Define the map server config path
    map_server_config_path = PathJoinSubstitution([
        get_package_prefix('path_planning').replace("/install/", "/src/"),
        'maps',
        map_name
    ])

    # Log the resolved map path
    ld.add_action(
        LogInfo(
            msg=["Resolved map path: ", map_server_config_path]
        )
    )

    # Map server node
    map_server_cmd = Node(
        package='nav2_map_server',
        executable='map_server',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{'yaml_filename': map_server_config_path}]
    )

    # Lifecycle manager node
    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True
    start_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        output='screen',
        emulate_tty=True,
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes}
        ]
    )

    # Add actions to launch description
    ld.add_action(map_server_cmd)
    ld.add_action(start_lifecycle_manager_cmd)

    return ld