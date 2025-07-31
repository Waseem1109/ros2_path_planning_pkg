from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('path_planning')
    param_file = os.path.join(pkg_share, 'params', 'astar_params.yaml')

    return LaunchDescription([
        Node(
            package='path_planning',
            executable='a_star_node',
            name='a_star_node',
            output='screen',
            parameters=[param_file]
        )
    ])