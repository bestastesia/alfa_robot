"""Launch file for box_perception node."""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('box_perception')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    perception_node = Node(
        package='box_perception',
        executable='perception_node',
        name='box_perception_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([perception_node])
    