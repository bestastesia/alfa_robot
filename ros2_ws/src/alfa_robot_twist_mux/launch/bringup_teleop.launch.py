from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():


    return LaunchDescription([
        Node(
            package='alfa_robot_twist_mux',
            executable='teleop_ctl_node',
        ),
    ])
