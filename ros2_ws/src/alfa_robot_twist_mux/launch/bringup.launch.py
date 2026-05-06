from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('alfa_robot_twist_mux')

    twist_mux_config = os.path.join(
        pkg_share,
        'config',
        'twist_mux.yaml'
    )

    return LaunchDescription([

        # twist_mux
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            parameters=[twist_mux_config],
	    remappings=[
		('/cmd_vel_out', '/cmd_vel')
	    ],
            output='screen'
        ),

        # estop node
        Node(
            package='alfa_robot_twist_mux',
            executable='estop_node',
        ),
    ])
