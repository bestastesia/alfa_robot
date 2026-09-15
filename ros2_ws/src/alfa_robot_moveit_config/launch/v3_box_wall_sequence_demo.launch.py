"""Continuous simulation-only 25-box transfer, using the comfort-height baseline."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("x", default_value="0.90"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory("alfa_robot_moveit_config")) /
            "launch" / "v3_box_wall_comfort_grasp_demo.launch.py")),
            launch_arguments={"x": LaunchConfiguration("x"), "sequence_mode": "true",
                              "box_id": "20"}.items()),
    ])
