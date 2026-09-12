"""Single-height shoulder-to-contact strategy; old launch defaults are unchanged."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Frozen 2026-09-12: three training seeds, 0.80/0.90/1.00m; see comfort Demo report.
    defaults = {"comfort_ratio_min": "1.10", "comfort_ratio_preferred": "1.15",
                "comfort_ratio_max": "1.15", "box_id": "5"}
    return LaunchDescription([
        DeclareLaunchArgument("x", description="Metres from chassis front to wall near face; >0"),
        *[DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
            Path(get_package_share_directory("alfa_robot_moveit_config")) /
            "launch" / "v3_box_wall_grasp_demo.launch.py")),
            launch_arguments={"x": LaunchConfiguration("x"), "height_strategy": "comfort_radius",
                              **{k: LaunchConfiguration(k) for k in defaults}}.items()),
    ])
