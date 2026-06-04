from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _default_scene_xml():
    package_share = Path(get_package_share_directory("alfa_robot_moveit_config"))
    repo_root = package_share.parents[4]
    return str(repo_root / "simulation" / "mujoco" / "scene.xml")


def generate_launch_description():
    xml_path = LaunchConfiguration("xml_path")
    frame_id = LaunchConfiguration("frame_id")
    republish_period = LaunchConfiguration("republish_period")
    apply_on_start = LaunchConfiguration("apply_on_start")
    attach_object_id = LaunchConfiguration("attach_object_id")
    attach_link = LaunchConfiguration("attach_link")

    return LaunchDescription([
        DeclareLaunchArgument("xml_path", default_value=_default_scene_xml()),
        DeclareLaunchArgument("frame_id", default_value="base_link"),
        DeclareLaunchArgument("republish_period", default_value="1.0"),
        DeclareLaunchArgument("apply_on_start", default_value="true"),
        DeclareLaunchArgument("attach_object_id", default_value=""),
        DeclareLaunchArgument("attach_link", default_value=""),
        Node(
            package="alfa_robot_moveit_config",
            executable="mujoco_planning_scene_bridge.py",
            name="mujoco_planning_scene_bridge",
            output="screen",
            parameters=[{
                "xml_path": xml_path,
                "frame_id": frame_id,
                "republish_period": republish_period,
                "apply_on_start": apply_on_start,
                "attach_object_id": attach_object_id,
                "attach_link": attach_link,
            }],
        ),
    ])
