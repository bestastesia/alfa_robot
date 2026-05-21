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
    joint_states = LaunchConfiguration("joint_states")
    initial_positions = LaunchConfiguration("initial_positions")
    robot_mode = LaunchConfiguration("robot_mode")
    scene_model_mode = LaunchConfiguration("scene_model_mode")
    scene_rate = LaunchConfiguration("scene_rate")
    pointcloud_rate = LaunchConfiguration("pointcloud_rate")
    viewer_rate = LaunchConfiguration("viewer_rate")
    physics_rate = LaunchConfiguration("physics_rate")

    return LaunchDescription([
        DeclareLaunchArgument("xml_path", default_value=_default_scene_xml()),
        DeclareLaunchArgument("joint_states", default_value="/joint_states"),
        DeclareLaunchArgument("initial_positions", default_value=str(
            Path(get_package_share_directory("alfa_robot_moveit_config")) / "config" / "initial_positions.yaml"
        )),
        DeclareLaunchArgument("robot_mode", default_value="kinematic"),
        DeclareLaunchArgument("scene_model_mode", default_value="semantic"),
        DeclareLaunchArgument("scene_rate", default_value="1.0"),
        DeclareLaunchArgument("pointcloud_rate", default_value="0.0"),
        DeclareLaunchArgument("viewer_rate", default_value="30.0"),
        DeclareLaunchArgument("physics_rate", default_value="200.0"),
        Node(
            package="alfa_robot_moveit_config",
            executable="mujoco_sync_bridge.py",
            name="mujoco_sync_bridge",
            output="screen",
            arguments=[
                "--xml", xml_path,
                "--joint-states", joint_states,
                "--initial-positions", initial_positions,
                "--robot-mode", robot_mode,
                "--rate", viewer_rate,
                "--physics-rate", physics_rate,
                "--scene-rate", scene_rate,
                "--scene-model-mode", scene_model_mode,
                "--pointcloud-rate", pointcloud_rate,
                "--frame-id", "base_link",
                "--apply-on-start", "true",
            ],
        ),
    ])
