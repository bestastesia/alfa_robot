from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _package_share():
    return Path(get_package_share_directory("alfa_robot_moveit_config"))


def _default_scene_xml():
    repo_root = _package_share().parents[4]
    return str(repo_root / "simulation" / "mujoco" / "scene.xml")


def generate_launch_description():
    package_share = _package_share()
    xml_path = LaunchConfiguration("xml_path")
    initial_positions = LaunchConfiguration("initial_positions")
    semantic_rate = LaunchConfiguration("semantic_rate")
    joint_state_rate = LaunchConfiguration("joint_state_rate")
    viewer_rate = LaunchConfiguration("viewer_rate")
    physics_rate = LaunchConfiguration("physics_rate")

    return LaunchDescription([
        DeclareLaunchArgument("xml_path", default_value=_default_scene_xml()),
        DeclareLaunchArgument("initial_positions", default_value=str(package_share / "config" / "mujoco_initial_positions.yaml")),
        DeclareLaunchArgument("semantic_rate", default_value="1.0"),
        DeclareLaunchArgument("joint_state_rate", default_value="50.0"),
        DeclareLaunchArgument("viewer_rate", default_value="30.0"),
        DeclareLaunchArgument("physics_rate", default_value="200.0"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package_share / "launch" / "static_virtual_joint_tfs.launch.py"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package_share / "launch" / "rsp.launch.py"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package_share / "launch" / "move_group.launch.py"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(package_share / "launch" / "moveit_rviz.launch.py"))),
        Node(
            package="alfa_robot_moveit_config",
            executable="mujoco_digital_twin.py",
            name="mujoco_digital_twin",
            output="screen",
            arguments=[
                "--xml", xml_path,
                "--initial-positions", initial_positions,
                "--semantic-rate", semantic_rate,
                "--joint-state-rate", joint_state_rate,
                "--viewer-rate", viewer_rate,
                "--physics-rate", physics_rate,
                "--robot-mode", "actuator",
                "--frame-id", "base_link",
            ],
        ),
        Node(
            package="alfa_robot_moveit_config",
            executable="semantic_scene_to_planning_scene.py",
            name="semantic_scene_to_planning_scene",
            output="screen",
            arguments=[
                "--frame-id", "base_link",
                "--pointcloud-topic", "/mujoco_scene_points",
                "--pointcloud-rate", "0.0",
                "--max-publish-rate", semantic_rate,
                "--apply-on-start", "true",
            ],
        ),
    ])
