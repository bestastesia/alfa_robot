from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("alfa_robot", package_name="alfa_robot_moveit_config").to_moveit_configs()
    launch_dir = moveit_config.package_path / "launch"

    description = LaunchDescription()
    description.add_action(DeclareLaunchArgument("db", default_value="false"))
    description.add_action(DeclareLaunchArgument("debug", default_value="false"))
    description.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))
    description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "static_virtual_joint_tfs.launch.py"))
        )
    )
    description.add_action(
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir / "rsp.launch.py")))
    )
    description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "move_group_demo.launch.py"))
        )
    )
    description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "moveit_rviz_demo.launch.py")),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )
    description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "warehouse_db.launch.py")),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )
    description.add_action(
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
        )
    )
    description.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(launch_dir / "spawn_controllers.launch.py"))
        )
    )
    return description
