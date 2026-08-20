from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "alfa_robot", package_name="alfa_robot_moveit_config"
    ).to_moveit_configs()

    side = LaunchConfiguration("side")
    rviz_config = LaunchConfiguration("rviz_config")
    start_rviz = LaunchConfiguration("start_rviz")
    start_rerun = LaunchConfiguration("start_rerun")
    psi_step_deg = LaunchConfiguration("psi_step_deg")
    rerun_recording_path = LaunchConfiguration("rerun_recording_path")

    return LaunchDescription(
        [
            DeclareLaunchArgument("side", default_value="left"),
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("start_rerun", default_value="true"),
            DeclareLaunchArgument("psi_step_deg", default_value="2.0"),
            DeclareLaunchArgument("rerun_recording_path", default_value=""),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(
                    moveit_config.package_path
                    / "config"
                    / "v3_redundant_ik_interactive_demo.rviz"
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[moveit_config.robot_description],
            ),
            Node(
                package="alfa_robot_moveit_config",
                executable="v3_redundant_ik_interactive_demo",
                output="screen",
                parameters=[{"side": side, "psi_step_deg": psi_step_deg}],
            ),
            Node(
                package="alfa_robot_rerun",
                executable="v3_redundant_solution_family_viewer",
                output="screen",
                parameters=[
                    {
                        "recording_path": rerun_recording_path,
                        "spawn_viewer": True,
                    }
                ],
                condition=IfCondition(start_rerun),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                condition=IfCondition(start_rviz),
            ),
        ]
    )
