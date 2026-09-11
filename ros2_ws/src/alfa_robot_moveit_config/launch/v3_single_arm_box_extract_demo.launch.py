from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("alfa_robot", package_name="alfa_robot_moveit_config")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    start_rviz = LaunchConfiguration("start_rviz")
    start_rerun = LaunchConfiguration("start_rerun")
    rviz_config = LaunchConfiguration("rviz_config")
    recording_path = LaunchConfiguration("rerun_recording_path")
    auto_run_once = LaunchConfiguration("auto_run_once")

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("start_rerun", default_value="true"),
            DeclareLaunchArgument("auto_run_once", default_value="false"),
            DeclareLaunchArgument(
                "demo_config",
                default_value=str(moveit_config.package_path / "config" /
                                  "v3_single_arm_box_extract_demo.yaml"),
                description="Simulation-only scene and planner ROS parameter file",
            ),
            DeclareLaunchArgument("rerun_recording_path", default_value=""),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=str(
                    moveit_config.package_path
                    / "config"
                    / "v3_single_arm_box_extract_demo.rviz"
                ),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[moveit_config.robot_description],
                remappings=[
                    (
                        "/joint_states",
                        "/v3_single_arm_box_extract_demo/joint_states",
                    )
                ],
            ),
            Node(
                package="alfa_robot_moveit_config",
                executable="v3_single_arm_box_extract_demo",
                output="screen",
                parameters=[
                    moveit_config.to_dict(),
                    LaunchConfiguration("demo_config"),
                    {"auto_run_once": auto_run_once},
                ],
            ),
            Node(
                package="alfa_robot_rerun",
                executable="v3_single_arm_box_extract_viewer",
                output="screen",
                parameters=[
                    {
                        "recording_path": recording_path,
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
