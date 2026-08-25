from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("alfa_robot", package_name="alfa_robot_moveit_config")
        .to_moveit_configs()
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("snapshot"),
            DeclareLaunchArgument("output"),
            DeclareLaunchArgument("frames", default_value="50"),
            DeclareLaunchArgument("target_radius", default_value="0.79"),
            DeclareLaunchArgument("orientation_only_step_deg", default_value="2.0"),
            DeclareLaunchArgument(
                "shortcut_start_radial_rotation_deg", default_value="30.0"
            ),
            DeclareLaunchArgument("path_mode", default_value="radial"),
            DeclareLaunchArgument("top_retreat_step_x", default_value="0.01"),
            DeclareLaunchArgument("top_shortcut_start_step", default_value="30"),
            DeclareLaunchArgument("candidate_index", default_value="0"),
            DeclareLaunchArgument("resume_radial_after_orientation", default_value="false"),
            DeclareLaunchArgument("interleave_loaded_shortcuts", default_value="false"),
            DeclareLaunchArgument("updown_compensation_enabled", default_value="true"),
            Node(
                package="alfa_robot_benchmarks",
                executable="analytic_radial_extract_prototype",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                ],
                arguments=[
                    "--snapshot",
                    LaunchConfiguration("snapshot"),
                    "--output",
                    LaunchConfiguration("output"),
                    "--frames",
                    LaunchConfiguration("frames"),
                    "--target-radius",
                    LaunchConfiguration("target_radius"),
                    "--orientation-only-step-deg",
                    LaunchConfiguration("orientation_only_step_deg"),
                    "--shortcut-start-radial-rotation-deg",
                    LaunchConfiguration("shortcut_start_radial_rotation_deg"),
                    "--path-mode",
                    LaunchConfiguration("path_mode"),
                    "--top-retreat-step-x",
                    LaunchConfiguration("top_retreat_step_x"),
                    "--top-shortcut-start-step",
                    LaunchConfiguration("top_shortcut_start_step"),
                    "--candidate-index",
                    LaunchConfiguration("candidate_index"),
                    "--resume-radial-after-orientation",
                    LaunchConfiguration("resume_radial_after_orientation"),
                    "--interleave-loaded-shortcuts",
                    LaunchConfiguration("interleave_loaded_shortcuts"),
                    "--updown-compensation-enabled",
                    LaunchConfiguration("updown_compensation_enabled"),
                ],
            ),
        ]
    )
