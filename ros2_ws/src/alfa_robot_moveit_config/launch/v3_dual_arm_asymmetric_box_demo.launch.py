from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        "alfa_robot", package_name="alfa_robot_moveit_config"
    ).to_moveit_configs()
    source = PythonLaunchDescriptionSource(
        str(
            moveit_config.package_path
            / "launch"
            / "v3_dual_arm_cartesian_box_demo.launch.py"
        )
    )

    arguments = {
        "start_rviz": LaunchConfiguration("start_rviz"),
        "start_rerun": LaunchConfiguration("start_rerun"),
        "auto_run_once": LaunchConfiguration("auto_run_once"),
        "rerun_recording_path": LaunchConfiguration("rerun_recording_path"),
        "initial_box_x": LaunchConfiguration("initial_box_x"),
        "initial_box_y": LaunchConfiguration("initial_box_y"),
        "initial_box_z": LaunchConfiguration("initial_box_z"),
        "motion_mode": "translate",
        "grasp_pattern": LaunchConfiguration("grasp_pattern"),
        "box_size": "0.30",
        "upward_contact_lateral_offset": LaunchConfiguration(
            "upward_contact_lateral_offset"
        ),
        "target_offset_x": LaunchConfiguration("target_offset_x"),
        "target_offset_y": LaunchConfiguration("target_offset_y"),
        "target_offset_z": LaunchConfiguration("target_offset_z"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("start_rerun", default_value="true"),
            DeclareLaunchArgument("auto_run_once", default_value="false"),
            DeclareLaunchArgument("rerun_recording_path", default_value=""),
            DeclareLaunchArgument("initial_box_x", default_value="0.75"),
            DeclareLaunchArgument("initial_box_y", default_value="0.0"),
            DeclareLaunchArgument("initial_box_z", default_value="1.44"),
            DeclareLaunchArgument(
                "grasp_pattern", default_value="right_side_left_bottom"
            ),
            DeclareLaunchArgument("target_offset_x", default_value="-0.10"),
            DeclareLaunchArgument("target_offset_y", default_value="0.0"),
            DeclareLaunchArgument("target_offset_z", default_value="0.0"),
            DeclareLaunchArgument(
                "upward_contact_lateral_offset", default_value="0.15"
            ),
            IncludeLaunchDescription(source, launch_arguments=arguments.items()),
        ]
    )
