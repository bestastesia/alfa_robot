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
        "motion_mode": "roll",
        "box_size": "0.30",
        "target_offset_x": "0.0",
        "target_offset_y": "0.0",
        "target_offset_z": "0.0",
        "initial_target_roll_deg": LaunchConfiguration("initial_target_roll_deg"),
        "angular_step_deg": LaunchConfiguration("angular_step_deg"),
        "maximum_joint_step_deg": LaunchConfiguration("maximum_joint_step_deg"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("start_rviz", default_value="true"),
            DeclareLaunchArgument("start_rerun", default_value="true"),
            DeclareLaunchArgument("auto_run_once", default_value="false"),
            DeclareLaunchArgument("rerun_recording_path", default_value=""),
            DeclareLaunchArgument("initial_box_x", default_value="0.55"),
            DeclareLaunchArgument("initial_box_y", default_value="0.0"),
            DeclareLaunchArgument("initial_box_z", default_value="0.65"),
            DeclareLaunchArgument("initial_target_roll_deg", default_value="25.0"),
            DeclareLaunchArgument("angular_step_deg", default_value="2.0"),
            DeclareLaunchArgument("maximum_joint_step_deg", default_value="12.0"),
            IncludeLaunchDescription(source, launch_arguments=arguments.items()),
        ]
    )
