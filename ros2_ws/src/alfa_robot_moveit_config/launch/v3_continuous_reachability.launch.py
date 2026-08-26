from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    arm_mount_forward_offset = LaunchConfiguration("arm_mount_forward_offset")
    moveit_config = MoveItConfigsBuilder(
        "alfa_robot", package_name="alfa_robot_moveit_config"
    ).robot_description(
        mappings={"arm_mount_forward_offset": arm_mount_forward_offset}
    ).to_moveit_configs()

    arguments = [
        DeclareLaunchArgument("side", default_value="left"),
        DeclareLaunchArgument("arm_mount_forward_offset", default_value="0.0"),
        DeclareLaunchArgument("test_pattern", default_value="linear_x"),
        DeclareLaunchArgument("x_start_min", default_value="0.15"),
        DeclareLaunchArgument("x_start_max", default_value="1.35"),
        DeclareLaunchArgument("x_start_step", default_value="0.01"),
        DeclareLaunchArgument("travel", default_value="0.40"),
        DeclareLaunchArgument("travel_x", default_value="0.0"),
        DeclareLaunchArgument("travel_y", default_value="0.0"),
        DeclareLaunchArgument("travel_z", default_value="0.0"),
        DeclareLaunchArgument("path_step", default_value="0.01"),
        DeclareLaunchArgument("disk_radius", default_value="0.15"),
        DeclareLaunchArgument("disk_ring_step", default_value="0.03"),
        DeclareLaunchArgument("lateral_min", default_value="-1.18054221"),
        DeclareLaunchArgument("lateral_max", default_value="0.51945779"),
        DeclareLaunchArgument("lateral_step", default_value="0.05"),
        DeclareLaunchArgument("height_min", default_value="0.3032993"),
        DeclareLaunchArgument("height_max", default_value="2.3032993"),
        DeclareLaunchArgument("height_step", default_value="0.05"),
        DeclareLaunchArgument("target_roll", default_value="0.0"),
        DeclareLaunchArgument("target_pitch", default_value="1.5707963267948966"),
        DeclareLaunchArgument("target_yaw", default_value="0.0"),
        DeclareLaunchArgument("swivel_step_deg", default_value="5.0"),
        DeclareLaunchArgument("maximum_joint_delta_deg", default_value="10.0"),
        DeclareLaunchArgument("edge_joint_step_deg", default_value="2.5"),
        DeclareLaunchArgument("maximum_task_position_deviation", default_value="0.001"),
        DeclareLaunchArgument(
            "maximum_task_orientation_deviation_deg", default_value="1.0"
        ),
        DeclareLaunchArgument("high_gain_swivel_neighbor_steps", default_value="2"),
        DeclareLaunchArgument("refinement_max_depth", default_value="3"),
        DeclareLaunchArgument("refinement_min_step", default_value="0.00125"),
        DeclareLaunchArgument("refinement_time_budget_ms", default_value="2.0"),
        DeclareLaunchArgument("ignore_opposite_arm", default_value="true"),
        DeclareLaunchArgument("record_failure_states", default_value="false"),
        DeclareLaunchArgument("record_failure_paths", default_value="false"),
        DeclareLaunchArgument(
            "output_json", default_value="/tmp/v3_continuous_reachability.json"
        ),
        DeclareLaunchArgument("progress_period", default_value="500"),
    ]

    scanner = Node(
        package="alfa_robot_moveit_config",
        executable="v3_continuous_reachability",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {
                "side": LaunchConfiguration("side"),
                "arm_mount_forward_offset": arm_mount_forward_offset,
                "test_pattern": LaunchConfiguration("test_pattern"),
                "x_start_min": LaunchConfiguration("x_start_min"),
                "x_start_max": LaunchConfiguration("x_start_max"),
                "x_start_step": LaunchConfiguration("x_start_step"),
                "travel": LaunchConfiguration("travel"),
                "travel_x": LaunchConfiguration("travel_x"),
                "travel_y": LaunchConfiguration("travel_y"),
                "travel_z": LaunchConfiguration("travel_z"),
                "path_step": LaunchConfiguration("path_step"),
                "disk_radius": LaunchConfiguration("disk_radius"),
                "disk_ring_step": LaunchConfiguration("disk_ring_step"),
                "lateral_min": LaunchConfiguration("lateral_min"),
                "lateral_max": LaunchConfiguration("lateral_max"),
                "lateral_step": LaunchConfiguration("lateral_step"),
                "height_min": LaunchConfiguration("height_min"),
                "height_max": LaunchConfiguration("height_max"),
                "height_step": LaunchConfiguration("height_step"),
                "target_roll": LaunchConfiguration("target_roll"),
                "target_pitch": LaunchConfiguration("target_pitch"),
                "target_yaw": LaunchConfiguration("target_yaw"),
                "swivel_step_deg": LaunchConfiguration("swivel_step_deg"),
                "maximum_joint_delta_deg": LaunchConfiguration(
                    "maximum_joint_delta_deg"
                ),
                "edge_joint_step_deg": LaunchConfiguration("edge_joint_step_deg"),
                "maximum_task_position_deviation": LaunchConfiguration(
                    "maximum_task_position_deviation"
                ),
                "maximum_task_orientation_deviation_deg": LaunchConfiguration(
                    "maximum_task_orientation_deviation_deg"
                ),
                "high_gain_swivel_neighbor_steps": LaunchConfiguration(
                    "high_gain_swivel_neighbor_steps"
                ),
                "refinement_max_depth": LaunchConfiguration("refinement_max_depth"),
                "refinement_min_step": LaunchConfiguration("refinement_min_step"),
                "refinement_time_budget_ms": LaunchConfiguration(
                    "refinement_time_budget_ms"
                ),
                "ignore_opposite_arm": LaunchConfiguration("ignore_opposite_arm"),
                "record_failure_states": LaunchConfiguration("record_failure_states"),
                "record_failure_paths": LaunchConfiguration("record_failure_paths"),
                "output_json": LaunchConfiguration("output_json"),
                "progress_period": LaunchConfiguration("progress_period"),
            },
        ],
    )
    return LaunchDescription(arguments + [scanner])
