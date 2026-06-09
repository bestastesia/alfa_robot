from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils.launches import generate_move_group_launch
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("alfa_robot", package_name="alfa_robot_moveit_config")
        .to_moveit_configs()
    )

    declared_arguments = [
        DeclareLaunchArgument("execute", default_value="true"),
        DeclareLaunchArgument("start_move_group", default_value="true"),
        DeclareLaunchArgument("box_front_x", default_value="0.625"),
        DeclareLaunchArgument("fixed_updown", default_value="0.45"),
        DeclareLaunchArgument("world_to_base_z", default_value="0.202094"),
        DeclareLaunchArgument("max_rounds", default_value="10"),
        DeclareLaunchArgument("include_top_suction", default_value="true"),
        DeclareLaunchArgument("reject_ik_collisions", default_value="false"),
        DeclareLaunchArgument("check_goal_collision", default_value="false"),
        DeclareLaunchArgument("prefer_commanded_state", default_value="true"),
        DeclareLaunchArgument("allowed_start_tolerance", default_value="0.05"),
        DeclareLaunchArgument("ik_timeout", default_value="2.0"),
        DeclareLaunchArgument("planning_time", default_value="8.0"),
        DeclareLaunchArgument("planning_attempts", default_value="20"),
        DeclareLaunchArgument("velocity_scale", default_value="1.0"),
        DeclareLaunchArgument("acceleration_scale", default_value="1.0"),
        DeclareLaunchArgument("record_jsonl_path", default_value="/mnt/mydisk/ALFA/alfa_robot/data/ik_benchmark/moveit_box_stack_flow/moveit_box_stack_flow.jsonl"),
        DeclareLaunchArgument("record_trajectories", default_value="true"),
        DeclareLaunchArgument("ik_workers", default_value="16"),
        DeclareLaunchArgument("ik_h_candidate_count", default_value="16"),
        DeclareLaunchArgument("ik_seed_count", default_value="32"),
        DeclareLaunchArgument("ik_candidate_timeout", default_value="0.01"),
        DeclareLaunchArgument("ik_h_search_margin", default_value="0.2"),
        DeclareLaunchArgument("ik_h_step", default_value="0.1"),
        DeclareLaunchArgument("front_z_reach_lower", default_value="0.45"),
        DeclareLaunchArgument("front_z_reach_upper", default_value="1.25"),
        DeclareLaunchArgument("top_z_reach_lower", default_value="0.3"),
        DeclareLaunchArgument("top_z_reach_upper", default_value="0.45"),
        DeclareLaunchArgument("ik_top_orientation_tolerance_deg", default_value="7.0"),
        DeclareLaunchArgument("optimized_ik_check_collision", default_value="true"),
        DeclareLaunchArgument("ik_fallback_enabled", default_value="false"),
        DeclareLaunchArgument("enable_container_obstacle", default_value="true"),
        DeclareLaunchArgument("container_frame", default_value="world"),
        DeclareLaunchArgument("container_length", default_value="4.0"),
        DeclareLaunchArgument("container_width", default_value="2.2"),
        DeclareLaunchArgument("container_height", default_value="2.4"),
        DeclareLaunchArgument("container_center_x", default_value="0.8"),
        DeclareLaunchArgument("container_center_y", default_value="0.0"),
        DeclareLaunchArgument("container_floor_z", default_value="0.0"),
        DeclareLaunchArgument("container_wall_thickness", default_value="0.02"),
        DeclareLaunchArgument("enable_attached_box_collision", default_value="true"),
        DeclareLaunchArgument("carried_box_depth", default_value="0.3"),
        DeclareLaunchArgument("carried_box_width", default_value="0.4"),
        DeclareLaunchArgument("carried_box_height", default_value="0.4"),
        DeclareLaunchArgument("enable_static_box_obstacles", default_value="true"),
        DeclareLaunchArgument("static_box_obstacle_inset", default_value="0.002"),
    ]

    # MoveIt defaults to 0.01 rad start-state tolerance. The ros2_control mock
    # controller can finish a trajectory with tiny spline residuals slightly
    # above that, causing the next execute() to abort even though the robot is
    # effectively at the previous target.
    moveit_config.trajectory_execution["trajectory_execution.allowed_start_tolerance"] = ParameterValue(
        LaunchConfiguration("allowed_start_tolerance"), value_type=float
    )

    dual_arm_planner = Node(
        package="alfa_robot_moveit_config",
        executable="dual_arm_planner_node",
        name="dual_arm_planner",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {
                "execute": ParameterValue(LaunchConfiguration("execute"), value_type=bool),
                "box_front_x": ParameterValue(LaunchConfiguration("box_front_x"), value_type=float),
                "fixed_updown": ParameterValue(LaunchConfiguration("fixed_updown"), value_type=float),
                "world_to_base_z": ParameterValue(LaunchConfiguration("world_to_base_z"), value_type=float),
                "max_rounds": ParameterValue(LaunchConfiguration("max_rounds"), value_type=int),
                "include_top_suction": ParameterValue(LaunchConfiguration("include_top_suction"), value_type=bool),
                "reject_ik_collisions": ParameterValue(LaunchConfiguration("reject_ik_collisions"), value_type=bool),
                "check_goal_collision": ParameterValue(LaunchConfiguration("check_goal_collision"), value_type=bool),
                "prefer_commanded_state": ParameterValue(LaunchConfiguration("prefer_commanded_state"), value_type=bool),
                "ik_timeout": ParameterValue(LaunchConfiguration("ik_timeout"), value_type=float),
                "planning_time": ParameterValue(LaunchConfiguration("planning_time"), value_type=float),
                "planning_attempts": ParameterValue(LaunchConfiguration("planning_attempts"), value_type=int),
                "velocity_scale": ParameterValue(LaunchConfiguration("velocity_scale"), value_type=float),
                "acceleration_scale": ParameterValue(LaunchConfiguration("acceleration_scale"), value_type=float),
                "record_jsonl_path": LaunchConfiguration("record_jsonl_path"),
                "record_trajectories": ParameterValue(LaunchConfiguration("record_trajectories"), value_type=bool),
                "ik_workers": ParameterValue(LaunchConfiguration("ik_workers"), value_type=int),
                "ik_h_candidate_count": ParameterValue(LaunchConfiguration("ik_h_candidate_count"), value_type=int),
                "ik_seed_count": ParameterValue(LaunchConfiguration("ik_seed_count"), value_type=int),
                "ik_candidate_timeout": ParameterValue(LaunchConfiguration("ik_candidate_timeout"), value_type=float),
                "ik_h_search_margin": ParameterValue(LaunchConfiguration("ik_h_search_margin"), value_type=float),
                "ik_h_step": ParameterValue(LaunchConfiguration("ik_h_step"), value_type=float),
                "front_z_reach_lower": ParameterValue(LaunchConfiguration("front_z_reach_lower"), value_type=float),
                "front_z_reach_upper": ParameterValue(LaunchConfiguration("front_z_reach_upper"), value_type=float),
                "top_z_reach_lower": ParameterValue(LaunchConfiguration("top_z_reach_lower"), value_type=float),
                "top_z_reach_upper": ParameterValue(LaunchConfiguration("top_z_reach_upper"), value_type=float),
                "ik_top_orientation_tolerance_deg": ParameterValue(LaunchConfiguration("ik_top_orientation_tolerance_deg"), value_type=float),
                "optimized_ik_check_collision": ParameterValue(LaunchConfiguration("optimized_ik_check_collision"), value_type=bool),
                "ik_fallback_enabled": ParameterValue(LaunchConfiguration("ik_fallback_enabled"), value_type=bool),
                "enable_container_obstacle": ParameterValue(LaunchConfiguration("enable_container_obstacle"), value_type=bool),
                "container_frame": LaunchConfiguration("container_frame"),
                "container_length": ParameterValue(LaunchConfiguration("container_length"), value_type=float),
                "container_width": ParameterValue(LaunchConfiguration("container_width"), value_type=float),
                "container_height": ParameterValue(LaunchConfiguration("container_height"), value_type=float),
                "container_center_x": ParameterValue(LaunchConfiguration("container_center_x"), value_type=float),
                "container_center_y": ParameterValue(LaunchConfiguration("container_center_y"), value_type=float),
                "container_floor_z": ParameterValue(LaunchConfiguration("container_floor_z"), value_type=float),
                "container_wall_thickness": ParameterValue(LaunchConfiguration("container_wall_thickness"), value_type=float),
                "enable_attached_box_collision": ParameterValue(LaunchConfiguration("enable_attached_box_collision"), value_type=bool),
                "carried_box_depth": ParameterValue(LaunchConfiguration("carried_box_depth"), value_type=float),
                "carried_box_width": ParameterValue(LaunchConfiguration("carried_box_width"), value_type=float),
                "carried_box_height": ParameterValue(LaunchConfiguration("carried_box_height"), value_type=float),
                "enable_static_box_obstacles": ParameterValue(LaunchConfiguration("enable_static_box_obstacles"), value_type=bool),
                "static_box_obstacle_inset": ParameterValue(LaunchConfiguration("static_box_obstacle_inset"), value_type=float),
            },
        ],
    )

    move_group_launch = generate_move_group_launch(moveit_config)
    launch_dir = moveit_config.package_path / "launch"
    support_nodes = [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir / "static_virtual_joint_tfs.launch.py"))),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir / "rsp.launch.py"))),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=[
                moveit_config.robot_description,
                str(moveit_config.package_path / "config/ros2_controllers.yaml"),
            ],
            output="screen",
        ),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(launch_dir / "spawn_controllers.launch.py"))),
    ]
    move_group_launch.entities.extend(support_nodes + [dual_arm_planner])

    return LaunchDescription(declared_arguments + move_group_launch.entities)
