from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("service_name", default_value="/robot_motion/plan_joint_segment"),
        DeclareLaunchArgument("collision_service_name", default_value="/robot_motion/check_curobo_collision"),
        DeclareLaunchArgument("robot_config_path", default_value=""),
        DeclareLaunchArgument("scene_config_path", default_value=""),
        DeclareLaunchArgument("expected_scene_id", default_value=""),
        DeclareLaunchArgument("curobo_python_root", default_value=""),
        DeclareLaunchArgument("dependency_venv", default_value=""),
        DeclareLaunchArgument("warmup_iterations", default_value="5"),
        DeclareLaunchArgument("interpolation_dt", default_value="0.02"),
        DeclareLaunchArgument("max_attempts", default_value="3"),
        DeclareLaunchArgument("request_timeout_s", default_value="0.5"),
        DeclareLaunchArgument("left_payload_link", default_value=""),
        DeclareLaunchArgument("right_payload_link", default_value=""),
        DeclareLaunchArgument("payload_grid_resolution_m", default_value="0.1"),
        DeclareLaunchArgument("world_payload_object_template", default_value="cargo_box_{box_id:02d}"),
        DeclareLaunchArgument("require_world_payload_object", default_value="false"),
        DeclareLaunchArgument("use_cuda_graph", default_value="true"),
        DeclareLaunchArgument("self_collision_check", default_value="true"),
        DeclareLaunchArgument("num_ik_seeds", default_value="32"),
        DeclareLaunchArgument("num_trajopt_seeds", default_value="4"),
        DeclareLaunchArgument("trajopt_num_iters", default_value="100"),
        DeclareLaunchArgument("trajopt_inner_iters", default_value="25"),
        DeclareLaunchArgument("trajopt_history", default_value="27"),
        DeclareLaunchArgument("trajopt_n_knots", default_value="16"),
        DeclareLaunchArgument("trajopt_interpolation_steps", default_value="4"),
        DeclareLaunchArgument("trajopt_finetune_attempts", default_value="3"),
        DeclareLaunchArgument("trajopt_finetune_dt_scale", default_value="0.75"),
        DeclareLaunchArgument("optimizer_collision_activation_distance", default_value="0.01"),
        DeclareLaunchArgument("collision_cache_cuboids", default_value="128"),
        DeclareLaunchArgument("collision_cache_meshes", default_value="8"),
    ]
    planner = Node(
        package="robot_motion_curobo",
        executable="joint_segment_planner_node",
        output="screen",
        parameters=[
            {
                "service_name": LaunchConfiguration("service_name"),
                "collision_service_name": LaunchConfiguration("collision_service_name"),
                "robot_config_path": LaunchConfiguration("robot_config_path"),
                "scene_config_path": LaunchConfiguration("scene_config_path"),
                "expected_scene_id": LaunchConfiguration("expected_scene_id"),
                "curobo_python_root": LaunchConfiguration("curobo_python_root"),
                "dependency_venv": LaunchConfiguration("dependency_venv"),
                "warmup_iterations": ParameterValue(LaunchConfiguration("warmup_iterations"), value_type=int),
                "interpolation_dt": ParameterValue(LaunchConfiguration("interpolation_dt"), value_type=float),
                "max_attempts": ParameterValue(LaunchConfiguration("max_attempts"), value_type=int),
                "request_timeout_s": ParameterValue(LaunchConfiguration("request_timeout_s"), value_type=float),
                "left_payload_link": LaunchConfiguration("left_payload_link"),
                "right_payload_link": LaunchConfiguration("right_payload_link"),
                "payload_grid_resolution_m": ParameterValue(LaunchConfiguration("payload_grid_resolution_m"), value_type=float),
                "world_payload_object_template": LaunchConfiguration("world_payload_object_template"),
                "require_world_payload_object": ParameterValue(LaunchConfiguration("require_world_payload_object"), value_type=bool),
                "use_cuda_graph": ParameterValue(LaunchConfiguration("use_cuda_graph"), value_type=bool),
                "self_collision_check": ParameterValue(LaunchConfiguration("self_collision_check"), value_type=bool),
                "num_ik_seeds": ParameterValue(LaunchConfiguration("num_ik_seeds"), value_type=int),
                "num_trajopt_seeds": ParameterValue(LaunchConfiguration("num_trajopt_seeds"), value_type=int),
                "trajopt_num_iters": ParameterValue(LaunchConfiguration("trajopt_num_iters"), value_type=int),
                "trajopt_inner_iters": ParameterValue(LaunchConfiguration("trajopt_inner_iters"), value_type=int),
                "trajopt_history": ParameterValue(LaunchConfiguration("trajopt_history"), value_type=int),
                "trajopt_n_knots": ParameterValue(LaunchConfiguration("trajopt_n_knots"), value_type=int),
                "trajopt_interpolation_steps": ParameterValue(LaunchConfiguration("trajopt_interpolation_steps"), value_type=int),
                "trajopt_finetune_attempts": ParameterValue(LaunchConfiguration("trajopt_finetune_attempts"), value_type=int),
                "trajopt_finetune_dt_scale": ParameterValue(LaunchConfiguration("trajopt_finetune_dt_scale"), value_type=float),
                "optimizer_collision_activation_distance": ParameterValue(LaunchConfiguration("optimizer_collision_activation_distance"), value_type=float),
                "collision_cache_cuboids": ParameterValue(LaunchConfiguration("collision_cache_cuboids"), value_type=int),
                "collision_cache_meshes": ParameterValue(LaunchConfiguration("collision_cache_meshes"), value_type=int),
            }
        ],
    )
    return LaunchDescription(arguments + [planner])
