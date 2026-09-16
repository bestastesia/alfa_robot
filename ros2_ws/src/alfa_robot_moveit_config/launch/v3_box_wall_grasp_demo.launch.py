"""Simulation-only fixed-wall service demo; reuses the legacy single-arm planner."""
import json
import math
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def launch_nodes(context):
    if not math.isfinite(float(LaunchConfiguration("model_ground_offset").perform(context))):
        raise ValueError("model_ground_offset must be finite metres")
    config = (MoveItConfigsBuilder("alfa_robot", package_name="alfa_robot_moveit_config")
              .robot_description(mappings={"model_ground_offset": LaunchConfiguration("model_ground_offset").perform(context)})
              .planning_pipelines(pipelines=["ompl"]).to_moveit_configs())
    # Match the thin-gap wall task's strict post-validation; the default OMPL
    # segment spacing can step over a collision and repeatedly return unusable paths.
    for arm in ("left_arm", "right_arm"):
        config.planning_pipelines["ompl"][arm]["longest_valid_segment_fraction"] = 0.0005
    name = "v3_box_wall_grasp_demo"
    params = {"distance_demo": True, "collision_inset": 0.0}
    for key, kind in (("x", float), ("box_id", int), ("arm", str), ("suction_mode", str), ("wall_context", str), ("initial_pose", str), ("post_extract_policy", str),
                      ("auto_run_once", bool), ("sequence_mode", bool), ("rear_clearance", float), ("wall_center_y", float),
                      ("wall_bottom_z", float), ("contact_numerical_gap", float),
                      ("align_height", bool), ("shoulder_box_offset", float), ("top_shoulder_above_wrist", float),
                      ("check_environment", bool), ("height_strategy", str), ("comfort_branch", str),
                      ("comfort_ratio_min", float), ("comfort_ratio_preferred", float),
                      ("comfort_ratio_max", float), ("planning_seed", int), ("trajectory_variant", str),
                      ("top_k_complete", int), ("trajectory_sample_period", float),
                      ("empty_velocity_scaling", float), ("empty_acceleration_scaling", float),
                      ("loaded_velocity_scaling", float), ("loaded_acceleration_scaling", float),
                      ("arm_max_jerk", float), ("head_max_jerk", float), ("updown_max_jerk", float)):
        params[key] = ParameterValue(LaunchConfiguration(key), value_type=kind)
    # Read once at startup; the C++ boundary validates exact geometry for every consumer.
    if LaunchConfiguration("check_environment").perform(context).lower() == "true":
        path = LaunchConfiguration("environment_file").perform(context)
        path = Path(path).expanduser() if path else config.package_path / "config" / "v3_box_wall_environment.json"
        params["environment_json"] = ParameterValue(json.dumps(json.loads(path.read_text())), value_type=str)
    front = LaunchConfiguration("chassis_front_x").perform(context)
    if front:
        params["chassis_front_x"] = float(front)
    return [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[config.robot_description], output="screen",
             remappings=[("/joint_states", f"/{name}/joint_states")]),
        Node(package="alfa_robot_moveit_config", executable="v3_single_arm_box_extract_demo",
             name=name, parameters=[config.to_dict(), params], output="screen",
             on_exit=[Shutdown(reason="Grasp service node exited; see its preceding error log")]),
        Node(package="alfa_robot_rerun", executable="v3_single_arm_box_extract_viewer",
             parameters=[config.robot_description, {"task_topic": f"/{name}/task_json",
                          "spawn_viewer": ParameterValue(LaunchConfiguration("spawn_viewer"), value_type=bool),
                          "recording_path": LaunchConfiguration("rerun_recording_path")}],
             condition=IfCondition(LaunchConfiguration("start_rerun")), output="screen"),
        Node(package="rviz2", executable="rviz2", output="screen",
             arguments=["-d", str(config.package_path / "config" / "v3_single_arm_box_extract_demo.rviz")],
             parameters=[config.robot_description, config.robot_description_semantic],
             remappings=[(f"/v3_single_arm_box_extract_demo/{topic}", f"/{name}/{topic}")
                         for topic in ("scene_markers", "status_markers")],
             condition=IfCondition(LaunchConfiguration("start_rviz"))),
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument("initial_pose", default_value="home", choices=["home", "arms_down"],
                              description="Explicit simulation start: home (unchanged default), or arms_down with both J4=0; no simulated transition from home"),
        DeclareLaunchArgument("wall_context", default_value="full", choices=["full", "sequence_prefix", "target_only"],
                              description="full: all 25 boxes; sequence_prefix: scene before this box; target_only: explicit local research compatibility without neighbor boxes"),
        DeclareLaunchArgument("suction_mode", default_value="auto", choices=["auto", "top"],
                              description="auto: front then top (bottom top only); top: diagnostic top-only"),
        DeclareLaunchArgument("x", description="Metres from chassis front to wall near face; >0"),
        DeclareLaunchArgument("arm", default_value="auto", choices=["left", "right", "auto"],
                              description="auto searches arms in the existing fallback order"),
        DeclareLaunchArgument("trajectory_variant", default_value="topk",
                              choices=["topk", "shortcut_ruckig", "chomp_ruckig"],
                              description="Cumulative trajectory postprocessing variant"),
    ]
    for name, default, description in (
        ("post_extract_policy", "rear_release", "rear_release places/releases behind chassis; loaded_home preserves the local attached return"),
        ("height_strategy", "fixed_offset", "fixed_offset or comfort_radius; one height per arm"),
        ("comfort_ratio_min", "0.8", "Minimum normalized shoulder-to-TCP comfort distance"),
        ("comfort_ratio_preferred", "0.8", "Preferred normalized distance; experiment hypothesis"),
        ("comfort_ratio_max", "0.8", "Maximum normalized comfort distance"),
        ("comfort_branch", "auto", "auto in normal use; above/below for offline branch diagnostics only"),
        ("planning_seed", "0", "0 keeps normal RNG; positive seed set before OMPL initialization"),
        ("top_k_complete", "3", "Complete geometric candidates retained per transfer round; 1..8"),
        ("trajectory_sample_period", "0.05", "Simulation replay sampling period in seconds"),
        ("empty_velocity_scaling", "0.50", "Empty-motion velocity scaling in (0,1]"),
        ("empty_acceleration_scaling", "0.50", "Empty-motion acceleration scaling in (0,1]"),
        ("loaded_velocity_scaling", "0.25", "Loaded-motion velocity scaling in (0,1]"),
        ("loaded_acceleration_scaling", "0.25", "Loaded-motion acceleration scaling in (0,1]"),
        ("arm_max_jerk", "2.0", "Simulation-only arm jerk limit rad/s^3"),
        ("head_max_jerk", "2.0", "Simulation-only head jerk limit rad/s^3"),
        ("updown_max_jerk", "0.30", "Simulation-only lift jerk limit m/s^3"),
        ("model_ground_offset", "0.000005", "V3 base_footprint grounding gap (m); 5um clears imported mesh tolerance"),
        ("check_environment", "true", "Ground/surroundings collision checks; false ONLY for historical regression"),
        ("environment_file", "", "World-axis aligned boxes JSON; empty uses 4 x 2.38 x 2.35m single-opening warehouse"),
        ("align_height", "true", "Lower shared lift before grasp; false keeps fixed height without disabling environment checks"),
        ("top_shoulder_above_wrist", "0.10", "Independent top-suction shoulder height above wrist (m), offline tested; not front comfort ratio"),
        ("shoulder_box_offset", "0.25", "Shoulder midpoint above target box center, finite nonnegative metres"),
        ("box_id", "0", "0..24; row=id/5 bottom-up, column=id%5 along +Y"),
        ("contact_numerical_gap", "0.000001", "Simulation-only contact gap in metres (0..0.0001), not suction calibration"),
        ("rear_clearance", "0.02", "Rear TCP clearance beyond measured chassis rear, metres; >=0.01 and collision checked"),
        ("sequence_mode", "false", "Auto-run a complete top-down wall sequence instead of one box"),
        ("auto_run_once", "true", "Plan launch request once, then wait for services"),
        ("chassis_front_x", "", "Optional calibrated world X; empty uses model_base collision maximum X"),
        ("wall_center_y", "0.0", "Wall middle column center in world Y, metres"),
        ("wall_bottom_z", "0.0", "Wall bottom face in world Z, metres"),
        ("start_rviz", "true", "Start RViz observer"),
        ("start_rerun", "true", "Start Rerun observer"),
        ("spawn_viewer", "true", "Open Rerun window; false for headless recording"),
        ("rerun_recording_path", "", "Optional .rrd recording path"),
    ):
        arguments.append(DeclareLaunchArgument(name, default_value=default, description=description))
    return LaunchDescription(arguments + [OpaqueFunction(function=launch_nodes)])
