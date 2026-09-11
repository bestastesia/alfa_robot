"""Simulation-only fixed-wall service demo; reuses the legacy single-arm planner."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def launch_nodes(context):
    config = (MoveItConfigsBuilder("alfa_robot", package_name="alfa_robot_moveit_config")
              .planning_pipelines(pipelines=["ompl"]).to_moveit_configs())
    name = "v3_box_wall_grasp_demo"
    params = {"distance_demo": True, "collision_inset": 0.0}
    for key, kind in (("x", float), ("box_id", int), ("arm", str),
                      ("auto_run_once", bool), ("wall_center_y", float),
                      ("wall_bottom_z", float), ("contact_numerical_gap", float),
                      ("align_height", bool), ("shoulder_box_offset", float)):
        params[key] = ParameterValue(LaunchConfiguration(key), value_type=kind)
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
             parameters=[{"task_topic": f"/{name}/task_json",
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
        DeclareLaunchArgument("x", description="Metres from chassis front to wall near face; >0"),
        DeclareLaunchArgument("arm", default_value="auto", choices=["left", "right", "auto"],
                              description="auto stops at first successful arm"),
    ]
    for name, default, description in (
        ("align_height", "true", "Lower shared lift before grasp; false restores frozen fixed-height mode"),
        ("shoulder_box_offset", "0.25", "Shoulder midpoint above target box center, finite nonnegative metres"),
        ("box_id", "0", "0..24; row=id/5 bottom-up, column=id%5 along +Y"),
        ("contact_numerical_gap", "0.000001", "Simulation-only contact gap in metres (0..0.0001), not suction calibration"),
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
