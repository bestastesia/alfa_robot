#!/usr/bin/env python3
from __future__ import annotations

import ast
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
EXECUTE = ROOT / "ros2_ws/src/alfa_robot_moveit_config/scripts/execute_l6_r8_mock_live.py"
JOINTS = ROOT / "ros2_ws/src/alfa_robot_execution_bridge/alfa_robot_execution_bridge/joints.py"
UPDOWN = ROOT / "ros2_ws/src/alfa_robot_execution_bridge/alfa_robot_execution_bridge/updown.py"
PLANNER = ROOT / "ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp"
LAUNCH = ROOT / "ros2_ws/src/alfa_robot_moveit_config/launch/dual_arm_planner.launch.py"
DOC = ROOT / "docs/ethercat/REAL_DIRECTION_SAFETY.md"
SEND_SEQUENCE = ROOT / "scripts/lhy_dev/send_dual_grasp_sequence.py"
MOVE_ALL_COMPAT = ROOT / "scripts/lhy_dev/run_move_all_joints_abs.sh"

EXPECTED_LEFT0 = [0.0, -45.0, 120.0, -75.0, 0.0, 0.0]
EXPECTED_RIGHT0 = [0.0, -45.0, 120.0, -75.0, 0.0, 0.0]
EXPECTED_PLACE_POSE = "[0.0,-55.0,-50.0,-60.0,0.0,0.0]"
EXPECTED_PLACE_UPDOWN = "0.10"
EXPECTED_SIGNS = {
    "left_joint1": 1.0,
    "left_joint2": 1.0,
    "left_joint3": -1.0,
    "left_joint4": 1.0,
    "left_joint5": -1.0,
    "left_joint6": 1.0,
    "right_joint1": 1.0,
    "right_joint2": -1.0,
    "right_joint3": 1.0,
    "right_joint4": -1.0,
    "right_joint5": 1.0,
    "right_joint6": 1.0,
    "turn": 1.0,
}


def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


def load_execute_tree() -> ast.Module:
    return ast.parse(EXECUTE.read_text(), filename=str(EXECUTE))


def load_joints_tree() -> ast.Module:
    return ast.parse(JOINTS.read_text(), filename=str(JOINTS))


def literal_assignment(tree: ast.Module, name: str):
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id == name:
                    return ast.literal_eval(node.value)
    fail(f"missing assignment {name}")


def default_arg(tree: ast.Module, function_name: str, arg_name: str):
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == function_name:
            args = node.args.args
            defaults = node.args.defaults
            default_by_name = {
                arg.arg: default for arg, default in zip(args[-len(defaults):], defaults)
            }
            if arg_name not in default_by_name:
                fail(f"{function_name} missing default for {arg_name}")
            return ast.literal_eval(default_by_name[arg_name])
    fail(f"missing function {function_name}")


def main() -> int:
    joints_tree = load_joints_tree()
    signs = literal_assignment(joints_tree, "ROS_TO_ETHERCAT_SIGN_BY_JOINT")
    if signs != EXPECTED_SIGNS:
        fail(f"EtherCAT direction signs changed: {signs}")
    if "EXECUTION_TO_ETHERCAT_SIGN" in EXECUTE.read_text():
        fail("execute script must not define its own EtherCAT direction sign table")
    if "ROS_TO_ETHERCAT_SIGN_BY_JOINT" in EXECUTE.read_text():
        fail("execute script must import direction helpers, not reference the sign table directly")
    if literal_assignment(joints_tree, "UPDOWN_PHYSICAL_ZERO_OFFSET_M") != 0.0:
        fail("updown physical/logical contract must retain the calibrated zero offset")
    if literal_assignment(joints_tree, "UPDOWN_LOGICAL_LOWER_M") != 0.0:
        fail("updown logical lower limit must remain 0.0m")
    if literal_assignment(joints_tree, "UPDOWN_LOGICAL_UPPER_M") != 0.7:
        fail("updown logical upper limit must remain 0.7m")

    tree = load_execute_tree()
    left_family = literal_assignment(tree, "LOADED_LEFT_POSE_FAMILY_DEG")
    right_family = literal_assignment(tree, "LOADED_RIGHT_POSE_FAMILY_DEG")
    if left_family[0] != EXPECTED_LEFT0:
        fail(f"left loaded pose index 0 changed: {left_family[0]}")
    if right_family[0] != EXPECTED_RIGHT0:
        fail(f"right loaded pose index 0 changed: {right_family[0]}")
    if default_arg(tree, "loaded_joint_map", "index") != 0:
        fail("loaded_joint_map default index must be 0")

    execute_text = EXECUTE.read_text()
    for required_import in [
        "from alfa_robot_execution_bridge.joints import",
        "from alfa_robot_execution_bridge.updown import",
        "ros_to_ethercat_position",
        "ethercat_to_ros_position",
        "REAL_CONTROLLER_JOINT_NAMES",
        "EXECUTION_JOINT_NAMES",
        "make_updown_command_data",
        "synchronized_updown_velocity_mps",
    ]:
        if required_import not in execute_text:
            fail(f"execute script missing canonical joint contract import: {required_import}")
    if "--loaded-preferred-pose-index" not in execute_text:
        fail("execute script lacks --loaded-preferred-pose-index")
    if "--no-real-apply-direction-signs" not in execute_text:
        fail("execute script lacks unsafe direction-disable guard flag")
    if "ALFA_ALLOW_UNSAFE_DIRECTION_OVERRIDE=I_UNDERSTAND_DIRECTION_RISK" not in execute_text:
        fail("execute script lacks unsafe direction override guard")
    if "default=0" not in execute_text:
        fail("execute script must default loaded pose index to 0")
    if "loaded_preferred_pose_index=args.loaded_preferred_pose_index" not in execute_text:
        fail("planner args must forward loaded_preferred_pose_index")
    if "loaded_updown=args.loaded_updown" not in execute_text:
        fail("planner loaded updown must not reuse the current IK fixed_updown")
    if "loaded = loaded_joint_map(args.loaded_preferred_pose_index)" not in execute_text:
        fail("execution loaded pose must use selected loaded_preferred_pose_index")
    if "apply_ethercat_signs=apply_ethercat_signs" not in execute_text:
        fail("real direct trajectories must apply EtherCAT direction signs")
    if "values[execution_name] = ethercat_to_ros_position(" not in execute_text:
        fail("current joint-state reads must convert EtherCAT signs through joints.py")
    if "def wait_for_enter_confirmation(" not in execute_text:
        fail("real execution must retain interactive enter confirmation gates")
    if "split_task_execution_phases(" not in execute_text:
        fail("real execution must split pre-contact, contact, and post-contact phases")
    if "split_post_contact_place_cycle(" not in execute_text:
        fail("real execution must split loaded-to-place and place-to-loaded phases")
    if 'parser.add_argument(\n        "--loaded-updown"' not in execute_text:
        fail("execute script must expose an independent loaded updown target")
    if "updown_samples=(home_updown_samples if args.send_updown else None)" not in execute_text:
        fail("home transition must synchronously command updown")
    if "msg.data = [logical_to_physical_updown" in execute_text:
        fail("execute script still publishes the rejected legacy one-element updown command")
    updown_text = UPDOWN.read_text()
    if "validate_updown_command_data" not in updown_text:
        fail("canonical updown helper must validate the four-element wire contract")
    if 'parser.add_argument("--place-cycle-enabled"' not in execute_text:
        fail("execute script must expose the place-cycle safety switch")
    if EXPECTED_PLACE_POSE not in execute_text:
        fail("execute script default place pose changed unexpectedly")

    planner_text = PLANNER.read_text()
    if 'get_or_declare_parameter<int>("loaded_preferred_pose_index", 0)' not in planner_text:
        fail("planner node default loaded_preferred_pose_index must be 0")
    if 'get_or_declare_parameter<double>("extract_monitor_place_updown", 0.10)' not in planner_text:
        fail("planner default place updown must remain 0.10m")
    if planner_text.count(EXPECTED_PLACE_POSE) < 2:
        fail("planner default left/right place poses changed unexpectedly")

    launch_text = LAUNCH.read_text()
    if 'DeclareLaunchArgument("loaded_preferred_pose_index", default_value="0")' not in launch_text:
        fail("planner launch default loaded_preferred_pose_index must be 0")
    if 'DeclareLaunchArgument("extract_monitor_place_updown", default_value="0.10")' not in launch_text:
        fail("planner launch default place updown must remain 0.10m")

    doc_text = DOC.read_text() if DOC.exists() else ""
    for required in [
        "--loaded-preferred-pose-index 0",
        "--no-real-apply-direction-signs",
        "left_joint3",
        "left_joint5",
        "right_joint2",
    ]:
        if required not in doc_text:
            fail(f"safety doc missing {required}")

    sequence_text = SEND_SEQUENCE.read_text()
    if "'--execute-backend', choices=['planner-live', 'service'], default='planner-live'" not in sequence_text:
        fail("send_dual_grasp_sequence.py must default --execute-backend to planner-live")
    if "'--yes-execute'" not in sequence_text:
        fail("send_dual_grasp_sequence.py lacks --yes-execute confirmation guard")
    if "Refusing real execution: add --yes-execute" not in sequence_text:
        fail("send_dual_grasp_sequence.py must refuse execution without --yes-execute")
    for required in [
        "request.request_id =",
        "self.assign_pose_6d(request.left, task.left_pose_6d)",
        "self.assign_pose_6d(request.right, task.right_pose_6d)",
        "request.left.grasp_mode = task.left_mode",
        "request.right.grasp_mode = task.right_mode",
    ]:
        if required not in sequence_text:
            fail(f"send_dual_grasp_sequence.py missing strict 6D task contract field: {required}")
    for forbidden in [
        "request.context.",
        "request.left_position",
        "request.right_position",
        "request.left_grasp_mode",
        "request.right_grasp_mode",
        "request.task_id",
    ]:
        if forbidden in sequence_text:
            fail(f"send_dual_grasp_sequence.py still writes removed task field: {forbidden}")
    if "'--max-joint-speed-deg-s', type=float, default=10.0" not in sequence_text:
        fail("send_dual_grasp_sequence.py --max-joint-speed-deg-s default changed unexpectedly")
    if "'--max-updown-speed-m-s', type=float, default=0.05" not in sequence_text:
        fail("send_dual_grasp_sequence.py --max-updown-speed-m-s default changed unexpectedly")
    if "'--hz', type=float, default=10.0" not in sequence_text:
        fail("send_dual_grasp_sequence.py --hz default changed unexpectedly")
    sequence_tree = ast.parse(sequence_text, filename=str(SEND_SEQUENCE))
    for node in ast.walk(sequence_tree):
        if isinstance(node, ast.FunctionDef) and node.name == "run_planner_live_task":
            source = ast.get_source_segment(sequence_text, node) or ""
            for forwarded in [
                "args.hz",
                "args.max_joint_speed_deg_s",
                "args.max_updown_speed_m_s",
                "args.updown_acceleration_m_s2",
                "args.updown_deceleration_m_s2",
            ]:
                if forwarded not in source:
                    fail(f"run_planner_live_task must forward {forwarded} to the real-direct planner script")
            break
    else:
        fail("send_dual_grasp_sequence.py missing run_planner_live_task")

    move_all_text = MOVE_ALL_COMPAT.read_text()
    if "alfa_robot_execution_bridge/scripts/jog_to_pose.py" not in move_all_text:
        fail("run_move_all_joints_abs.sh must delegate to canonical jog_to_pose.py")
    if "scripts/move_all_joints_abs.py" in move_all_text and "不允许" not in move_all_text:
        fail("run_move_all_joints_abs.sh must not execute the deprecated duplicate implementation")

    print("OK: L6/R8 real direction safety defaults are locked in repo.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
