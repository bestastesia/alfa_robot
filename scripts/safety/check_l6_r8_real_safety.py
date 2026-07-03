#!/usr/bin/env python3
from __future__ import annotations

import ast
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
EXECUTE = ROOT / "ros2_ws/src/alfa_robot_moveit_config/scripts/execute_l6_r8_mock_live.py"
PLANNER = ROOT / "ros2_ws/src/alfa_robot_moveit_config/src/dual_arm_planner_node.cpp"
LAUNCH = ROOT / "ros2_ws/src/alfa_robot_moveit_config/launch/dual_arm_planner.launch.py"
DOC = ROOT / "docs/ethercat/REAL_DIRECTION_SAFETY.md"

EXPECTED_LEFT0 = [0.0, 59.04, -135.16, 0.0, -76.13, 0.0]
EXPECTED_RIGHT0 = [0.0, 58.88, -134.84, 0.0, -75.96, 0.0]
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
    "right_joint4": 1.0,
    "right_joint5": 1.0,
    "right_joint6": 1.0,
    "turn": 1.0,
}


def fail(message: str) -> None:
    print(f"FAIL: {message}", file=sys.stderr)
    raise SystemExit(1)


def load_execute_tree() -> ast.Module:
    return ast.parse(EXECUTE.read_text(), filename=str(EXECUTE))


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
    tree = load_execute_tree()
    signs = literal_assignment(tree, "EXECUTION_TO_ETHERCAT_SIGN")
    if signs != EXPECTED_SIGNS:
        fail(f"EtherCAT direction signs changed: {signs}")

    left_family = literal_assignment(tree, "LOADED_LEFT_POSE_FAMILY_DEG")
    right_family = literal_assignment(tree, "LOADED_RIGHT_POSE_FAMILY_DEG")
    if left_family[0] != EXPECTED_LEFT0:
        fail(f"left loaded pose index 0 changed: {left_family[0]}")
    if right_family[0] != EXPECTED_RIGHT0:
        fail(f"right loaded pose index 0 changed: {right_family[0]}")
    if default_arg(tree, "loaded_joint_map", "index") != 0:
        fail("loaded_joint_map default index must be 0")

    execute_text = EXECUTE.read_text()
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
    if "loaded = loaded_joint_map(args.loaded_preferred_pose_index)" not in execute_text:
        fail("execution loaded pose must use selected loaded_preferred_pose_index")
    if "apply_ethercat_signs=apply_ethercat_signs" not in execute_text:
        fail("real direct trajectories must apply EtherCAT direction signs")

    planner_text = PLANNER.read_text()
    if 'get_or_declare_parameter<int>("loaded_preferred_pose_index", 0)' not in planner_text:
        fail("planner node default loaded_preferred_pose_index must be 0")

    launch_text = LAUNCH.read_text()
    if 'DeclareLaunchArgument("loaded_preferred_pose_index", default_value="0")' not in launch_text:
        fail("planner launch default loaded_preferred_pose_index must be 0")

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

    print("OK: L6/R8 real direction safety defaults are locked in repo.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
