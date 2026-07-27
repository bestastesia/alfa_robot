#!/usr/bin/python3
from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys
from types import SimpleNamespace


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "extract_sequence_rerun.py"
WORKSPACE_SRC = SCRIPT.parents[2]
sys.path.insert(0, str(WORKSPACE_SRC / "alfa_robot_rerun"))
SPEC = importlib.util.spec_from_file_location("extract_sequence_rerun", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def main() -> int:
    expected_pairs = [
        (1, 3), (4, 6), (7, 9), (10, 12), (13, 15),
    ]
    pairs = MODULE.parse_pair_sequence(MODULE.DEFAULT_SEQUENCE)
    assert pairs == expected_pairs, pairs
    assert MODULE.OUTER_GRASP_TARGET_Y_M == 0.50
    assert MODULE.TASK_LAYOUT_Y_OFFSETS == {
        "centered": 0.0,
        "right_shift_0p1": 0.05,
    }
    assert MODULE.DIRECT_UPDOWN_LIFT_PAIRS == {(7, 9)}
    assert math.isclose(
        MODULE.OUTER_GRASP_TARGET_Y_M + MODULE.TASK_LAYOUT_Y_OFFSETS["right_shift_0p1"],
        0.55,
    )
    assert math.isclose(
        -MODULE.OUTER_GRASP_TARGET_Y_M + MODULE.TASK_LAYOUT_Y_OFFSETS["right_shift_0p1"],
        -0.45,
    )

    left_modes = MODULE.parse_arm_grasp_mode_sequence("", pairs, "left")
    right_modes = MODULE.parse_arm_grasp_mode_sequence("", pairs, "right")
    raw_left_modes = [
        "front", "front", "front", "top_suction", "top_suction",
    ]
    raw_right_modes = [
        "front", "front", "front", "top_suction", "top_suction",
    ]
    assert left_modes == raw_left_modes, left_modes
    assert right_modes == raw_right_modes, right_modes

    left_modes, right_modes = MODULE.convert_mixed_grasp_modes_to_front(left_modes, right_modes)
    assert left_modes == [
        "front", "front", "front", "top_suction", "top_suction",
    ], left_modes
    assert right_modes == [
        "front", "front", "front", "top_suction", "top_suction",
    ], right_modes

    vehicle_modes = [
        MODULE.pair_vehicle_mode(left_mode, right_mode)
        for left_mode, right_mode in zip(left_modes, right_modes)
    ]
    assert vehicle_modes.count("front") == 3, vehicle_modes
    assert vehicle_modes.count("top_suction") == 2, vehicle_modes
    rollout_args = SimpleNamespace(
        extract_rollout_mode="box_pose_rrt",
        top_extract_rollout_mode="top_updown_lift",
    )
    assert MODULE.extract_rollout_mode_for_pair(rollout_args, 1, 3, "front") == "box_pose_rrt"
    assert MODULE.extract_rollout_mode_for_pair(rollout_args, 7, 9, "front") == "direct_updown_lift"
    assert (
        MODULE.extract_rollout_mode_for_pair(rollout_args, 10, 12, "top_suction")
        == "top_updown_lift"
    )

    parser_source = SCRIPT.read_text()
    assert 'parser.add_argument("--loaded-updown", type=float, default=0.1)' in parser_source
    assert 'parser.add_argument("--box-front-x", type=float, default=0.90)' in parser_source
    assert 'parser.add_argument("--top-box-front-x", type=float, default=0.70' in parser_source
    assert 'choices=["centered", "right_shift_0p1", "both"]' in parser_source
    assert 'OUTER_GRASP_TARGET_Y_M = 0.50' in parser_source
    assert 'default=True,\n        help="默认在0~0.7m范围' in parser_source
    assert 'choices=["rrt", "shortcut"], default="shortcut"' in parser_source
    assert '"--place-cycle-enabled"' in parser_source
    assert 'default=True,\n        help="负重后规划到放置姿态' in parser_source
    assert '"--extract-only"' in parser_source
    assert 'default=False,\n        help="只计算 IK 和抽离' in parser_source
    assert 'default="box_pose_rrt"' in parser_source
    assert 'default="top_updown_lift"' in parser_source
    assert '"direct_updown_lift"' in parser_source
    assert 'default=0.40' in parser_source
    assert '"--ik-only-raw"' in parser_source
    print("extract sequence definition passed: 5 equal-height pairs, 3 front + 2 direct top lifts")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
