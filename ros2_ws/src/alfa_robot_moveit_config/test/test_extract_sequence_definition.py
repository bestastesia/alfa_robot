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
        (1, 3), (1, 6), (4, 3), (4, 6), (4, 9), (7, 6), (7, 9),
        (7, 12), (10, 9), (10, 12), (10, 15), (13, 12), (13, 15),
    ]
    pairs = MODULE.parse_pair_sequence(MODULE.DEFAULT_SEQUENCE)
    assert pairs == expected_pairs, pairs
    assert MODULE.DEFAULT_LOADED_POSE_FAMILY_DEG == "[0.0,-45.0,120.0,-75.0,0.0,0.0]"
    assert MODULE.DEFAULT_PRE_PLACE_POSE_DEG == "[0.0,-90.0,120.0,-75.0,0.0,0.0]"
    assert MODULE.OUTER_GRASP_TARGET_Y_M == 0.40
    assert MODULE.TASK_LAYOUT_Y_OFFSETS == {
        "centered": 0.0,
        "right_shift_0p1": 0.05,
    }
    assert math.isclose(
        MODULE.OUTER_GRASP_TARGET_Y_M + MODULE.TASK_LAYOUT_Y_OFFSETS["right_shift_0p1"],
        0.45,
    )
    assert math.isclose(
        -MODULE.OUTER_GRASP_TARGET_Y_M + MODULE.TASK_LAYOUT_Y_OFFSETS["right_shift_0p1"],
        -0.35,
    )

    left_modes = MODULE.parse_arm_grasp_mode_sequence("", pairs, "left")
    right_modes = MODULE.parse_arm_grasp_mode_sequence("", pairs, "right")
    left_modes, right_modes = MODULE.promote_mixed_grasp_modes_to_top(
        left_modes, right_modes
    )
    raw_left_modes = [
        "front", "front", "front", "front", "front", "front", "front",
        "top_suction", "top_suction", "top_suction", "top_suction", "top_suction",
        "top_suction",
    ]
    raw_right_modes = [
        "front", "front", "front", "front", "front", "front", "front",
        "top_suction", "top_suction", "top_suction", "top_suction", "top_suction",
        "top_suction",
    ]
    assert left_modes == raw_left_modes, left_modes
    assert right_modes == raw_right_modes, right_modes
    mixed_modes = {
        pair: (left_mode, right_mode)
        for pair, left_mode, right_mode in zip(pairs, left_modes, right_modes)
        if left_mode != right_mode
    }
    assert mixed_modes == {}, mixed_modes

    vehicle_modes = [
        MODULE.pair_vehicle_mode(left_mode, right_mode)
        for left_mode, right_mode in zip(left_modes, right_modes)
    ]
    assert vehicle_modes.count("front") == 7, vehicle_modes
    assert vehicle_modes.count("top_suction") == 6, vehicle_modes
    rollout_args = SimpleNamespace(
        extract_rollout_mode="auto",
        top_extract_rollout_mode="top_updown_lift",
    )
    for (left_id, right_id), mode in zip(pairs, vehicle_modes):
        assert MODULE.extract_rollout_mode_for_pair(
            rollout_args, left_id, right_id, mode
        ) == "auto"

    parser_source = SCRIPT.read_text()
    assert '"--loaded-candidate-limit",\n        type=int,\n        default=0' in parser_source
    assert 'parser.add_argument("--loaded-updown", type=float, default=0.1)' in parser_source
    assert 'parser.add_argument("--box-front-x", type=float, default=0.90)' in parser_source
    assert 'parser.add_argument("--top-box-front-x", type=float, default=0.70' in parser_source
    assert 'choices=["centered", "right_shift_0p1", "both"]' in parser_source
    assert 'OUTER_GRASP_TARGET_Y_M = 0.40' in parser_source
    assert 'default=True,\n        help="默认在0~0.7m范围' in parser_source
    assert 'choices=["rrt", "shortcut"], default="shortcut"' in parser_source
    assert '"--place-cycle-enabled"' in parser_source
    assert '"--pre-place-left-pose-deg"' in parser_source
    assert '"--pre-place-right-pose-deg"' in parser_source
    assert 'default=True,\n        help="负重后规划到放置姿态' in parser_source
    assert '"--extract-only"' in parser_source
    assert 'default=False,\n        help="只计算 IK 和抽离' in parser_source
    assert 'default="auto"' in parser_source
    assert 'default="box_pose_rrt"' in parser_source
    assert '"--extract-box-pose-rrt-top-goal-min-pitch-deg", type=float, default=0.0' in parser_source
    assert '"auto"' in parser_source
    assert '"direct_updown_lift"' in parser_source
    assert 'default=0.40' in parser_source
    assert '"--extract-top-updown-retreat-distance"' in parser_source
    assert 'default=0.0,\n        help="兼容参数；默认不在抬升阶段同步后抽' in parser_source
    assert "convert_mixed_grasp_modes_to_front" not in parser_source
    assert '"--ik-only-raw"' in parser_source
    print("extract sequence definition passed: 13 centered tasks; 26 with both layouts")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
