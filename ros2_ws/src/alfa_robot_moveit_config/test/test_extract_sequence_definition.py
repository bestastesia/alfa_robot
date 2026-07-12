#!/usr/bin/python3
from __future__ import annotations

import importlib.util
from pathlib import Path
import sys


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "extract_sequence_rerun.py"
WORKSPACE_SRC = SCRIPT.parents[2]
sys.path.insert(0, str(WORKSPACE_SRC / "alfa_robot_rerun"))
SPEC = importlib.util.spec_from_file_location("extract_sequence_rerun", SCRIPT)
assert SPEC and SPEC.loader
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def main() -> int:
    expected_pairs = [
        (1, 3), (1, 8),
        (6, 3), (6, 8), (6, 13),
        (11, 8), (11, 13), (11, 18),
        (16, 13), (16, 18), (16, 23),
        (21, 18), (21, 23),
    ]
    pairs = MODULE.parse_pair_sequence(MODULE.DEFAULT_SEQUENCE)
    assert pairs == expected_pairs, pairs

    left_modes = MODULE.parse_arm_grasp_mode_sequence("", pairs, "left")
    right_modes = MODULE.parse_arm_grasp_mode_sequence("", pairs, "right")
    assert left_modes == [
        "front", "front",
        "front", "front", "front",
        "top_suction", "top_suction", "top_suction",
        "top_suction", "top_suction", "top_suction",
        "top_suction", "top_suction",
    ], left_modes
    assert right_modes == [
        "front", "front",
        "front", "front", "top_suction",
        "front", "top_suction", "top_suction",
        "top_suction", "top_suction", "top_suction",
        "top_suction", "top_suction",
    ], right_modes

    vehicle_modes = [
        MODULE.pair_vehicle_mode(left_mode, right_mode)
        for left_mode, right_mode in zip(left_modes, right_modes)
    ]
    assert vehicle_modes.count("front") == 4, vehicle_modes
    assert vehicle_modes.count("top_suction") == 9, vehicle_modes

    parser_source = SCRIPT.read_text()
    assert 'parser.add_argument("--loaded-updown", type=float, default=0.3)' in parser_source
    assert 'default="box_pose_rrt"' in parser_source
    assert '"--ik-only-raw"' in parser_source
    print("extract sequence definition passed: 13 pairs, 4 front vehicle tasks, 9 top/mixed tasks")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
