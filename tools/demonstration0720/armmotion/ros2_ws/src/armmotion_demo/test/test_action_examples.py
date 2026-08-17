import json
import math

import pytest

from armmotion_demo import action_examples
from armmotion_demo.action_examples import as_single_arm, load_examples


def test_all_successful_cached_action_examples_are_complete():
    examples = load_examples()
    assert len(examples) == 53
    assert (80, 3) not in {
        (item["distance_cm"], item["row"]) for item in examples
    }
    assert (80, 4) not in {
        (item["distance_cm"], item["row"]) for item in examples
    }
    assert all(len(item["goals"]) == 5 for item in examples)


def test_recapture_is_thirty_five_centimetres_behind_suction_surface():
    for item in load_examples():
        camera = item["goals"][0]["goal"]["targets"]
        pregrasp = item["goals"][1]["goal"]["targets"]
        for side in ("left", "right"):
            lhs = camera[f"{side}_pose"]["position"]
            rhs = pregrasp[f"{side}_pose"]["position"]
            distance = math.sqrt(sum((lhs[axis] - rhs[axis]) ** 2 for axis in ("x", "y", "z")))
            assert distance == pytest.approx(0.35)


def test_stage_modes_match_cached_grasp_family():
    for item in load_examples():
        expected = 2 if item["row"] <= 2 else 1
        for stage_index in (0, 1):
            targets = item["goals"][stage_index]["goal"]["targets"]
            assert targets["left_grasp_mode"] == expected
            assert targets["right_grasp_mode"] == expected
        for stage_index in (2, 3, 4):
            targets = item["goals"][stage_index]["goal"]["targets"]
            assert targets["left_grasp_mode"] == 3
            assert targets["right_grasp_mode"] == 3


def test_single_arm_example_marks_other_arm_no_move():
    example = as_single_arm(load_examples()[0], "left")
    for stage_index in (0, 1):
        targets = example["goals"][stage_index]["goal"]["targets"]
        assert targets["left_grasp_mode"] == 2
        assert targets["right_grasp_mode"] == 3
        assert targets["right_pose"]["position"] == {"x": 0.0, "y": 0.0, "z": 0.0}


def test_json_output_separates_diagnostics_from_wire_goals(monkeypatch, capsys):
    monkeypatch.setattr("sys.argv", ["dump_cached_action_examples"])
    action_examples.main()
    payload = json.loads(capsys.readouterr().out)
    assert len(payload["wire_goal_groups"]) == 53
    first_goal = payload["wire_goal_groups"]["x_70cm_row_1"][0]
    assert set(first_goal) == {"execution_stage", "targets"}
    assert "label" not in first_goal
    assert "cache_key" not in first_goal
