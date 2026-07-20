#!/usr/bin/python3
from __future__ import annotations

import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts/execute_l6_r8_mock_live.py"
SPEC = importlib.util.spec_from_file_location("execute_l6_r8_mock_live", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def sample(
    time_s: float,
    value: float,
    stage_index: int,
    updown: float,
    stage: str | None = None,
):
    return (
        time_s,
        {"left_joint1": value},
        {"stage_index": stage_index, "stage": stage or f"stage_{stage_index}", "updown": updown},
    )


def main() -> int:
    zero = {name: 0.0 for name in MODULE.EXECUTION_JOINT_NAMES}
    goal = dict(zero)
    goal["left_joint1"] = 0.1
    home = MODULE.resample_segment(
        zero,
        goal,
        start_time=0.0,
        hz=10.0,
        max_joint_speed_deg_s=10.0,
        minimum_duration_s=3.0,
    )
    assert home[-1][0] >= 3.0

    samples = [
        sample(0.0, 0.0, 0, 0.30),
        sample(1.0, 1.0, 0, 0.15),
        sample(1.1, 1.1, 1, 0.15),
        sample(1.6, 1.6, 1, 0.15),
        sample(1.7, 1.7, 2, 0.15),
        sample(2.0, 2.0, 2, 0.30),
        sample(2.1, 2.1, 3, 0.30, "task/selected_loaded_to_place"),
        sample(3.0, 3.0, 3, 0.20, "task/selected_loaded_to_place"),
        sample(3.1, 3.1, 4, 0.20, "task/selected_place_to_loaded"),
        sample(4.0, 4.0, 4, 0.30, "task/selected_place_to_loaded"),
    ]

    pre_contact, contact, post_contact = MODULE.split_task_execution_phases(samples)
    assert len(pre_contact) == 2
    assert len(contact) == 3
    assert len(post_contact) == 7

    assert contact[0][0] == pre_contact[-1][0]
    assert contact[0][1] == pre_contact[-1][1]
    assert contact[0][2]["stage_index"] == 1
    assert contact[0][2]["updown"] == pre_contact[-1][2]["updown"]

    assert post_contact[0][0] == contact[-1][0]
    assert post_contact[0][1] == contact[-1][1]
    assert post_contact[0][2]["stage_index"] == 2

    assert MODULE.phase_updown_samples(pre_contact) == [(0.0, 0.30), (1.0, 0.15)]
    assert MODULE.phase_updown_samples(contact)[0] == (0.0, 0.15)
    assert MODULE.phase_updown_samples(post_contact)[0] == (0.0, 0.15)

    extract, place, returned = MODULE.split_post_contact_place_cycle(post_contact, required=True)
    assert len(extract) == 3
    assert len(place) == 3
    assert len(returned) == 3
    assert place[0][1] == extract[-1][1]
    assert returned[0][1] == place[-1][1]
    assert MODULE.phase_updown_samples(place)[-1] == (1.0, 0.20)
    assert MODULE.phase_updown_samples(returned)[-1] == (1.0, 0.30)

    no_place = post_contact[:3]
    assert MODULE.split_post_contact_place_cycle(no_place, required=False) == (no_place, [], [])
    try:
        MODULE.split_post_contact_place_cycle(no_place, required=True)
    except RuntimeError as exception:
        assert "missing the required" in str(exception)
    else:
        raise AssertionError("required place cycle must be rejected when absent")

    try:
        MODULE.split_task_execution_phases(samples[:2])
    except RuntimeError as exception:
        assert "missing required execution phases" in str(exception)
    else:
        raise AssertionError("missing phases must be rejected")

    print("live execution phase split passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
