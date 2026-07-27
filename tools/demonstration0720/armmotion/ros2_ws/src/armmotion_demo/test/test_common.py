import math
import threading

import pytest

from armmotion_demo.common import (
    DIRECT_LIFT_TASKS,
    MotionSample,
    loaded_joint_map,
    parse_task_code,
    retime_segment,
    split_seven_stages,
    validate_stage_contracts,
)
from armmotion_demo.controller_interpolated_rerun import segment_controller_trace
from armmotion_demo.hardware_executor import HardwareExecutor
from alfa_robot_execution_bridge.joints import REAL_CONTROLLER_JOINT_NAMES


JOINT_NAMES = [f"left_joint{index}" for index in range(1, 7)] + [
    f"right_joint{index}" for index in range(1, 7)
] + ["turn"]


def sample(stage, time_s, value, updown):
    return MotionSample(
        time_s=time_s,
        joints={name: value for name in JOINT_NAMES},
        updown_m=updown,
        context={"stage": f"task/{stage}", "updown": updown},
    )


def make_direct_lift_samples():
    return [
        sample("selected_pre_attach_loaded_to_pre_contact", 0.0, 0.0, 0.3),
        sample("selected_pre_attach_loaded_to_pre_contact", 1.0, 0.1, 0.2),
        sample("selected_pre_attach_pre_contact_to_ik", 1.0, 0.1, 0.2),
        sample("selected_pre_attach_pre_contact_to_ik", 2.0, 0.2, 0.2),
        sample("selected_extract_step_0", 2.0, 0.2, 0.2),
        sample("selected_extract_step_1", 3.0, 0.2, 0.6),
        sample("selected_loaded_plan", 3.0, 0.2, 0.6),
        sample("selected_loaded_plan", 4.0, 0.0, 0.45),
        sample("selected_loaded_to_place_height", 4.0, 0.0, 0.45),
        sample("selected_loaded_to_place_height", 5.0, 0.0, 0.1),
        sample("selected_loaded_to_place_arms", 5.0, 0.0, 0.1),
        sample("selected_loaded_to_place_arms", 6.0, -0.1, 0.1),
        sample("selected_place_to_loaded", 6.0, -0.1, 0.1),
        sample("selected_place_to_loaded", 7.0, 0.0, 0.3),
    ]


def test_task_code_contract():
    task = parse_task_code("a3", 0.9, 0.7)
    assert task.code == "A3"
    assert (task.left_box_id, task.right_box_id) == (7, 9)
    assert task.index in DIRECT_LIFT_TASKS
    assert task.task_layout == "right_shift_0p1"
    assert task.effective_distance_m == pytest.approx(0.9)


def test_all_ten_task_codes_are_supported():
    expected_pairs = [(1, 3), (4, 6), (7, 9), (10, 12), (13, 15)]
    for layout in ("A", "B"):
        for index, expected_pair in enumerate(expected_pairs, start=1):
            task = parse_task_code(f"{layout}{index}", 0.9, 0.7)
            assert (task.left_box_id, task.right_box_id) == expected_pair
            assert task.extraction_mode == (
                "box_pose_rrt" if index <= 2 else "direct_updown_lift"
            )


def test_loaded_pose_keeps_contract_names():
    result = loaded_joint_map(JOINT_NAMES)
    assert math.degrees(result["left_joint2"]) == pytest.approx(-45.0)
    assert math.degrees(result["right_joint3"]) == pytest.approx(120.0)
    assert result["turn"] == pytest.approx(0.0)


def test_split_and_validate_direct_lift():
    task = parse_task_code("B3", 0.9, 0.7)
    stages = split_seven_stages(make_direct_lift_samples())
    validate_stage_contracts(task, stages, JOINT_NAMES)
    assert len(stages) == 7
    assert len(stages[3]) == 2
    assert stages[6] == []


def test_stage_three_preserves_extract_height_below_cap():
    samples = make_direct_lift_samples()
    for item in samples:
        if item.context["stage"].endswith("selected_pre_attach_loaded_to_pre_contact") and item.time_s == 1.0:
            item.updown_m = 0.35
        if item.context["stage"].endswith("selected_pre_attach_pre_contact_to_ik"):
            item.updown_m = 0.35
        if "selected_extract_step_" in item.context["stage"]:
            item.updown_m = 0.35
        if item.context["stage"].endswith("selected_loaded_plan"):
            item.updown_m = 0.35
        if item.context["stage"].endswith("selected_loaded_to_place_height") and item.time_s == 4.0:
            item.updown_m = 0.35
    task = parse_task_code("B1", 0.9, 0.7)
    stages = split_seven_stages(samples)
    validate_stage_contracts(task, stages, JOINT_NAMES)
    assert stages[3][-1][-1].updown_m == pytest.approx(0.35)


def test_retime_is_10hz_and_enforces_speed_limits():
    source = [
        sample("x", 0.0, 0.0, 0.3),
        sample("x", 0.1, math.radians(20.0), 0.4),
    ]
    result = retime_segment(
        source,
        JOINT_NAMES,
        rate_hz=10.0,
        max_joint_speed_deg_s=10.0,
        max_updown_speed_m_s=0.05,
    )
    assert result[-1].time_s >= 2.0
    for previous, current in zip(result, result[1:]):
        assert current.time_s - previous.time_s == pytest.approx(0.1)
        assert abs(current.joints[JOINT_NAMES[0]] - previous.joints[JOINT_NAMES[0]]) <= math.radians(1.0) + 1e-9
        assert abs(current.updown_m - previous.updown_m) <= 0.005 + 1e-9
    assert max(
        abs(item.joint_velocities[JOINT_NAMES[0]]) for item in result
    ) <= math.radians(10.0) + 1e-9


def test_retime_three_times_faster_respects_smooth_limits_at_30hz():
    source = [
        sample("x", 0.0, 0.0, 0.3),
        sample("x", 0.1, math.radians(20.0), 0.4),
    ]
    result = retime_segment(
        source,
        JOINT_NAMES,
        rate_hz=30.0,
        max_joint_speed_deg_s=10.0,
        max_updown_speed_m_s=0.05,
        speed_scale=3.0,
    )
    assert result[-1].time_s < 2.0
    for previous, current in zip(result, result[1:]):
        assert current.time_s - previous.time_s == pytest.approx(1.0 / 30.0)
    assert max(
        abs(item.joint_velocities[JOINT_NAMES[0]]) for item in result
    ) <= math.radians(30.0) + 1e-9


def test_retime_populates_continuous_velocity_with_acceleration_limit():
    source = [
        sample("x", 0.0, 0.0, 0.3),
        sample("x", 0.2, math.radians(12.0), 0.3),
        sample("x", 0.4, math.radians(15.0), 0.3),
    ]
    result = retime_segment(
        source,
        JOINT_NAMES,
        rate_hz=30.0,
        max_joint_speed_deg_s=10.0,
        max_joint_acceleration_deg_s2=60.0,
        max_updown_speed_m_s=0.05,
        speed_scale=3.0,
    )
    assert all(set(item.joint_velocities) == set(JOINT_NAMES) for item in result)
    assert all(value == pytest.approx(0.0) for value in result[0].joint_velocities.values())
    assert all(value == pytest.approx(0.0) for value in result[-1].joint_velocities.values())
    max_acceleration = 0.0
    for previous, current in zip(result, result[1:]):
        dt = current.time_s - previous.time_s
        for name in JOINT_NAMES:
            max_acceleration = max(
                max_acceleration,
                abs(current.joint_velocities[name] - previous.joint_velocities[name]) / dt,
            )
    assert max_acceleration <= math.radians(60.0) + 1e-6


def test_retime_only_slows_local_acceleration_transitions():
    source = [
        sample("x", index * 0.1, math.radians(index * 0.5), 0.3)
        for index in range(101)
    ]
    result = retime_segment(
        source,
        JOINT_NAMES,
        rate_hz=30.0,
        max_joint_speed_deg_s=10.0,
        max_joint_acceleration_deg_s2=60.0,
        max_updown_speed_m_s=0.05,
        speed_scale=3.0,
    )
    assert result[-1].time_s < 5.0
    assert max(
        abs(item.joint_velocities[JOINT_NAMES[0]]) for item in result
    ) > math.radians(10.0)


def test_retime_does_not_stop_at_same_direction_rrt_corner():
    source = [
        sample("x", 0.0, 0.0, 0.3),
        sample("x", 0.5, math.radians(10.0), 0.3),
        sample("x", 1.0, math.radians(20.0), 0.3),
    ]
    source[2].joints[JOINT_NAMES[1]] = math.radians(2.0)
    result = retime_segment(
        source,
        JOINT_NAMES,
        rate_hz=30.0,
        max_joint_speed_deg_s=10.0,
        max_joint_acceleration_deg_s2=60.0,
        max_updown_speed_m_s=0.05,
        speed_scale=3.0,
    )
    middle = min(
        result,
        key=lambda item: abs(item.joints[JOINT_NAMES[0]] - math.radians(10.0)),
    )
    assert middle.joint_velocities[JOINT_NAMES[0]] > math.radians(1.0)


def test_controller_rerun_trace_uses_250hz_cubic_samples_and_90hz_output():
    start = sample("x", 0.0, 0.0, 0.3)
    goal = sample("x", 0.8, 1.0, 0.4)
    start.joint_velocities = {name: 0.0 for name in JOINT_NAMES}
    goal.joint_velocities = {name: 0.0 for name in JOINT_NAMES}

    display, controller_count = segment_controller_trace(
        [start, goal],
        control_rate_hz=250.0,
        display_rate_hz=90.0,
    )

    assert controller_count == 201
    assert len(display) == 73
    quarter = min(
        display,
        key=lambda item: abs(item.controller_state.time_from_start - 0.2),
    )
    assert quarter.controller_state.positions[0] < 0.2
    assert quarter.updown_m == pytest.approx(0.325, abs=0.002)


def test_retime_zeroes_only_the_joint_that_reverses_direction():
    source = [
        sample("x", 0.0, 0.0, 0.3),
        sample("x", 0.5, math.radians(10.0), 0.3),
        sample("x", 1.0, 0.0, 0.3),
    ]
    for item, value in zip(source, (0.0, 5.0, 10.0)):
        item.joints[JOINT_NAMES[1]] = math.radians(value)
    result = retime_segment(
        source,
        JOINT_NAMES,
        rate_hz=30.0,
        max_joint_speed_deg_s=10.0,
        max_joint_acceleration_deg_s2=60.0,
        max_updown_speed_m_s=0.05,
        speed_scale=3.0,
    )
    middle = max(result, key=lambda item: item.joints[JOINT_NAMES[0]])
    assert middle.joint_velocities[JOINT_NAMES[0]] == pytest.approx(0.0, abs=1e-9)
    assert middle.joint_velocities[JOINT_NAMES[1]] > 0.0


def test_hardware_trajectory_contains_every_frame_velocity_without_zero_offset():
    source = [
        sample("x", 0.0, 0.0, 0.3),
        sample("x", 0.2, math.radians(10.0), 0.3),
    ]
    samples = retime_segment(
        source,
        JOINT_NAMES,
        rate_hz=30.0,
        max_joint_speed_deg_s=10.0,
        max_joint_acceleration_deg_s2=60.0,
        max_updown_speed_m_s=0.05,
        speed_scale=1.0,
    )
    executor = HardwareExecutor.__new__(HardwareExecutor)
    executor._state_lock = threading.Lock()
    executor._latest_joints = {}
    trajectory = executor._make_trajectory(samples)
    assert trajectory.joint_names == list(REAL_CONTROLLER_JOINT_NAMES)
    assert all(len(point.velocities) == len(REAL_CONTROLLER_JOINT_NAMES) for point in trajectory.points)
    middle = trajectory.points[len(trajectory.points) // 2]
    left_joint6 = trajectory.joint_names.index("left_joint6")
    right_joint4 = trajectory.joint_names.index("right_joint4")
    assert middle.velocities[left_joint6] > 0.0
    assert middle.velocities[right_joint4] < 0.0
    frame_deltas = [
        current.joints[JOINT_NAMES[0]] - previous.joints[JOINT_NAMES[0]]
        for previous, current in zip(samples, samples[1:])
    ]
    assert min(frame_deltas) >= -1e-12
    assert frame_deltas[0] < max(frame_deltas)
    assert frame_deltas[-1] < max(frame_deltas)
