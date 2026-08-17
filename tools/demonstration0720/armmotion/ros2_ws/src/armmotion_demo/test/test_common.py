import math
import threading

import pytest

from armmotion_demo.common import (
    DIRECT_LIFT_TASKS,
    MotionSample,
    front_face_poses_for_task,
    front_face_task_request_fields,
    loaded_joint_map,
    planning_task_from_front_face_poses,
    planning_task_from_suction_surface_poses,
    parse_task_code,
    retime_segment,
    split_execution_stages,
    suction_surface_poses_for_task,
    validate_stage_contracts,
)
from armmotion_demo.controller_interpolated_rerun import (
    segment_controller_trace,
    segment_raw_trace,
)
from armmotion_demo import hardware_executor as hardware_executor_module
from armmotion_demo.hardware_executor import HardwareExecutor
from armmotion_demo.domain_motion_server import DomainMotionServer
from alfa_robot_execution_bridge.joints import RT_CONTROL_JOINT_NAMES


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
        sample("selected_loaded_plan", 4.0, 0.0, 0.1),
        sample("selected_loaded_to_place", 4.0, 0.0, 0.1),
        sample("selected_loaded_to_place", 6.0, -0.1, 0.1),
        sample("selected_place_to_loaded", 6.0, -0.1, 0.1),
        sample("selected_place_to_loaded", 7.0, 0.0, 0.3),
    ]


def test_task_code_contract():
    task = parse_task_code("a3", 0.9, 0.7)
    assert task.code == "A3"
    assert (task.left_box_id, task.right_box_id) == (7, 9)
    assert task.index in DIRECT_LIFT_TASKS
    assert task.task_layout == "right_shift_0p1"
    assert task.effective_distance_m == pytest.approx(0.7)


def test_all_ten_task_codes_are_supported():
    expected_pairs = [(1, 3), (4, 6), (7, 9), (10, 12), (13, 15)]
    for layout in ("A", "B"):
        for index, expected_pair in enumerate(expected_pairs, start=1):
            task = parse_task_code(f"{layout}{index}", 0.9, 0.7)
            assert (task.left_box_id, task.right_box_id) == expected_pair
            assert task.extraction_mode == (
                "box_pose_rrt" if index <= 2 else "direct_updown_lift"
            )


def test_task_thread_contract_contains_only_front_face_poses():
    fixture = parse_task_code("A4", 0.9, 0.7)
    left_pose, right_pose = front_face_poses_for_task(fixture)
    payload = front_face_task_request_fields("camera-request-42", left_pose, right_pose)
    assert set(payload) == {"request_id", "left", "right"}
    assert set(payload["left"]) == {"pose_6d"}
    assert set(payload["right"]) == {"pose_6d"}
    serialized = str(payload)
    assert "box_id" not in serialized
    assert "grasp_mode" not in serialized
    assert "task_code" not in serialized


def test_algorithm_derives_rows_and_modes_from_noisy_front_face_poses():
    fixture = parse_task_code("B4", 0.9, 0.7)
    left_pose, right_pose = front_face_poses_for_task(fixture)
    left_pose = type(left_pose)(
        x=left_pose.x + 0.03,
        y=left_pose.y - 0.02,
        z=left_pose.z + 0.04,
        roll=left_pose.roll,
        pitch=left_pose.pitch,
        yaw=left_pose.yaw,
    )
    right_pose = type(right_pose)(
        x=right_pose.x - 0.02,
        y=right_pose.y + 0.01,
        z=right_pose.z - 0.05,
        roll=right_pose.roll,
        pitch=right_pose.pitch,
        yaw=right_pose.yaw,
    )
    task = planning_task_from_front_face_poses("camera-request-43", left_pose, right_pose)
    assert (task.left_row, task.right_row) == (4, 4)
    assert task.grasp_family == "top_suction"
    assert task.left_front_face_pose == left_pose
    assert task.right_front_face_pose == right_pose
    assert task.left_tool_pose.x == pytest.approx(left_pose.x + 0.15)
    assert task.left_tool_pose.z == pytest.approx(left_pose.z + 0.20)
    assert task.extraction_mode == "box_pose_rrt"


def test_algorithm_uses_third_row_top_suction_strategy():
    fixture = parse_task_code("B3", 0.9, 0.7)
    left_pose, right_pose = suction_surface_poses_for_task(fixture)
    task = planning_task_from_suction_surface_poses(
        "camera-request-row3",
        left_pose,
        right_pose,
        "top_suction",
        "top_suction",
    )
    assert (task.left_row, task.right_row) == (3, 3)
    assert task.grasp_family == "top_suction"
    assert task.effective_distance_m == pytest.approx(0.7)


def test_pose_task_validation_accepts_collision_planned_top_extract_motion():
    fixture = parse_task_code("B4", 0.9, 0.7)
    left_pose, right_pose = front_face_poses_for_task(fixture)
    task = planning_task_from_front_face_poses("camera-request-top", left_pose, right_pose)
    samples = make_direct_lift_samples()
    extract_end = next(
        item
        for item in samples
        if item.context["stage"].endswith("selected_extract_step_1")
    )
    extract_end.joints[JOINT_NAMES[0]] = math.radians(3.0)
    stages = split_execution_stages(samples)
    validate_stage_contracts(task, stages, JOINT_NAMES)


def test_algorithm_rejects_height_between_rows_instead_of_guessing():
    fixture = parse_task_code("B1", 0.9, 0.7)
    left_pose, right_pose = front_face_poses_for_task(fixture)
    ambiguous_left = type(left_pose)(
        x=left_pose.x,
        y=left_pose.y,
        z=left_pose.z - 0.20,
        roll=left_pose.roll,
        pitch=left_pose.pitch,
        yaw=left_pose.yaw,
    )
    with pytest.raises(ValueError, match="does not match a box row"):
        planning_task_from_front_face_poses(
            "camera-request-44",
            ambiguous_left,
            right_pose,
        )


def test_loaded_pose_keeps_contract_names():
    result = loaded_joint_map(JOINT_NAMES)
    assert math.degrees(result["left_joint2"]) == pytest.approx(-45.0)
    assert math.degrees(result["right_joint3"]) == pytest.approx(120.0)
    assert result["turn"] == pytest.approx(0.0)


def test_planning_sample_ignores_external_turn():
    source = sample("x", 0.0, 0.2, 0.3)
    source.joints["turn"] = math.radians(-90.0)
    masked = DomainMotionServer._planning_sample_without_external_turn(source)
    assert masked.joints["turn"] == pytest.approx(0.0)
    assert masked.joints["left_joint1"] == pytest.approx(0.2)
    assert source.joints["turn"] == pytest.approx(math.radians(-90.0))


def test_split_and_validate_direct_lift():
    task = parse_task_code("B3", 0.9, 0.7)
    stages = split_execution_stages(make_direct_lift_samples())
    validate_stage_contracts(task, stages, JOINT_NAMES)
    assert len(stages) == 6
    assert len(stages[3]) == 2
    assert stages[5] == []


def test_stage_three_accepts_current_loaded_transition_height():
    samples = make_direct_lift_samples()
    for item in samples:
        if item.context["stage"].endswith("selected_pre_attach_loaded_to_pre_contact") and item.time_s == 1.0:
            item.updown_m = 0.35
        if item.context["stage"].endswith("selected_pre_attach_pre_contact_to_ik"):
            item.updown_m = 0.35
        if "selected_extract_step_" in item.context["stage"]:
            item.updown_m = 0.35
        if item.context["stage"].endswith("selected_loaded_plan"):
            item.updown_m = 0.1
        if item.context["stage"].endswith("selected_loaded_to_place") and item.time_s == 4.0:
            item.updown_m = 0.1
        if item.context["stage"].endswith("selected_place_to_loaded") and item.time_s == 7.0:
            item.updown_m = 0.3
    task = parse_task_code("B1", 0.9, 0.7)
    stages = split_execution_stages(samples)
    validate_stage_contracts(task, stages, JOINT_NAMES)
    assert stages[3][-1][-1].updown_m == pytest.approx(0.1)


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
    assert max(abs(item.updown_velocity_m_s) for item in result) <= 0.05 + 1e-9


def test_retime_30hz_respects_updown_speed_and_acceleration_limits():
    source = [
        sample("x", 0.0, 0.0, 0.3),
        sample("x", 0.1, math.radians(20.0), 0.4),
    ]
    result = retime_segment(
        source,
        JOINT_NAMES,
        rate_hz=30.0,
        max_joint_speed_deg_s=10.0,
        max_updown_speed_m_s=0.15,
        speed_scale=3.0,
    )
    assert 2.8 <= result[-1].time_s <= 2.9
    for previous, current in zip(result, result[1:]):
        assert current.time_s - previous.time_s == pytest.approx(1.0 / 30.0)
    assert max(
        abs(item.joint_velocities[JOINT_NAMES[0]]) for item in result
    ) <= math.radians(30.0) + 1e-9
    assert max(abs(item.updown_velocity_m_s) for item in result) <= 0.15 + 1e-9
    assert max(abs(item.updown_acceleration_m_s2) for item in result) <= 0.05 + 1e-9


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


def test_controller_rerun_trace_uses_250hz_quintic_samples_and_90hz_output():
    start = sample("x", 0.0, 0.0, 0.3)
    goal = sample("x", 0.8, 1.0, 0.4)
    start.joint_velocities = {name: 0.0 for name in JOINT_NAMES}
    goal.joint_velocities = {name: 0.0 for name in JOINT_NAMES}
    start.joint_accelerations = {name: 0.0 for name in JOINT_NAMES}
    goal.joint_accelerations = {name: 0.0 for name in JOINT_NAMES}

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
    assert quarter.updown_m == pytest.approx(0.310352, abs=0.002)


def test_raw_rerun_trace_keeps_algorithm_samples_without_interpolation():
    source = [
        sample("x", 0.0, 0.0, 0.3),
        sample("x", 0.1, 0.2, 0.35),
        sample("x", 0.2, 0.4, 0.4),
    ]
    display, sample_count = segment_raw_trace(source)
    assert sample_count == len(source)
    assert [item.controller_state.time_from_start for item in display] == [0.0, 0.1, 0.2]
    assert [item.updown_m for item in display] == pytest.approx([0.3, 0.35, 0.4])


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


def test_hardware_trajectory_crosses_joint_contract_as_full_14_axis(monkeypatch):
    position_calls = []
    velocity_calls = []
    acceleration_calls = []

    def contract_position(name, value):
        position_calls.append(name)
        return value

    def contract_velocity(name, value):
        velocity_calls.append(name)
        return value

    def contract_acceleration(name, value):
        acceleration_calls.append(name)
        return value

    monkeypatch.setattr(
        hardware_executor_module,
        "model_to_rt_control_position",
        contract_position,
    )
    monkeypatch.setattr(
        hardware_executor_module,
        "model_to_rt_control_velocity",
        contract_velocity,
    )
    monkeypatch.setattr(
        hardware_executor_module,
        "model_to_rt_control_acceleration",
        contract_acceleration,
    )
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
    assert trajectory.joint_names == list(RT_CONTROL_JOINT_NAMES)
    expected_calls = list(RT_CONTROL_JOINT_NAMES) * len(trajectory.points)
    assert position_calls == expected_calls
    assert velocity_calls == expected_calls
    assert acceleration_calls == expected_calls
    assert all(len(point.positions) == len(RT_CONTROL_JOINT_NAMES) for point in trajectory.points)
    assert all(len(point.velocities) == len(RT_CONTROL_JOINT_NAMES) for point in trajectory.points)
    assert all(len(point.accelerations) == len(RT_CONTROL_JOINT_NAMES) for point in trajectory.points)
    middle = trajectory.points[len(trajectory.points) // 2]
    left_joint6 = trajectory.joint_names.index("left_joint6")
    right_joint4 = trajectory.joint_names.index("right_joint4")
    updown = trajectory.joint_names.index("updown")
    assert middle.velocities[left_joint6] > 0.0
    assert middle.velocities[right_joint4] > 0.0
    assert middle.positions[left_joint6] == pytest.approx(samples[len(samples) // 2].joints["left_joint6"])
    assert middle.positions[updown] == pytest.approx(samples[len(samples) // 2].updown_m)
    frame_deltas = [
        current.joints[JOINT_NAMES[0]] - previous.joints[JOINT_NAMES[0]]
        for previous, current in zip(samples, samples[1:])
    ]
    assert min(frame_deltas) >= -1e-12
    assert frame_deltas[0] < max(frame_deltas)
    assert frame_deltas[-1] < max(frame_deltas)


def test_hardware_always_holds_external_turn():
    source = [
        sample("x", 0.0, 0.0, 0.3),
        sample("x", 0.2, 0.0, 0.3),
    ]
    source[-1].joints["turn"] = math.radians(-90.0)
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
    executor._latest_joints = {"turn": math.radians(12.0)}
    held = executor._make_trajectory(samples)
    turn_index = held.joint_names.index("turn")
    assert all(
        point.positions[turn_index] == pytest.approx(math.radians(12.0))
        for point in held.points
    )
    assert all(point.velocities[turn_index] == pytest.approx(0.0) for point in held.points)
    assert all(point.accelerations[turn_index] == pytest.approx(0.0) for point in held.points)


def test_hardware_rejects_cached_trajectory_outside_rt_control_limits():
    samples = [
        sample("x", 0.0, 0.0, 0.3),
        sample("x", 1.0, 0.0, 0.3),
    ]
    samples[-1].joints["left_joint3"] = math.radians(141.0)
    executor = HardwareExecutor.__new__(HardwareExecutor)
    executor._state_lock = threading.Lock()
    executor._latest_joints = {}
    with pytest.raises(ValueError, match="left_joint3.*rt-control limit"):
        executor._make_trajectory(samples)
