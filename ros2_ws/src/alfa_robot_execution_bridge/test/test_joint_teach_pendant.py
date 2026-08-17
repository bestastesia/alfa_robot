import math

import pytest

from alfa_robot_execution_bridge.joint_teach_pendant import (
    JOINT_SPEC_BY_NAME,
    REQUIRED_TF_LINKS,
    TF_ROOT_FRAME,
    build_smoothstep_trajectory,
    trajectory_duration_s,
)
from alfa_robot_execution_bridge.joints import RT_CONTROL_JOINT_NAMES


def zero_state():
    return {name: 0.0 for name in RT_CONTROL_JOINT_NAMES}


def duration_seconds(duration):
    return duration.sec + duration.nanosec * 1e-9


def test_rt_control_tf_contract_covers_all_controlled_body_links():
    assert TF_ROOT_FRAME == "base_footprint"
    assert REQUIRED_TF_LINKS == (
        "base_link",
        "pitch",
        "turn",
        "updown",
        "left_joint1",
        "left_joint2",
        "left_joint3",
        "left_joint4",
        "left_joint5",
        "left_joint6",
        "right_joint1",
        "right_joint2",
        "right_joint3",
        "right_joint4",
        "right_joint5",
        "right_joint6",
    )


def test_teach_pendant_uses_rt_control_arm_limits():
    expected = [90.0, 90.0, 140.0, 180.0, 125.0, 179.0]
    for side in ('left', 'right'):
        for index, limit_deg in enumerate(expected, start=1):
            spec = JOINT_SPEC_BY_NAME[f'{side}_joint{index}']
            assert spec.lower == pytest.approx(-limit_deg)
            assert spec.upper == pytest.approx(limit_deg)


def test_duration_extends_for_rotary_velocity_limit():
    current = zero_state()
    target = zero_state()
    target["left_joint1"] = math.radians(20.0)
    duration = trajectory_duration_s(
        current,
        target,
        minimum_s=0.1,
        rotary_velocity_rad_s=math.radians(10.0),
        rotary_acceleration_rad_s2=math.radians(1000.0),
        updown_velocity_m_s=1.0,
        updown_acceleration_m_s2=1.0,
    )
    assert duration == pytest.approx(3.75)


def test_trajectory_has_full_contract_and_smooth_endpoints():
    current = zero_state()
    target = zero_state()
    target["right_joint2"] = math.radians(-15.0)
    target["updown"] = 0.1
    trajectory, duration = build_smoothstep_trajectory(
        current,
        target,
        hz=30.0,
        minimum_s=1.0,
        rotary_velocity_rad_s=math.radians(20.0),
        rotary_acceleration_rad_s2=math.radians(20.0),
        updown_velocity_m_s=0.05,
        updown_acceleration_m_s2=0.05,
    )
    assert trajectory.joint_names == RT_CONTROL_JOINT_NAMES
    assert len(trajectory.points) == math.ceil(duration * 30.0) + 1
    assert trajectory.points[0].positions == pytest.approx(
        [current[name] for name in RT_CONTROL_JOINT_NAMES]
    )
    assert trajectory.points[-1].positions == pytest.approx(
        [target[name] for name in RT_CONTROL_JOINT_NAMES]
    )
    assert trajectory.points[0].velocities == pytest.approx([0.0] * 14)
    assert trajectory.points[-1].velocities == pytest.approx([0.0] * 14, abs=1e-12)
    assert duration_seconds(trajectory.points[-1].time_from_start) == pytest.approx(duration)
