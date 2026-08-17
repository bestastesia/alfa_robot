import math

import pytest

from alfa_robot_execution_bridge.joints import (
    ARM_JOINT_POSITION_LIMITS_RAD,
    DEFAULT_DIRECTION_SIGNS,
    DEFAULT_JOINT_NAMES,
    ETHERCAT_ZERO_OFFSET_BY_JOINT,
    EXECUTION_JOINT_NAMES,
    FLIPPED_JOINT_NAMES,
    REAL_CONTROLLER_JOINT_NAMES,
    RT_CONTROL_ACTION_NAME,
    RT_CONTROL_DIRECTION_SIGN_BY_JOINT,
    RT_CONTROL_JOINT_NAMES,
    RT_CONTROL_POSITION_OFFSET_BY_JOINT,
    ROS_TO_ETHERCAT_SIGN_BY_JOINT,
    UPDOWN_LOGICAL_LOWER_M,
    UPDOWN_LOGICAL_UPPER_M,
    UPDOWN_PHYSICAL_LOWER_M,
    UPDOWN_PHYSICAL_UPPER_M,
    UPDOWN_PHYSICAL_ZERO_OFFSET_M,
    direction_signs_for,
    ethercat_zero_offset_for,
    ethercat_to_ros_position,
    logical_to_physical_updown,
    model_to_rt_control_acceleration,
    model_to_rt_control_position,
    model_to_rt_control_velocity,
    physical_to_logical_updown,
    ros_to_ethercat_position,
    ros_to_ethercat_velocity,
    require_arm_joint_in_range,
    rt_control_to_model_position,
)
from alfa_robot_execution_bridge.updown import (
    make_updown_command_data,
    synchronized_updown_velocity_mps,
    validate_updown_command_data,
)


def test_joint_direction_contract_is_canonical():
    assert DEFAULT_JOINT_NAMES is EXECUTION_JOINT_NAMES
    assert len(EXECUTION_JOINT_NAMES) == 13
    assert len(REAL_CONTROLLER_JOINT_NAMES) == 13
    assert set(REAL_CONTROLLER_JOINT_NAMES) == set(EXECUTION_JOINT_NAMES)
    assert tuple(FLIPPED_JOINT_NAMES) == ()
    assert DEFAULT_DIRECTION_SIGNS == direction_signs_for(EXECUTION_JOINT_NAMES)
    assert ROS_TO_ETHERCAT_SIGN_BY_JOINT == {
        'left_joint1': 1.0,
        'left_joint2': 1.0,
        'left_joint3': 1.0,
        'left_joint4': 1.0,
        'left_joint5': 1.0,
        'left_joint6': 1.0,
        'right_joint1': 1.0,
        'right_joint2': 1.0,
        'right_joint3': 1.0,
        'right_joint4': 1.0,
        'right_joint5': 1.0,
        'right_joint6': 1.0,
        'turn': 1.0,
    }
    assert set(ETHERCAT_ZERO_OFFSET_BY_JOINT) == set(EXECUTION_JOINT_NAMES)
    assert ethercat_zero_offset_for('left_joint6') == pytest.approx(0.05235987755982989)
    assert ethercat_zero_offset_for('right_joint6') == pytest.approx(-0.03490658503988659)


def test_rt_control_contract_is_full_14_axis():
    assert RT_CONTROL_ACTION_NAME == '/whole_body_jtc/follow_joint_trajectory'
    assert RT_CONTROL_JOINT_NAMES == [*REAL_CONTROLLER_JOINT_NAMES, 'updown']


def test_arm_position_limits_match_rt_control():
    expected = [90.0, 90.0, 140.0, 180.0, 125.0, 179.0]
    for side in ('left', 'right'):
        for index, limit_deg in enumerate(expected, start=1):
            lower, upper = ARM_JOINT_POSITION_LIMITS_RAD[f'{side}_joint{index}']
            assert math.degrees(lower) == pytest.approx(-limit_deg, abs=1e-8)
            assert math.degrees(upper) == pytest.approx(limit_deg, abs=1e-8)
    require_arm_joint_in_range('left_joint3', math.radians(140.0))
    with pytest.raises(ValueError, match='left_joint3.*rt-control limit'):
        require_arm_joint_in_range('left_joint3', math.radians(140.01))


def test_rt_control_public_boundary_uses_positive_direction_signs():
    assert set(RT_CONTROL_DIRECTION_SIGN_BY_JOINT) == set(RT_CONTROL_JOINT_NAMES)
    assert set(RT_CONTROL_POSITION_OFFSET_BY_JOINT) == set(RT_CONTROL_JOINT_NAMES)
    assert RT_CONTROL_DIRECTION_SIGN_BY_JOINT == {
        **ROS_TO_ETHERCAT_SIGN_BY_JOINT,
        'updown': 1.0,
    }
    assert all(offset == 0.0 for offset in RT_CONTROL_POSITION_OFFSET_BY_JOINT.values())
    assert all(sign == 1.0 for sign in RT_CONTROL_DIRECTION_SIGN_BY_JOINT.values())
    for joint_name, sign in RT_CONTROL_DIRECTION_SIGN_BY_JOINT.items():
        controller_position = model_to_rt_control_position(joint_name, 0.25)
        assert controller_position == pytest.approx(0.25 * sign)
        assert model_to_rt_control_velocity(joint_name, -0.5) == pytest.approx(-0.5 * sign)
        assert model_to_rt_control_acceleration(joint_name, 0.75) == pytest.approx(0.75 * sign)
        assert rt_control_to_model_position(joint_name, controller_position) == pytest.approx(0.25)
    assert model_to_rt_control_position('left_joint6', 0.0) == pytest.approx(0.0)
    assert model_to_rt_control_position('right_joint6', 0.0) == pytest.approx(0.0)
    with pytest.raises(KeyError):
        model_to_rt_control_position('unknown_joint', 0.0)


def test_direction_conversion_round_trip():
    for joint_name in EXECUTION_JOINT_NAMES:
        command_value = ros_to_ethercat_position(joint_name, 0.25)
        assert ethercat_to_ros_position(joint_name, command_value) == 0.25

    assert ros_to_ethercat_position('left_joint3', 0.25) == 0.25
    assert ros_to_ethercat_position('left_joint5', 0.25) == 0.25
    assert ros_to_ethercat_position('right_joint2', 0.25) == 0.25
    assert ros_to_ethercat_position('right_joint4', 0.25) == 0.25
    assert ros_to_ethercat_position('right_joint5', 0.25) == 0.25


def test_joint6_zero_offsets_apply_at_ethercat_boundary():
    assert ros_to_ethercat_position('left_joint6', 0.0) == pytest.approx(0.05235987755982989)
    assert ros_to_ethercat_position('right_joint6', 0.0) == pytest.approx(-0.03490658503988659)
    assert ethercat_to_ros_position('left_joint6', 0.05235987755982989) == pytest.approx(0.0)
    assert ethercat_to_ros_position('right_joint6', -0.03490658503988659) == pytest.approx(0.0)


def test_velocity_conversion_applies_direction_without_position_offset():
    assert ros_to_ethercat_velocity('left_joint6', 0.25) == pytest.approx(0.25)
    assert ros_to_ethercat_velocity('right_joint6', 0.25) == pytest.approx(0.25)
    assert ros_to_ethercat_velocity('right_joint4', 0.25) == pytest.approx(0.25)


def test_updown_conversion_endpoints():
    # 2026-07-19 实机复测：电机物理值与 URDF/逻辑值相同，合同转换保留但零偏移。
    assert UPDOWN_LOGICAL_LOWER_M == 0.0
    assert UPDOWN_LOGICAL_UPPER_M == 0.7
    assert UPDOWN_PHYSICAL_ZERO_OFFSET_M == 0.0
    assert UPDOWN_PHYSICAL_LOWER_M == 0.0
    assert UPDOWN_PHYSICAL_UPPER_M == pytest.approx(0.7)
    # 逻辑范围端点精确映射电机满行程端点。
    assert logical_to_physical_updown(0.0) == pytest.approx(0.0)
    assert logical_to_physical_updown(0.7) == pytest.approx(0.7)
    assert physical_to_logical_updown(0.0) == pytest.approx(0.0)
    assert physical_to_logical_updown(0.7) == pytest.approx(0.7)
    assert logical_to_physical_updown(0.28) == pytest.approx(0.28)
    assert physical_to_logical_updown(0.20) == pytest.approx(0.20)


def test_updown_physical_command_never_exceeds_motor_range():
    # 电机物理行程 [0, 0.7] 绝不可超出：logical 换算后强制夹紧。
    # 规划域与物理域均为 [0, 0.7]；clamp 是最后一道硬防线。
    assert logical_to_physical_updown(0.0) == pytest.approx(0.0)
    assert logical_to_physical_updown(0.7) == pytest.approx(0.7)
    # 任何输入（含越界）的输出都落在 [0, 0.7]。
    for logical_m in (-1.0, 0.0, 0.05, 0.08, 0.3, 0.7, 0.9, 1.5):
        physical_m = logical_to_physical_updown(logical_m)
        assert UPDOWN_PHYSICAL_LOWER_M <= physical_m <= UPDOWN_PHYSICAL_UPPER_M


def test_updown_conversion_round_trip_within_clampable_range():
    # logical 落在 [0,0.7] 时 round-trip 精确还原。
    for logical_m in (0.0, 0.08, 0.12, 0.3, 0.55, 0.7):
        physical_m = logical_to_physical_updown(logical_m)
        assert physical_to_logical_updown(physical_m) == pytest.approx(logical_m)


def test_updown_command_contract_has_four_atomic_fields():
    data = make_updown_command_data(0.3, 0.05, 0.06, 0.07)
    assert data == pytest.approx([0.3, 0.05, 0.06, 0.07])
    assert validate_updown_command_data(data) == pytest.approx((0.3, 0.05, 0.06, 0.07))
    with pytest.raises(ValueError):
        validate_updown_command_data([0.3])
    with pytest.raises(ValueError):
        make_updown_command_data(0.3, 0.0, 0.05, 0.05)
    with pytest.raises(ValueError):
        make_updown_command_data(0.3, 0.21, 0.05, 0.05)
    with pytest.raises(ValueError):
        make_updown_command_data(0.3, 0.05, float("nan"), 0.05)


def test_updown_phase_velocity_matches_final_arrival_time():
    samples = [(0.0, 0.1), (1.0, 0.12), (4.0, 0.3)]
    assert synchronized_updown_velocity_mps(samples, 0.05) == pytest.approx(0.05)
    assert synchronized_updown_velocity_mps([(0.0, 0.3), (2.0, 0.3)], 0.05) is None
    with pytest.raises(ValueError):
        synchronized_updown_velocity_mps([(0.0, 0.1), (1.0, 0.3)], 0.05)
