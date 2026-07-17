import pytest

from alfa_robot_execution_bridge.joints import (
    DEFAULT_DIRECTION_SIGNS,
    DEFAULT_JOINT_NAMES,
    EXECUTION_JOINT_NAMES,
    FLIPPED_JOINT_NAMES,
    REAL_CONTROLLER_JOINT_NAMES,
    ROS_TO_ETHERCAT_SIGN_BY_JOINT,
    UPDOWN_LOGICAL_LOWER_M,
    UPDOWN_LOGICAL_UPPER_M,
    UPDOWN_PHYSICAL_LOWER_M,
    UPDOWN_PHYSICAL_UPPER_M,
    UPDOWN_PHYSICAL_ZERO_OFFSET_M,
    direction_signs_for,
    ethercat_to_ros_position,
    logical_to_physical_updown,
    physical_to_logical_updown,
    ros_to_ethercat_position,
)


def test_joint_direction_contract_is_canonical():
    assert DEFAULT_JOINT_NAMES is EXECUTION_JOINT_NAMES
    assert len(EXECUTION_JOINT_NAMES) == 13
    assert len(REAL_CONTROLLER_JOINT_NAMES) == 13
    assert set(REAL_CONTROLLER_JOINT_NAMES) == set(EXECUTION_JOINT_NAMES)
    assert tuple(FLIPPED_JOINT_NAMES) == ('left_joint3', 'left_joint5', 'right_joint2', 'right_joint4')
    assert DEFAULT_DIRECTION_SIGNS == direction_signs_for(EXECUTION_JOINT_NAMES)
    assert ROS_TO_ETHERCAT_SIGN_BY_JOINT == {
        'left_joint1': 1.0,
        'left_joint2': 1.0,
        'left_joint3': -1.0,
        'left_joint4': 1.0,
        'left_joint5': -1.0,
        'left_joint6': 1.0,
        'right_joint1': 1.0,
        'right_joint2': -1.0,
        'right_joint3': 1.0,
        'right_joint4': -1.0,
        'right_joint5': 1.0,
        'right_joint6': 1.0,
        'turn': 1.0,
    }


def test_direction_conversion_round_trip():
    for joint_name in EXECUTION_JOINT_NAMES:
        command_value = ros_to_ethercat_position(joint_name, 0.25)
        assert ethercat_to_ros_position(joint_name, command_value) == 0.25

    assert ros_to_ethercat_position('left_joint3', 0.25) == -0.25
    assert ros_to_ethercat_position('left_joint5', 0.25) == -0.25
    assert ros_to_ethercat_position('right_joint2', 0.25) == -0.25
    assert ros_to_ethercat_position('right_joint4', 0.25) == -0.25
    assert ros_to_ethercat_position('right_joint5', 0.25) == 0.25


def test_updown_conversion_endpoints():
    assert UPDOWN_LOGICAL_LOWER_M == 0.0
    assert UPDOWN_LOGICAL_UPPER_M == 0.7
    assert UPDOWN_PHYSICAL_ZERO_OFFSET_M == 0.08
    assert UPDOWN_PHYSICAL_LOWER_M == 0.08
    assert UPDOWN_PHYSICAL_UPPER_M == pytest.approx(0.78)
    assert logical_to_physical_updown(0.0) == pytest.approx(0.08)
    assert logical_to_physical_updown(0.7) == pytest.approx(0.78)
    assert physical_to_logical_updown(0.08) == pytest.approx(0.0)
    assert physical_to_logical_updown(0.78) == pytest.approx(0.7)


def test_updown_conversion_round_trip():
    for logical_m in (0.0, 0.12, 0.3, 0.55, 0.7):
        physical_m = logical_to_physical_updown(logical_m)
        assert physical_to_logical_updown(physical_m) == pytest.approx(logical_m)


def test_updown_conversion_rejects_out_of_range():
    with pytest.raises(ValueError):
        logical_to_physical_updown(-0.01)
    with pytest.raises(ValueError):
        logical_to_physical_updown(0.71)
    with pytest.raises(ValueError):
        physical_to_logical_updown(0.07)
    with pytest.raises(ValueError):
        physical_to_logical_updown(0.79)
