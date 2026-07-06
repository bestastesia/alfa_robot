from alfa_robot_execution_bridge.joints import (
    DEFAULT_DIRECTION_SIGNS,
    DEFAULT_JOINT_NAMES,
    EXECUTION_JOINT_NAMES,
    FLIPPED_JOINT_NAMES,
    REAL_CONTROLLER_JOINT_NAMES,
    ROS_TO_ETHERCAT_SIGN_BY_JOINT,
    direction_signs_for,
    ethercat_to_ros_position,
    ros_to_ethercat_position,
)


def test_joint_direction_contract_is_canonical():
    assert DEFAULT_JOINT_NAMES is EXECUTION_JOINT_NAMES
    assert len(EXECUTION_JOINT_NAMES) == 13
    assert len(REAL_CONTROLLER_JOINT_NAMES) == 13
    assert set(REAL_CONTROLLER_JOINT_NAMES) == set(EXECUTION_JOINT_NAMES)
    assert tuple(FLIPPED_JOINT_NAMES) == ('left_joint3', 'left_joint5', 'right_joint2')
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
        'right_joint4': 1.0,
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
    assert ros_to_ethercat_position('right_joint5', 0.25) == 0.25
