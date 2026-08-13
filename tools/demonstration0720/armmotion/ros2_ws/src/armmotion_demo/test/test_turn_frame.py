import math

import pytest
from geometry_msgs.msg import Pose, Transform

from armmotion_demo.turn_frame import compensate_pose_y, pose_at_zero_turn


def test_pose_at_zero_turn_recovers_position_and_orientation():
    turn_angle = math.radians(-94.0)
    turn_origin = (0.07134, 0.0, 0.185)
    virtual_position = (0.35, 0.4, 1.55)
    cosine = math.cos(turn_angle)
    sine = math.sin(turn_angle)
    relative_x = virtual_position[0] - turn_origin[0]
    relative_y = virtual_position[1] - turn_origin[1]

    measured = Pose()
    measured.position.x = turn_origin[0] + cosine * relative_x - sine * relative_y
    measured.position.y = turn_origin[1] + sine * relative_x + cosine * relative_y
    measured.position.z = virtual_position[2]
    measured.orientation.z = math.sin(0.5 * turn_angle)
    measured.orientation.w = math.cos(0.5 * turn_angle)

    base_to_turn = Transform()
    base_to_turn.translation.x = turn_origin[0]
    base_to_turn.translation.y = turn_origin[1]
    base_to_turn.translation.z = turn_origin[2]
    base_to_turn.rotation.z = math.sin(0.5 * turn_angle)
    base_to_turn.rotation.w = math.cos(0.5 * turn_angle)

    corrected = pose_at_zero_turn(measured, base_to_turn, turn_angle)

    assert corrected.position.x == pytest.approx(virtual_position[0], abs=1e-9)
    assert corrected.position.y == pytest.approx(virtual_position[1], abs=1e-9)
    assert corrected.position.z == pytest.approx(virtual_position[2], abs=1e-9)
    assert corrected.orientation.x == pytest.approx(0.0, abs=1e-9)
    assert corrected.orientation.y == pytest.approx(0.0, abs=1e-9)
    assert corrected.orientation.z == pytest.approx(0.0, abs=1e-9)
    assert abs(corrected.orientation.w) == pytest.approx(1.0, abs=1e-9)


def test_y_compensation_is_applied_after_turn_mapping():
    pose = Pose()
    pose.position.x = 0.8
    pose.position.y = 0.37
    pose.position.z = 1.6
    pose.orientation.w = 1.0

    corrected = compensate_pose_y(pose, -0.10)

    assert corrected.position.x == pytest.approx(0.8)
    assert corrected.position.y == pytest.approx(0.27)
    assert corrected.position.z == pytest.approx(1.6)
    assert pose.position.y == pytest.approx(0.37)


def test_y_compensation_rejects_non_finite_offset():
    pose = Pose()
    with pytest.raises(ValueError, match="Y 补偿"):
        compensate_pose_y(pose, math.nan)
