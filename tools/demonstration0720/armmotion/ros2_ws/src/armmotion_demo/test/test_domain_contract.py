import pytest

from alfa_motion_interfaces.action import ExecuteMotionStage
from alfa_motion_interfaces.msg import MotionPoseTarget
from armmotion_demo.manual_domain_task import _quaternion_from_rpy
from armmotion_demo.stage_contract import (
    planning_task_from_stage_goal,
    validate_stage_pose_targets,
)


def goal(left_y=0.5, right_y=-0.5):
    message = ExecuteMotionStage.Goal()
    message.context.request_id = "request-1"
    message.context.task_id = "task-1"
    message.context.sequence_id = 1
    message.stage = ExecuteMotionStage.Goal.MOVE_TO_PREGRASP
    for target, y in (
        (message.left_target, left_y),
        (message.right_target, right_y),
    ):
        target.grasp_mode = MotionPoseTarget.SIDE_SUCTION
        stamped = target.pose
        stamped.header.frame_id = "base_link"
        stamped.pose.position.x = 0.9
        stamped.pose.position.y = y
        stamped.pose.position.z = 1.6
        stamped.pose.orientation.x = 0.70710678
        stamped.pose.orientation.z = 0.70710678
    return message


def test_domain_task_uses_explicit_left_right_poses():
    task = planning_task_from_stage_goal(goal())
    assert task.code == "task-1"
    assert task.effective_distance_m == pytest.approx(0.9)
    assert task.scene_y_shift == pytest.approx(0.0)
    assert task.left_front_face_pose.y == pytest.approx(0.5)
    assert task.right_front_face_pose.y == pytest.approx(-0.5)


def test_domain_task_rejects_non_base_link_body_pose():
    message = goal()
    message.left_target.pose.header.frame_id = "map"
    with pytest.raises(ValueError, match="left_target.pose"):
        planning_task_from_stage_goal(message)


def test_domain_task_rejects_zero_quaternion():
    message = goal()
    message.left_target.pose.pose.orientation.x = 0.0
    message.left_target.pose.pose.orientation.y = 0.0
    message.left_target.pose.pose.orientation.z = 0.0
    message.left_target.pose.pose.orientation.w = 0.0
    with pytest.raises(ValueError, match="四元数为零"):
        planning_task_from_stage_goal(message)


def test_recapture_pair_accepts_mode_field_without_changing_pose_validation():
    message = goal()
    message.stage = ExecuteMotionStage.Goal.MOVE_TO_RECAPTURE
    message.left_target.grasp_mode = MotionPoseTarget.NO_MOVE
    message.right_target.grasp_mode = MotionPoseTarget.TOP_SUCTION
    validate_stage_pose_targets(message)


def test_pose_target_rejects_unknown_mode():
    message = goal()
    message.left_target.grasp_mode = 99
    with pytest.raises(ValueError, match="grasp_mode"):
        validate_stage_pose_targets(message)


def test_rpy_is_converted_to_normalized_quaternion():
    quaternion = _quaternion_from_rpy(3.141592653589793, 0.0, 1.5707963267948966)
    assert sum(value * value for value in quaternion) == pytest.approx(1.0)
