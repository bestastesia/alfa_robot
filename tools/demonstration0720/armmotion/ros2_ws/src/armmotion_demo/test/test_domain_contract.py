import pytest

from alfa_motion_interfaces.action import ExecuteMotionStage
from alfa_motion_interfaces.msg import DualArmPoseTargets
from armmotion_demo.manual_domain_task import _quaternion_from_rpy
from armmotion_demo.stage_contract import (
    planning_task_from_stage_goal,
    validate_stage_pose_targets,
)


def goal(left_y=0.5, right_y=-0.5):
    message = ExecuteMotionStage.Goal()
    message.execution_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP
    for side, y in (
        ("left", left_y),
        ("right", right_y),
    ):
        setattr(message.targets, f"{side}_stage", DualArmPoseTargets.STAGE_SIDE_SUCTION)
        pose = getattr(message.targets, f"{side}_pose")
        pose.position.x = 0.9
        pose.position.y = y
        pose.position.z = 1.6
        pose.orientation.x = 0.70710678
        pose.orientation.z = 0.70710678
    return message


def test_domain_task_uses_explicit_left_right_poses():
    task = planning_task_from_stage_goal(goal(), "cycle-1")
    assert task.code == "cycle-1"
    assert task.effective_distance_m == pytest.approx(0.9)
    assert task.scene_y_shift == pytest.approx(0.0)
    assert task.left_front_face_pose.y == pytest.approx(0.5)
    assert task.right_front_face_pose.y == pytest.approx(-0.5)


def test_domain_task_rejects_zero_quaternion():
    message = goal()
    message.targets.left_pose.orientation.x = 0.0
    message.targets.left_pose.orientation.y = 0.0
    message.targets.left_pose.orientation.z = 0.0
    message.targets.left_pose.orientation.w = 0.0
    with pytest.raises(ValueError, match="四元数为零"):
        planning_task_from_stage_goal(message, "cycle-1")


def test_recapture_pair_accepts_mode_field_without_changing_pose_validation():
    message = goal()
    message.execution_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW
    message.targets.left_stage = DualArmPoseTargets.STAGE_NO_MOVE
    message.targets.right_stage = DualArmPoseTargets.STAGE_TOP_SUCTION
    message.targets.left_pose.orientation.x = 0.0
    message.targets.left_pose.orientation.z = 0.0
    validate_stage_pose_targets(message)


def test_pose_target_rejects_unknown_mode():
    message = goal()
    message.targets.left_stage = 99
    with pytest.raises(ValueError, match="left_stage"):
        validate_stage_pose_targets(message)


def test_rpy_is_converted_to_normalized_quaternion():
    quaternion = _quaternion_from_rpy(3.141592653589793, 0.0, 1.5707963267948966)
    assert sum(value * value for value in quaternion) == pytest.approx(1.0)
