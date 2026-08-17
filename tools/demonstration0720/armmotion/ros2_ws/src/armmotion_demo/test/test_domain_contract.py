import pytest

from robot_motion_interfaces.action import ExecuteMotionStage
from robot_motion_interfaces.msg import DualArmPoseTargets
from armmotion_demo.domain_motion_server import DomainMotionServer, pregrasp_entry_mode
from armmotion_demo.manual_domain_task import _quaternion_from_rpy
from armmotion_demo.stage_contract import (
    align_target_pair_to_lower_height,
    canonicalize_grasp_pose_orientation,
    canonicalize_stage_target_orientations,
    planning_task_from_resolved_targets,
    planning_task_from_stage_goal,
    resolve_dual_stage_targets,
    validate_stage_pose_targets,
)
from robot_motion_runtime.dual_grasp_strategy import (
    BOTTOM_ROW_FRONT_CENTER_Z_M,
    BOX_ROW_PITCH_M,
)
from robot_system_interfaces.msg import ErrorCode, ErrorInfo


def goal(left_y=0.5, right_y=-0.5):
    message = ExecuteMotionStage.Goal()
    message.execution_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP
    for side, y in (
        ("left", left_y),
        ("right", right_y),
    ):
        setattr(
            message.targets,
            f"{side}_grasp_mode",
            DualArmPoseTargets.GRASP_MODE_SIDE_SUCTION,
        )
        pose = getattr(message.targets, f"{side}_pose")
        pose.position.x = 0.9
        pose.position.y = y
        pose.position.z = 1.6
        pose.orientation.x = 0.70710678
        pose.orientation.z = 0.70710678
    return message


def test_public_action_result_uses_structured_error_contract():
    result = ExecuteMotionStage.Result()
    result.ok = False
    result.error = DomainMotionServer._error(
        ErrorCode.MOTION_PLANNING_FAILED,
        "planner failed",
        retryable=True,
    )
    result.diagnostic = "diagnostic only"

    assert result.ok is False
    assert result.error.code == ErrorCode.MOTION_PLANNING_FAILED
    assert result.error.retryable is True
    assert result.error.severity == ErrorInfo.WARN
    assert result.error.source == "motion"


def test_planning_failure_is_mapped_to_canonical_error_codes():
    assert DomainMotionServer._planning_error_code("analytic IK no solution") == (
        ErrorCode.MOTION_IK_NO_SOLUTION
    )
    assert DomainMotionServer._planning_error_code("检测到碰撞") == (
        ErrorCode.MOTION_COLLISION_DETECTED
    )
    assert DomainMotionServer._planning_error_code("planner timeout") == (
        ErrorCode.MOTION_PLANNING_FAILED
    )


def test_domain_task_uses_explicit_left_right_poses():
    task = planning_task_from_stage_goal(goal(), "cycle-1")
    assert task.code == "cycle-1"
    assert task.effective_distance_m == pytest.approx(0.9)
    assert task.scene_y_shift == pytest.approx(0.0)
    assert task.left_front_face_pose.y == pytest.approx(0.5)
    assert task.right_front_face_pose.y == pytest.approx(-0.5)


def test_domain_top_target_is_actual_top_surface_center():
    message = goal()
    row3_center = BOTTOM_ROW_FRONT_CENTER_Z_M + 2.0 * BOX_ROW_PITCH_M
    for side in ("left", "right"):
        setattr(
            message.targets,
            f"{side}_grasp_mode",
            DualArmPoseTargets.GRASP_MODE_TOP_SUCTION,
        )
        pose = getattr(message.targets, f"{side}_pose")
        pose.position.x = 0.85
        pose.position.z = row3_center + 0.20
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0
    task = planning_task_from_stage_goal(message, "cycle-top")
    assert (task.left_row, task.right_row) == (3, 3)
    assert task.left_suction_surface_pose.x == pytest.approx(0.85)
    assert task.left_box_center_pose.z == pytest.approx(row3_center)
    assert task.left_front_face_pose.x == pytest.approx(0.70)
    assert task.effective_distance_m == pytest.approx(0.70)


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
    message.targets.left_grasp_mode = DualArmPoseTargets.GRASP_MODE_NO_MOVE
    message.targets.right_grasp_mode = DualArmPoseTargets.GRASP_MODE_TOP_SUCTION
    message.targets.left_pose.orientation.x = 0.0
    message.targets.left_pose.orientation.z = 0.0
    validate_stage_pose_targets(message)


def test_single_right_arm_target_is_mirrored_to_left_arm():
    message = goal(left_y=0.0, right_y=-0.43)
    message.targets.left_grasp_mode = DualArmPoseTargets.GRASP_MODE_NO_MOVE
    message.targets.right_grasp_mode = DualArmPoseTargets.GRASP_MODE_TOP_SUCTION
    message.targets.left_pose.orientation.x = 0.0
    message.targets.left_pose.orientation.z = 0.0
    targets = resolve_dual_stage_targets(message)
    assert targets.mirrored_from == "right"
    assert targets.left_grasp_mode == DualArmPoseTargets.GRASP_MODE_TOP_SUCTION
    assert targets.right_grasp_mode == DualArmPoseTargets.GRASP_MODE_TOP_SUCTION
    assert targets.left_pose.position.y == pytest.approx(0.43)
    assert targets.right_pose.position.y == pytest.approx(-0.43)
    assert targets.left_pose.orientation == targets.right_pose.orientation


def test_single_left_grasp_target_builds_mirrored_dual_task():
    message = goal(left_y=0.47, right_y=0.0)
    message.targets.right_grasp_mode = DualArmPoseTargets.GRASP_MODE_NO_MOVE
    message.targets.right_pose.orientation.x = 0.0
    message.targets.right_pose.orientation.z = 0.0
    task = planning_task_from_stage_goal(message, "single-left")
    assert task.left_suction_surface_pose.y == pytest.approx(0.47)
    assert task.right_suction_surface_pose.y == pytest.approx(-0.47)
    assert task.scene_y_shift == pytest.approx(0.0)


def test_both_no_move_targets_are_rejected():
    message = goal()
    message.targets.left_grasp_mode = DualArmPoseTargets.GRASP_MODE_NO_MOVE
    message.targets.right_grasp_mode = DualArmPoseTargets.GRASP_MODE_NO_MOVE
    with pytest.raises(ValueError, match="不能同时"):
        validate_stage_pose_targets(message)


def test_pose_target_rejects_unknown_mode():
    message = goal()
    message.targets.left_grasp_mode = 99
    with pytest.raises(ValueError, match="left_grasp_mode"):
        validate_stage_pose_targets(message)


def test_pregrasp_can_start_after_recapture():
    assert pregrasp_entry_mode(
        active_plan=None,
        recapture_sample=object(),
        cycle_id="motion-cycle-000001",
        next_stage=ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP,
    ) == "after_recapture"


def test_pregrasp_can_skip_recapture_from_idle_state():
    assert pregrasp_entry_mode(
        active_plan=None,
        recapture_sample=None,
        cycle_id="",
        next_stage=ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW,
    ) == "skip_recapture"


def test_pregrasp_cannot_skip_outside_idle_state():
    assert pregrasp_entry_mode(
        active_plan=object(),
        recapture_sample=None,
        cycle_id="",
        next_stage=ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW,
    ) is None
    assert pregrasp_entry_mode(
        active_plan=None,
        recapture_sample=None,
        cycle_id="",
        next_stage=ExecuteMotionStage.Goal.EXECUTION_STAGE_APPROACH,
    ) is None


def test_rpy_is_converted_to_normalized_quaternion():
    quaternion = _quaternion_from_rpy(3.141592653589793, 0.0, 1.5707963267948966)
    assert sum(value * value for value in quaternion) == pytest.approx(1.0)


def test_side_grasp_orientation_is_canonicalized_without_moving_position():
    message = goal().targets.left_pose
    message.position.x = 0.81
    message.position.y = 0.29
    message.position.z = 1.62
    message.orientation.x = 0.68
    message.orientation.y = 0.03
    message.orientation.z = 0.73
    message.orientation.w = 0.02

    corrected, deviation = canonicalize_grasp_pose_orientation(
        message,
        DualArmPoseTargets.GRASP_MODE_SIDE_SUCTION,
    )

    expected = _quaternion_from_rpy(3.141592653589793, -1.5707963267948966, 0.0)
    assert corrected.position == message.position
    assert tuple(
        getattr(corrected.orientation, name) for name in ("x", "y", "z", "w")
    ) == pytest.approx(expected)
    assert deviation > 0.0


def test_top_grasp_orientation_is_canonicalized_without_moving_position():
    message = goal().targets.right_pose
    message.orientation.x = 0.99
    message.orientation.y = 0.02
    message.orientation.z = -0.01
    message.orientation.w = 0.03

    corrected, _ = canonicalize_grasp_pose_orientation(
        message,
        DualArmPoseTargets.GRASP_MODE_TOP_SUCTION,
    )

    expected = _quaternion_from_rpy(3.141592653589793, 0.0, 0.0)
    assert tuple(
        getattr(corrected.orientation, name) for name in ("x", "y", "z", "w")
    ) == pytest.approx(expected)


def test_stage_target_orientations_are_canonicalized_when_requested_for_pregrasp():
    message = goal()
    message.execution_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP
    message.targets.right_grasp_mode = DualArmPoseTargets.GRASP_MODE_TOP_SUCTION
    message.targets.left_pose.orientation.x = 0.61
    message.targets.left_pose.orientation.y = 0.12
    message.targets.left_pose.orientation.z = 0.73
    message.targets.left_pose.orientation.w = 0.27
    message.targets.right_pose.orientation.x = 0.74
    message.targets.right_pose.orientation.y = 0.16
    message.targets.right_pose.orientation.z = 0.51
    message.targets.right_pose.orientation.w = 0.41

    corrected, deviations = canonicalize_stage_target_orientations(message)

    expected_left = _quaternion_from_rpy(
        3.141592653589793,
        -1.5707963267948966,
        0.0,
    )
    expected_right = _quaternion_from_rpy(3.141592653589793, 0.0, 0.0)
    assert tuple(
        getattr(corrected.targets.left_pose.orientation, name)
        for name in ("x", "y", "z", "w")
    ) == pytest.approx(expected_left)
    assert tuple(
        getattr(corrected.targets.right_pose.orientation, name)
        for name in ("x", "y", "z", "w")
    ) == pytest.approx(expected_right)
    assert deviations["left"] > 0.0
    assert deviations["right"] > 0.0


def test_target_pair_uses_lower_z_without_changing_each_xy():
    message = goal(left_y=0.47, right_y=-0.39)
    message.targets.left_pose.position.x = 0.78
    message.targets.left_pose.position.z = 1.64
    message.targets.right_pose.position.x = 0.83
    message.targets.right_pose.position.z = 1.59

    aligned = align_target_pair_to_lower_height(resolve_dual_stage_targets(message))

    assert aligned.left_pose.position.x == pytest.approx(0.78)
    assert aligned.left_pose.position.y == pytest.approx(0.47)
    assert aligned.right_pose.position.x == pytest.approx(0.83)
    assert aligned.right_pose.position.y == pytest.approx(-0.39)
    assert aligned.left_pose.position.z == pytest.approx(1.59)
    assert aligned.right_pose.position.z == pytest.approx(1.59)


def test_lower_height_alignment_drives_equal_row_planning_task():
    message = goal(left_y=0.46, right_y=-0.42)
    message.targets.left_pose.position.x = 0.79
    message.targets.right_pose.position.x = 0.84
    message.targets.left_pose.position.z = (
        BOTTOM_ROW_FRONT_CENTER_Z_M + 2.0 * BOX_ROW_PITCH_M
    )
    message.targets.right_pose.position.z = (
        BOTTOM_ROW_FRONT_CENTER_Z_M + BOX_ROW_PITCH_M
    )

    resolved = resolve_dual_stage_targets(message)
    original_task = planning_task_from_resolved_targets(resolved, "uneven-original")
    aligned = align_target_pair_to_lower_height(resolved)
    task = planning_task_from_resolved_targets(aligned, "uneven-input")

    assert original_task.left_row != original_task.right_row
    assert task.left_row == original_task.right_row
    assert task.right_row == original_task.right_row
    assert task.left_suction_surface_pose.x == pytest.approx(0.79)
    assert task.left_suction_surface_pose.y == pytest.approx(0.46)
    assert task.right_suction_surface_pose.x == pytest.approx(0.84)
    assert task.right_suction_surface_pose.y == pytest.approx(-0.42)


def test_mirrored_single_target_keeps_equal_height():
    message = goal(left_y=0.0, right_y=-0.43)
    message.targets.left_grasp_mode = DualArmPoseTargets.GRASP_MODE_NO_MOVE
    message.targets.right_grasp_mode = DualArmPoseTargets.GRASP_MODE_TOP_SUCTION
    message.targets.right_pose.position.z = 1.23
    message.targets.left_pose.orientation.x = 0.0
    message.targets.left_pose.orientation.z = 0.0

    aligned = align_target_pair_to_lower_height(resolve_dual_stage_targets(message))

    assert aligned.mirrored_from == "right"
    assert aligned.left_pose.position.z == pytest.approx(1.23)
    assert aligned.right_pose.position.z == pytest.approx(1.23)
