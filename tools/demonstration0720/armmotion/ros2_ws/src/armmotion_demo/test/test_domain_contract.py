import math
import threading
from types import SimpleNamespace

import pytest

from robot_motion_interfaces.action import ExecuteMotionStage
from robot_motion_interfaces.msg import DualArmPoseTargets
from armmotion_demo.domain_motion_server import (
    DomainMotionServer,
    cache_result_diagnostic,
    fixed_side_recapture_target,
    named_joint_pose_target,
    pregrasp_entry_mode,
    pregrasp_planning_start_sample,
    turn_stage_target,
)
from armmotion_demo.common import MotionSample
from armmotion_demo.manual_domain_task import _quaternion_from_rpy
from armmotion_demo.stage_contract import (
    align_target_pair_to_average_x,
    align_target_pair_to_lower_height,
    canonicalize_grasp_pose_orientation,
    canonicalize_stage_target_orientations,
    planning_task_from_resolved_targets,
    planning_task_from_stage_goal,
    resolve_dual_stage_targets,
    validate_default_stage_targets,
    validate_stage_pose_targets,
)
from robot_motion_runtime.dual_grasp_strategy import (
    BOTTOM_ROW_FRONT_CENTER_Z_M,
    BOX_ROW_PITCH_M,
)
from robot_system_interfaces.msg import ErrorCode, ErrorInfo
from rclpy.action import GoalResponse


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


def test_cache_result_diagnostic_reports_requested_and_selected_grid():
    diagnostic = cache_result_diagnostic(
        {
            "trajectory_cache_path": "/tmp/x_78cm_y_p06cm_row_3.json.gz",
            "trajectory_cache_nearest_success_used": True,
            "trajectory_cache_requested_distance_m": 0.78,
            "trajectory_cache_distance_m": 0.78,
            "trajectory_cache_requested_lateral_offset_m": 0.07,
            "trajectory_cache_lateral_offset_m": 0.06,
        }
    )
    assert "cache_key=x_78cm_y_p06cm_row_3" in diagnostic
    assert "nearest_success=True" in diagnostic
    assert "requested_y_offset=+0.070m" in diagnostic
    assert "selected_y_offset=+0.060m" in diagnostic


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


def test_domain_sends_place_as_one_continuous_controller_goal():
    trajectory = [
        MotionSample(0.0, {"left_joint1": 0.0}, 0.3, {}),
        MotionSample(1.0, {"left_joint1": 0.1}, 0.1, {}),
    ]
    calls = []
    server = DomainMotionServer.__new__(DomainMotionServer)
    server._lock = threading.RLock()
    server._active_plan = SimpleNamespace(
        action_trajectories={"place": trajectory}
    )
    server._hardware = SimpleNamespace(
        execute_segment=lambda samples, label: calls.append((samples, label))
        or {"duration_s": 1.0}
    )
    goal_handle = SimpleNamespace(is_cancel_requested=False)

    duration = server._run_plan_action(goal_handle, "place", "抽离到放置")

    assert duration == pytest.approx(1.0)
    assert calls == [(trajectory, "抽离到放置")]


def test_domain_task_uses_explicit_left_right_poses():
    task = planning_task_from_stage_goal(goal(), "cycle-1")
    assert task.code == "cycle-1"
    assert task.effective_distance_m == pytest.approx(0.9)
    assert task.scene_y_shift == pytest.approx(0.0)
    assert task.left_front_face_pose.y == pytest.approx(0.5)
    assert task.right_front_face_pose.y == pytest.approx(-0.5)


def test_target_pair_average_x_preserves_each_y():
    targets = resolve_dual_stage_targets(goal(left_y=0.47, right_y=-0.35))
    targets.left_pose.position.x = 0.72
    targets.right_pose.position.x = 0.78

    aligned = align_target_pair_to_average_x(targets)

    assert aligned.left_pose.position.x == pytest.approx(0.75)
    assert aligned.right_pose.position.x == pytest.approx(0.75)
    assert aligned.left_pose.position.y == pytest.approx(0.47)
    assert aligned.right_pose.position.y == pytest.approx(-0.35)


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
    assert targets.left_pose.position.y == pytest.approx(0.39)
    assert targets.right_pose.position.y == pytest.approx(-0.43)
    assert targets.left_pose.position.y - targets.right_pose.position.y == pytest.approx(0.82)
    assert 0.5 * (targets.left_pose.position.y + targets.right_pose.position.y) == pytest.approx(-0.02)
    assert targets.left_pose.orientation == targets.right_pose.orientation


def test_single_left_grasp_target_builds_mirrored_dual_task():
    message = goal(left_y=0.47, right_y=0.0)
    message.targets.right_grasp_mode = DualArmPoseTargets.GRASP_MODE_NO_MOVE
    message.targets.right_pose.orientation.x = 0.0
    message.targets.right_pose.orientation.z = 0.0
    task = planning_task_from_stage_goal(message, "single-left")
    assert task.left_suction_surface_pose.y == pytest.approx(0.47)
    assert task.right_suction_surface_pose.y == pytest.approx(-0.35)
    assert task.scene_y_shift == pytest.approx(0.06)


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


def test_pregrasp_cannot_start_from_stale_recapture_context():
    assert pregrasp_entry_mode(
        active_plan=None,
        recapture_sample=object(),
        cycle_id="motion-cycle-000001",
        next_stage=ExecuteMotionStage.Goal.EXECUTION_STAGE_PREGRASP,
    ) is None


def test_pregrasp_can_start_from_idle_state():
    assert pregrasp_entry_mode(
        active_plan=None,
        recapture_sample=None,
        cycle_id="",
        next_stage=ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW,
    ) == "from_idle"


def test_pregrasp_from_idle_plans_from_fresh_joint_state():
    fresh_current = MotionSample(
        0.0,
        {"right_joint2": -1.19},
        0.3,
        {"stage": "joint_states"},
    )

    selected = pregrasp_planning_start_sample(
        entry_mode="from_idle",
        current_sample=fresh_current,
        recapture_sample=None,
    )

    assert selected is fresh_current


def test_turn_stage_changes_only_turn():
    current = MotionSample(
        0.0,
        {
            **{
                f"{side}_joint{index}": 0.01 * index
                for side in ("left", "right")
                for index in range(1, 7)
            },
            "turn": 0.2,
        },
        0.31,
        {},
    )

    target = turn_stage_target(current, -1.25)

    assert target.joints["turn"] == pytest.approx(-1.25)
    assert target.updown_m == pytest.approx(current.updown_m)
    for name, value in current.joints.items():
        if name != "turn":
            assert target.joints[name] == pytest.approx(value)


@pytest.mark.parametrize(
    ("named_pose", "expected_label", "left_joint4", "right_joint4"),
    (
        (
            ExecuteMotionStage.Goal.NAMED_JOINT_POSE_REMOTE_CAMERA_VIEW,
            "remote_camera_view",
            -1.309444351,
            -1.204246822,
        ),
        (
            ExecuteMotionStage.Goal.NAMED_JOINT_POSE_ARM_CONVERGED,
            "arm_converged",
            -0.685905129,
            -0.516603981,
        ),
    ),
)
def test_named_joint_pose_preserves_turn_and_updown(
    named_pose,
    expected_label,
    left_joint4,
    right_joint4,
):
    current = MotionSample(
        0.0,
        {
            **{
                f"{side}_joint{index}": 0.0
                for side in ("left", "right")
                for index in range(1, 7)
            },
            "turn": -0.75,
        },
        0.26,
        {},
    )

    target, label = named_joint_pose_target(current, named_pose)

    assert label == expected_label
    assert target.joints["turn"] == pytest.approx(-0.75)
    assert target.updown_m == pytest.approx(0.26)
    assert target.joints["left_joint4"] == pytest.approx(left_joint4)
    assert target.joints["right_joint4"] == pytest.approx(right_joint4)


def test_non_pose_stage_requires_default_targets():
    message = ExecuteMotionStage.Goal()
    message.execution_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_TURN
    validate_default_stage_targets(message)

    message.targets.left_grasp_mode = DualArmPoseTargets.GRASP_MODE_NO_MOVE
    with pytest.raises(ValueError, match="left_grasp_mode"):
        validate_default_stage_targets(message)


def test_stage_request_accepts_turn_and_named_pose_fields_only_in_own_stage():
    server = DomainMotionServer.__new__(DomainMotionServer)
    server.get_parameter = lambda _: SimpleNamespace(value=True)

    turn_goal = ExecuteMotionStage.Goal()
    turn_goal.execution_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_TURN
    turn_goal.turn_target_rad = -1.2
    server._validate_stage_request(turn_goal)

    named_goal = ExecuteMotionStage.Goal()
    named_goal.execution_stage = (
        ExecuteMotionStage.Goal.EXECUTION_STAGE_NAMED_JOINT_POSE
    )
    named_goal.named_joint_pose = (
        ExecuteMotionStage.Goal.NAMED_JOINT_POSE_ARM_CONVERGED
    )
    server._validate_stage_request(named_goal)

    named_goal.turn_target_rad = 0.1
    with pytest.raises(ValueError, match="仅 TURN"):
        server._validate_stage_request(named_goal)


@pytest.mark.parametrize(
    ("stage", "named_pose", "expected_label", "expected_hold_turn"),
    (
        (
            ExecuteMotionStage.Goal.EXECUTION_STAGE_TURN,
            ExecuteMotionStage.Goal.NAMED_JOINT_POSE_UNSPECIFIED,
            "Turn 独立运动",
            False,
        ),
        (
            ExecuteMotionStage.Goal.EXECUTION_STAGE_NAMED_JOINT_POSE,
            ExecuteMotionStage.Goal.NAMED_JOINT_POSE_REMOTE_CAMERA_VIEW,
            "命名姿态 remote_camera_view",
            True,
        ),
    ),
)
def test_standalone_stage_executes_without_creating_grasp_context(
    stage,
    named_pose,
    expected_label,
    expected_hold_turn,
):
    current = MotionSample(
        0.0,
        {
            **{
                f"{side}_joint{index}": 0.0
                for side in ("left", "right")
                for index in range(1, 7)
            },
            "turn": 0.1,
        },
        0.24,
        {},
    )
    calls = []
    server = DomainMotionServer.__new__(DomainMotionServer)
    server._lock = threading.RLock()
    server._busy = False
    server._goal_reserved = True
    server._scene_unknown = False
    server._active_plan = None
    server._recapture_sample = None
    server._cycle_id = ""
    server._next_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW
    server._publish_readiness = lambda: None
    server._current_sample_for_planning = lambda: current
    server._planner = SimpleNamespace(
        smooth_motion_samples=lambda samples, context_label: (
            samples,
            {"mode": context_label},
        )
    )
    server._hardware = SimpleNamespace(
        execute_segment=lambda samples, label, **kwargs: calls.append(
            (samples, label, kwargs)
        )
        or {"duration_s": 1.0}
    )
    server.get_logger = lambda: SimpleNamespace(error=lambda _: None)

    request = ExecuteMotionStage.Goal()
    request.execution_stage = stage
    request.turn_target_rad = -1.0
    request.named_joint_pose = named_pose
    if stage != ExecuteMotionStage.Goal.EXECUTION_STAGE_TURN:
        request.turn_target_rad = 0.0
    terminal = []
    goal_handle = SimpleNamespace(
        request=request,
        is_cancel_requested=False,
        publish_feedback=lambda _: None,
        succeed=lambda: terminal.append("succeeded"),
        abort=lambda: terminal.append("aborted"),
        canceled=lambda: terminal.append("canceled"),
    )

    result = server._execute_stage(goal_handle)

    assert result.ok is True
    assert terminal == ["succeeded"]
    assert len(calls) == 1
    samples, label, kwargs = calls[0]
    assert label == expected_label
    assert kwargs.get("hold_turn", True) is expected_hold_turn
    assert samples[-1].updown_m == pytest.approx(current.updown_m)
    assert server._active_plan is None
    assert server._cycle_id == ""


def test_standalone_stage_is_rejected_during_active_grasp_flow():
    server = DomainMotionServer.__new__(DomainMotionServer)
    server._lock = threading.RLock()
    server._busy = False
    server._goal_reserved = False
    server._scene_unknown = False
    server._active_plan = object()
    server._recapture_sample = None
    server._cycle_id = "motion-cycle-000001"
    server._next_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_APPROACH
    server._publish_readiness = lambda: None
    server.get_parameter = lambda _: SimpleNamespace(value=True)
    server.get_logger = lambda: SimpleNamespace(error=lambda _: None)
    request = ExecuteMotionStage.Goal()
    request.execution_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_TURN
    request.turn_target_rad = -1.0

    assert server._accept_stage_goal(request) == GoalResponse.REJECT
    assert server._goal_reserved is False

    server._active_plan = None
    server._cycle_id = ""
    server._next_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW
    assert server._accept_stage_goal(request) == GoalResponse.ACCEPT
    assert server._goal_reserved is True


@pytest.mark.parametrize(
    ("target_z", "expected_row", "expected_updown"),
    ((1.30, 1, 0.40), (0.90, 2, 0.0)),
)
def test_side_recapture_uses_fixed_joint_pose_by_row(
    target_z,
    expected_row,
    expected_updown,
):
    message = goal()
    message.execution_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW
    message.targets.left_pose.position.z = target_z
    message.targets.right_pose.position.z = target_z + 0.02
    current = MotionSample(
        0.0,
        {
            **{
                f"{side}_joint{index}": 0.1
                for side in ("left", "right")
                for index in range(1, 7)
            },
            "turn": 0.0,
        },
        0.2,
        {"stage": "joint_states"},
    )

    result = fixed_side_recapture_target(
        current,
        resolve_dual_stage_targets(message),
        row_split_z_m=1.10,
        first_row_updown_m=0.40,
        second_row_updown_m=0.0,
    )

    assert result is not None
    target, row, average_z = result
    assert row == expected_row
    assert average_z == pytest.approx(target_z + 0.01)
    assert target.updown_m == pytest.approx(expected_updown)
    expected_deg = (0.0, -88.0, 135.0, -40.0, 0.0, 0.0)
    for side in ("left", "right"):
        for index, value_deg in enumerate(expected_deg, start=1):
            assert target.joints[f"{side}_joint{index}"] == pytest.approx(
                math.radians(value_deg)
            )


def test_top_recapture_keeps_existing_ik_path():
    message = goal()
    message.execution_stage = ExecuteMotionStage.Goal.EXECUTION_STAGE_CAMERA_VIEW
    message.targets.left_grasp_mode = DualArmPoseTargets.GRASP_MODE_TOP_SUCTION
    message.targets.right_grasp_mode = DualArmPoseTargets.GRASP_MODE_TOP_SUCTION
    current = MotionSample(0.0, {"turn": 0.0}, 0.2, {})

    assert fixed_side_recapture_target(
        current,
        resolve_dual_stage_targets(message),
        row_split_z_m=1.10,
        first_row_updown_m=0.40,
        second_row_updown_m=0.0,
    ) is None


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
