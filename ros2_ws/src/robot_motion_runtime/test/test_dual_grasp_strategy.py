import math

import pytest

from robot_motion_runtime.dual_grasp_strategy import (
    DUAL_FRONT,
    DUAL_FRONT_EQUAL,
    DUAL_FRONT_LEFT_HIGH,
    DUAL_FRONT_RIGHT_HIGH,
    DUAL_TOP_EQUAL,
    DUAL_TOP_LEFT_HIGH,
    DUAL_TOP_RIGHT_HIGH,
    BOTTOM_ROW_FRONT_CENTER_Z_M,
    FRONT_TOOL_RPY,
    OUTER_BOX_GRASP_LATERAL_OFFSET_M,
    OUTER_BOX_GRASP_TARGET_Y_M,
    Pose6DValue,
    TOP_TOOL_RPY,
    classify_dual_grasp_strategy,
    degrade_top_target_to_front,
    match_box_row,
    promote_front_target_to_top,
    resolve_front_face_dual_grasp_strategy,
    resolve_dual_grasp_strategy,
    quaternion_xyzw,
    rpy_from_quaternion_xyzw,
)


def test_front_quaternion_survives_gimbal_lock_rpy_round_trip():
    expected = (math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0)
    roll, pitch, yaw = rpy_from_quaternion_xyzw(expected)
    actual = quaternion_xyzw(Pose6DValue(0.0, 0.0, 0.0, roll, pitch, yaw))
    dot = sum(left * right for left, right in zip(expected, actual))
    assert abs(dot) == pytest.approx(1.0)


def test_outer_box_grasp_uses_40cm_target_without_attachment_offset():
    assert math.isclose(OUTER_BOX_GRASP_TARGET_Y_M, 0.40, abs_tol=1e-9)
    assert math.isclose(OUTER_BOX_GRASP_LATERAL_OFFSET_M, 0.0, abs_tol=1e-9)


def test_nominal_tool_orientations_match_planner_pose_contract():
    front = quaternion_xyzw(Pose6DValue(0.0, 0.0, 0.0, *FRONT_TOOL_RPY))
    top = quaternion_xyzw(Pose6DValue(0.0, 0.0, 0.0, *TOP_TOOL_RPY))
    assert front == pytest.approx((math.sqrt(0.5), 0.0, math.sqrt(0.5), 0.0))
    assert top == pytest.approx((1.0, 0.0, 0.0, 0.0))


def test_dual_front_requires_both_boxes_detached():
    strategy = classify_dual_grasp_strategy("front", "front", 1.0, 1.01)
    assert strategy.task_type == DUAL_FRONT_EQUAL == DUAL_FRONT
    assert strategy.left.require_full_detachment
    assert strategy.right.require_full_detachment
    assert strategy.left.front_clearance_levels == 1
    assert strategy.right.front_clearance_levels == 1
    assert strategy.left.retreat_priority > strategy.left.lift_priority


def test_unequal_front_requires_two_height_layers_for_lower_box():
    left_high = classify_dual_grasp_strategy("front", "front", 1.4, 1.0)
    assert left_high.task_type == DUAL_FRONT_LEFT_HIGH
    assert left_high.left.front_clearance_levels == 1
    assert left_high.right.front_clearance_levels == 2
    assert left_high.right.lift_priority > left_high.right.retreat_priority

    right_high = classify_dual_grasp_strategy("front", "front", 1.0, 1.4)
    assert right_high.task_type == DUAL_FRONT_RIGHT_HIGH
    assert right_high.left.front_clearance_levels == 2
    assert right_high.right.front_clearance_levels == 1
    assert right_high.left.lift_priority > right_high.left.retreat_priority


def test_equal_top_uses_height_tolerance():
    strategy = classify_dual_grasp_strategy("top_suction", "top", 1.0, 1.015, 0.02)
    assert strategy.task_type == DUAL_TOP_EQUAL
    assert strategy.left.require_full_detachment
    assert strategy.right.require_full_detachment
    assert strategy.left.lift_priority > strategy.left.retreat_priority


def test_unequal_top_relaxes_only_high_box():
    left_high = classify_dual_grasp_strategy("top", "top", 1.2, 1.0, 0.02)
    assert left_high.task_type == DUAL_TOP_LEFT_HIGH
    assert not left_high.left.require_full_detachment
    assert left_high.right.require_full_detachment

    right_high = classify_dual_grasp_strategy("top", "top", 1.0, 1.2, 0.02)
    assert right_high.task_type == DUAL_TOP_RIGHT_HIGH
    assert right_high.left.require_full_detachment
    assert not right_high.right.require_full_detachment


def test_mixed_modes_promote_front_arm_to_top_suction():
    strategy, left, right = resolve_dual_grasp_strategy(
        "front",
        "top",
        Pose6DValue(0.75, 0.2, 1.2, *FRONT_TOOL_RPY),
        Pose6DValue(0.9, -0.2, 1.4, *TOP_TOOL_RPY),
    )
    assert strategy.task_type == DUAL_TOP_EQUAL
    assert strategy.left.grasp_mode == "top_suction"
    assert strategy.right.grasp_mode == "top_suction"
    assert strategy.left.front_clearance_levels == 0
    assert strategy.right.front_clearance_levels == 0
    assert math.isclose(left.x, 0.9, abs_tol=1e-9)
    assert math.isclose(left.z, 1.4, abs_tol=1e-9)
    assert math.isclose(right.z, 1.4, abs_tol=1e-9)


def test_mixed_modes_classify_height_after_promoting_front_target():
    strategy, left, right = resolve_dual_grasp_strategy(
        "front",
        "top_suction",
        Pose6DValue(0.75, 0.2, 0.8, *FRONT_TOOL_RPY),
        Pose6DValue(0.9, -0.2, 1.4, *TOP_TOOL_RPY),
    )
    assert strategy.task_type == DUAL_TOP_RIGHT_HIGH
    assert strategy.left.front_clearance_levels == 0
    assert strategy.right.front_clearance_levels == 0
    assert strategy.left.grasp_mode == "top_suction"
    assert strategy.right.grasp_mode == "top_suction"
    assert right.z > left.z


def test_top_contact_converts_to_front_contact_using_box_geometry():
    top = Pose6DValue(0.9, 0.2, 1.4, *TOP_TOOL_RPY)
    front = degrade_top_target_to_front(top)
    assert math.isclose(front.x, 0.75, abs_tol=1e-9)
    assert math.isclose(front.y, 0.2, abs_tol=1e-9)
    assert math.isclose(front.z, 1.2, abs_tol=1e-9)


def test_front_contact_converts_to_top_contact_using_box_geometry():
    front = Pose6DValue(0.75, 0.2, 1.2, *FRONT_TOOL_RPY)
    top = promote_front_target_to_top(front)
    assert math.isclose(top.x, 0.9, abs_tol=1e-9)
    assert math.isclose(top.y, 0.2, abs_tol=1e-9)
    assert math.isclose(top.z, 1.4, abs_tol=1e-9)


def test_top_contact_conversion_preserves_measured_box_yaw():
    yaw_error = 0.10
    front = Pose6DValue(
        0.75,
        0.2,
        1.2,
        -math.pi + yaw_error,
        -math.pi / 2.0,
        0.0,
    )
    top = promote_front_target_to_top(front)
    assert math.isclose(top.x, front.x + 0.15 * math.cos(yaw_error), abs_tol=1e-9)
    assert math.isclose(top.y, front.y + 0.15 * math.sin(yaw_error), abs_tol=1e-9)
    assert math.isclose(top.z, front.z + 0.20, abs_tol=1e-9)


def test_row_match_accepts_measurement_offset_but_rejects_ambiguous_height():
    row2_center = BOTTOM_ROW_FRONT_CENTER_Z_M + 3.0 * 0.4
    match = match_box_row(row2_center + 0.08)
    assert match.row_from_top == 2
    assert math.isclose(match.residual_m, 0.08, abs_tol=1e-9)

    ambiguous = BOTTOM_ROW_FRONT_CENTER_Z_M + 0.2
    try:
        match_box_row(ambiguous)
    except ValueError as exc:
        assert "does not match a box row" in str(exc)
    else:
        raise AssertionError("ambiguous row height must be rejected")


def test_front_face_contract_derives_modes_and_preserves_position_offsets():
    row3_center = BOTTOM_ROW_FRONT_CENTER_Z_M + 2.0 * 0.4
    row4_center = BOTTOM_ROW_FRONT_CENTER_Z_M + 1.0 * 0.4
    left_front = Pose6DValue(
        0.93,
        0.44,
        row3_center + 0.06,
        *FRONT_TOOL_RPY,
    )
    right_front = Pose6DValue(
        0.88,
        -0.37,
        row4_center - 0.05,
        *FRONT_TOOL_RPY,
    )
    resolution = resolve_front_face_dual_grasp_strategy(left_front, right_front)

    assert resolution.left_row.row_from_top == 3
    assert resolution.right_row.row_from_top == 4
    assert resolution.strategy.task_type == DUAL_TOP_LEFT_HIGH
    assert resolution.strategy.left.grasp_mode == "top_suction"
    assert resolution.strategy.right.grasp_mode == "top_suction"
    assert math.isclose(resolution.left_tool_pose.x, left_front.x + 0.15, abs_tol=1e-9)
    assert math.isclose(resolution.left_tool_pose.y, left_front.y, abs_tol=1e-9)
    assert math.isclose(resolution.left_tool_pose.z, left_front.z + 0.20, abs_tol=1e-9)
    assert math.isclose(resolution.right_tool_pose.x, right_front.x + 0.15, abs_tol=1e-9)
    assert math.isclose(resolution.right_tool_pose.y, right_front.y, abs_tol=1e-9)
    assert math.isclose(resolution.right_tool_pose.z, right_front.z + 0.20, abs_tol=1e-9)


def test_thirteen_reference_tasks_are_classified_without_box_ids():
    pairs = [
        (1, 1),
        (1, 2),
        (2, 1),
        (2, 2),
        (2, 3),
        (3, 2),
        (3, 3),
        (3, 4),
        (4, 3),
        (4, 4),
        (4, 5),
        (5, 4),
        (5, 5),
    ]
    expected_modes = ["front"] * 7 + ["top_suction"] * 6
    for index, ((left_row, right_row), expected_mode) in enumerate(
        zip(pairs, expected_modes)
    ):
        left_z = BOTTOM_ROW_FRONT_CENTER_Z_M + (5 - left_row) * 0.4
        right_z = BOTTOM_ROW_FRONT_CENTER_Z_M + (5 - right_row) * 0.4
        left = Pose6DValue(
            0.90 + 0.01 * ((index % 3) - 1),
            0.40 + 0.02 * ((index % 2) - 0.5),
            left_z + 0.04,
            *FRONT_TOOL_RPY,
        )
        right = Pose6DValue(
            0.90 - 0.01 * ((index % 3) - 1),
            -0.40 - 0.02 * ((index % 2) - 0.5),
            right_z - 0.04,
            *FRONT_TOOL_RPY,
        )
        resolution = resolve_front_face_dual_grasp_strategy(left, right)
        assert resolution.left_row.row_from_top == left_row
        assert resolution.right_row.row_from_top == right_row
        assert resolution.strategy.left.grasp_mode == expected_mode
        assert resolution.strategy.right.grasp_mode == expected_mode
