import math

from robot_motion_runtime.dual_grasp_strategy import (
    DUAL_FRONT,
    DUAL_FRONT_EQUAL,
    DUAL_FRONT_LEFT_HIGH,
    DUAL_FRONT_RIGHT_HIGH,
    DUAL_TOP_EQUAL,
    DUAL_TOP_LEFT_HIGH,
    DUAL_TOP_RIGHT_HIGH,
    MIXED_DEGRADED_TO_FRONT,
    MIXED_DEGRADED_TO_FRONT_EQUAL,
    MIXED_DEGRADED_TO_FRONT_RIGHT_HIGH,
    Pose6DValue,
    classify_dual_grasp_strategy,
    degrade_top_target_to_front,
    resolve_dual_grasp_strategy,
)


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

    right_high = classify_dual_grasp_strategy("front", "front", 1.0, 1.4)
    assert right_high.task_type == DUAL_FRONT_RIGHT_HIGH
    assert right_high.left.front_clearance_levels == 2
    assert right_high.right.front_clearance_levels == 1


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


def test_mixed_modes_degrade_to_dual_front():
    strategy, left, right = resolve_dual_grasp_strategy(
        "front",
        "top",
        Pose6DValue(0.75, 0.2, 1.2, math.pi, math.pi / 2.0, math.pi),
        Pose6DValue(0.9, -0.2, 1.4, math.pi, 0.0, math.pi),
    )
    assert strategy.task_type == MIXED_DEGRADED_TO_FRONT_EQUAL == MIXED_DEGRADED_TO_FRONT
    assert strategy.left.grasp_mode == "front"
    assert strategy.right.grasp_mode == "front"
    assert strategy.left.front_clearance_levels == 1
    assert strategy.right.front_clearance_levels == 1
    assert math.isclose(left.z, right.z, abs_tol=1e-9)


def test_mixed_modes_classify_unequal_after_top_target_conversion():
    strategy, left, right = resolve_dual_grasp_strategy(
        "front",
        "top_suction",
        Pose6DValue(0.75, 0.2, 0.8, math.pi, math.pi / 2.0, math.pi),
        Pose6DValue(0.9, -0.2, 1.4, math.pi, 0.0, math.pi),
    )
    assert strategy.task_type == MIXED_DEGRADED_TO_FRONT_RIGHT_HIGH
    assert strategy.left.front_clearance_levels == 2
    assert strategy.right.front_clearance_levels == 1
    assert right.z > left.z


def test_top_contact_converts_to_front_contact_using_box_geometry():
    top = Pose6DValue(0.9, 0.2, 1.4, math.pi, 0.0, math.pi)
    front = degrade_top_target_to_front(top)
    assert math.isclose(front.x, 0.75, abs_tol=1e-9)
    assert math.isclose(front.y, 0.2, abs_tol=1e-9)
    assert math.isclose(front.z, 1.2, abs_tol=1e-9)
