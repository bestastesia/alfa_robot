from __future__ import annotations

import math
from dataclasses import dataclass


DUAL_FRONT_EQUAL = 1
DUAL_FRONT = DUAL_FRONT_EQUAL
DUAL_TOP_EQUAL = 2
DUAL_TOP_LEFT_HIGH = 3
DUAL_TOP_RIGHT_HIGH = 4
MIXED_DEGRADED_TO_FRONT_EQUAL = 5
MIXED_DEGRADED_TO_FRONT = MIXED_DEGRADED_TO_FRONT_EQUAL
DUAL_FRONT_LEFT_HIGH = 6
DUAL_FRONT_RIGHT_HIGH = 7
MIXED_DEGRADED_TO_FRONT_LEFT_HIGH = 8
MIXED_DEGRADED_TO_FRONT_RIGHT_HIGH = 9

FRONT = "front"
TOP_SUCTION = "top_suction"
BOX_DEPTH_M = 0.3
BOX_WIDTH_M = 0.5
BOX_HEIGHT_M = 0.4
OUTER_BOX_GRASP_TARGET_Y_M = 0.50
OUTER_BOX_GRASP_LATERAL_OFFSET_M = BOX_WIDTH_M - OUTER_BOX_GRASP_TARGET_Y_M


@dataclass(frozen=True)
class Pose6DValue:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


@dataclass(frozen=True)
class ArmExtractPolicyValue:
    grasp_mode: str
    require_full_detachment: bool
    front_clearance_levels: int
    retreat_priority: float
    lift_priority: float
    pitch_priority: float


@dataclass(frozen=True)
class DualGraspStrategyValue:
    task_type: int
    name: str
    left: ArmExtractPolicyValue
    right: ArmExtractPolicyValue
    height_difference_m: float


def normalize_grasp_mode(value: str) -> str:
    normalized = str(value).strip().lower()
    aliases = {
        "front": FRONT,
        "side": FRONT,
        "side_suction": FRONT,
        "top": TOP_SUCTION,
        "top_suction": TOP_SUCTION,
        "down": TOP_SUCTION,
    }
    if normalized not in aliases:
        raise ValueError(f"unsupported grasp mode: {value}")
    return aliases[normalized]


def _front_policy(
    require_full_detachment: bool = True,
    front_clearance_levels: int = 1,
) -> ArmExtractPolicyValue:
    return ArmExtractPolicyValue(
        grasp_mode=FRONT,
        require_full_detachment=require_full_detachment,
        front_clearance_levels=max(1, int(front_clearance_levels)),
        retreat_priority=3.0,
        lift_priority=1.0,
        pitch_priority=1.0,
    )


def _top_policy(require_full_detachment: bool = True) -> ArmExtractPolicyValue:
    return ArmExtractPolicyValue(
        grasp_mode=TOP_SUCTION,
        require_full_detachment=require_full_detachment,
        front_clearance_levels=0,
        retreat_priority=1.0,
        lift_priority=3.0,
        pitch_priority=2.0,
    )


def classify_dual_grasp_strategy(
    left_mode: str,
    right_mode: str,
    left_z: float,
    right_z: float,
    equal_height_tolerance_m: float = 0.02,
) -> DualGraspStrategyValue:
    left_mode = normalize_grasp_mode(left_mode)
    right_mode = normalize_grasp_mode(right_mode)
    tolerance = max(0.0, float(equal_height_tolerance_m))
    height_difference = float(left_z) - float(right_z)

    mixed_degraded = left_mode != right_mode
    front_strategy = mixed_degraded or left_mode == FRONT
    if front_strategy:
        if abs(height_difference) <= tolerance:
            return DualGraspStrategyValue(
                task_type=(
                    MIXED_DEGRADED_TO_FRONT_EQUAL
                    if mixed_degraded else DUAL_FRONT_EQUAL
                ),
                name=(
                    "mixed_degraded_to_front_equal"
                    if mixed_degraded else "dual_front_equal"
                ),
                left=_front_policy(),
                right=_front_policy(),
                height_difference_m=height_difference,
            )
        if height_difference > 0.0:
            return DualGraspStrategyValue(
                task_type=(
                    MIXED_DEGRADED_TO_FRONT_LEFT_HIGH
                    if mixed_degraded else DUAL_FRONT_LEFT_HIGH
                ),
                name=(
                    "mixed_degraded_to_front_left_high"
                    if mixed_degraded else "dual_front_left_high"
                ),
                left=_front_policy(front_clearance_levels=1),
                right=_front_policy(front_clearance_levels=2),
                height_difference_m=height_difference,
            )
        return DualGraspStrategyValue(
            task_type=(
                MIXED_DEGRADED_TO_FRONT_RIGHT_HIGH
                if mixed_degraded else DUAL_FRONT_RIGHT_HIGH
            ),
            name=(
                "mixed_degraded_to_front_right_high"
                if mixed_degraded else "dual_front_right_high"
            ),
            left=_front_policy(front_clearance_levels=2),
            right=_front_policy(front_clearance_levels=1),
            height_difference_m=height_difference,
        )
    if abs(height_difference) <= tolerance:
        return DualGraspStrategyValue(
            task_type=DUAL_TOP_EQUAL,
            name="dual_top_equal",
            left=_top_policy(),
            right=_top_policy(),
            height_difference_m=height_difference,
        )
    if height_difference > 0.0:
        return DualGraspStrategyValue(
            task_type=DUAL_TOP_LEFT_HIGH,
            name="dual_top_left_high",
            left=_top_policy(require_full_detachment=False),
            right=_top_policy(require_full_detachment=True),
            height_difference_m=height_difference,
        )
    return DualGraspStrategyValue(
        task_type=DUAL_TOP_RIGHT_HIGH,
        name="dual_top_right_high",
        left=_top_policy(require_full_detachment=True),
        right=_top_policy(require_full_detachment=False),
        height_difference_m=height_difference,
    )


def pose6d_value(pose_6d) -> Pose6DValue:
    values = Pose6DValue(
        x=float(pose_6d.x),
        y=float(pose_6d.y),
        z=float(pose_6d.z),
        roll=float(pose_6d.roll),
        pitch=float(pose_6d.pitch),
        yaw=float(pose_6d.yaw),
    )
    if not all(math.isfinite(value) for value in values.__dict__.values()):
        raise ValueError("pose_6d contains a non-finite value")
    return values


def quaternion_xyzw(pose: Pose6DValue) -> tuple[float, float, float, float]:
    half_roll = 0.5 * pose.roll
    half_pitch = 0.5 * pose.pitch
    half_yaw = 0.5 * pose.yaw
    cr, sr = math.cos(half_roll), math.sin(half_roll)
    cp, sp = math.cos(half_pitch), math.sin(half_pitch)
    cy, sy = math.cos(half_yaw), math.sin(half_yaw)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def tool_z_axis(pose: Pose6DValue) -> tuple[float, float, float]:
    x, y, z, w = quaternion_xyzw(pose)
    return (
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    )


def degrade_top_target_to_front(
    pose: Pose6DValue,
    *,
    box_depth_m: float = BOX_DEPTH_M,
    box_height_m: float = BOX_HEIGHT_M,
) -> Pose6DValue:
    top_normal = tool_z_axis(pose)
    box_center = (
        pose.x + 0.5 * box_height_m * top_normal[0],
        pose.y + 0.5 * box_height_m * top_normal[1],
        pose.z + 0.5 * box_height_m * top_normal[2],
    )
    return Pose6DValue(
        x=box_center[0] - 0.5 * box_depth_m,
        y=box_center[1],
        z=box_center[2],
        roll=math.pi,
        pitch=math.pi / 2.0,
        yaw=math.pi,
    )


def resolve_dual_grasp_strategy(
    left_mode: str,
    right_mode: str,
    left_pose: Pose6DValue,
    right_pose: Pose6DValue,
    equal_height_tolerance_m: float = 0.02,
) -> tuple[DualGraspStrategyValue, Pose6DValue, Pose6DValue]:
    """Resolve effective front targets before comparing mixed-task heights."""
    left_mode = normalize_grasp_mode(left_mode)
    right_mode = normalize_grasp_mode(right_mode)
    mixed = left_mode != right_mode
    effective_left = (
        degrade_top_target_to_front(left_pose)
        if mixed and left_mode == TOP_SUCTION else left_pose
    )
    effective_right = (
        degrade_top_target_to_front(right_pose)
        if mixed and right_mode == TOP_SUCTION else right_pose
    )
    strategy = classify_dual_grasp_strategy(
        left_mode,
        right_mode,
        effective_left.z,
        effective_right.z,
        equal_height_tolerance_m,
    )
    return strategy, effective_left, effective_right
