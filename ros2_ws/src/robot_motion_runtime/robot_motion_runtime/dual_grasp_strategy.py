from __future__ import annotations

import math
from dataclasses import dataclass, replace


DUAL_FRONT_EQUAL = 1
DUAL_FRONT = DUAL_FRONT_EQUAL
DUAL_TOP_EQUAL = 2
DUAL_TOP_LEFT_HIGH = 3
DUAL_TOP_RIGHT_HIGH = 4
MIXED_EQUAL = 5
MIXED_DEGRADED_TO_FRONT_EQUAL = MIXED_EQUAL
MIXED_DEGRADED_TO_FRONT = MIXED_EQUAL
DUAL_FRONT_LEFT_HIGH = 6
DUAL_FRONT_RIGHT_HIGH = 7
MIXED_LEFT_HIGH = 8
MIXED_RIGHT_HIGH = 9
MIXED_DEGRADED_TO_FRONT_LEFT_HIGH = MIXED_LEFT_HIGH
MIXED_DEGRADED_TO_FRONT_RIGHT_HIGH = MIXED_RIGHT_HIGH

FRONT = "front"
TOP_SUCTION = "top_suction"
BOX_DEPTH_M = 0.3
BOX_WIDTH_M = 0.4
BOX_HEIGHT_M = 0.4
BOX_ROW_GAP_M = 0.01
BOX_ROW_PITCH_M = BOX_HEIGHT_M + BOX_ROW_GAP_M
BOX_ROW_COUNT = 5
TOP_SUCTION_FIRST_ROW = 3
WORLD_TO_BASE_Z_M = 0.202094
BOTTOM_ROW_CENTER_WORLD_Z_M = 0.21
BOTTOM_ROW_FRONT_CENTER_Z_M = BOTTOM_ROW_CENTER_WORLD_Z_M - WORLD_TO_BASE_Z_M
ROW_MATCH_TOLERANCE_M = 0.12
OUTER_BOX_GRASP_TARGET_Y_M = 0.40
OUTER_BOX_GRASP_LATERAL_OFFSET_M = BOX_WIDTH_M - OUTER_BOX_GRASP_TARGET_Y_M
FRONT_TOOL_RPY = (math.pi, -math.pi / 2.0, 0.0)
TOP_TOOL_RPY = (math.pi, 0.0, 0.0)


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


@dataclass(frozen=True)
class BoxRowMatch:
    row_from_top: int
    center_z_m: float
    residual_m: float


@dataclass(frozen=True)
class FrontFaceTaskResolution:
    strategy: DualGraspStrategyValue
    left_tool_pose: Pose6DValue
    right_tool_pose: Pose6DValue
    left_row: BoxRowMatch
    right_row: BoxRowMatch


@dataclass(frozen=True)
class SuctionSurfaceTaskResolution:
    strategy: DualGraspStrategyValue
    left_tool_pose: Pose6DValue
    right_tool_pose: Pose6DValue
    left_box_center_pose: Pose6DValue
    right_box_center_pose: Pose6DValue
    left_front_face_pose: Pose6DValue
    right_front_face_pose: Pose6DValue
    left_row: BoxRowMatch
    right_row: BoxRowMatch


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


def promote_mixed_grasp_modes_to_top(
    left_mode: str,
    right_mode: str,
) -> tuple[str, str]:
    left_mode = normalize_grasp_mode(left_mode)
    right_mode = normalize_grasp_mode(right_mode)
    if left_mode != right_mode:
        return TOP_SUCTION, TOP_SUCTION
    return left_mode, right_mode


def _front_policy(
    require_full_detachment: bool = True,
    front_clearance_levels: int = 1,
    *,
    lift_first: bool = False,
) -> ArmExtractPolicyValue:
    return ArmExtractPolicyValue(
        grasp_mode=FRONT,
        require_full_detachment=require_full_detachment,
        front_clearance_levels=max(1, int(front_clearance_levels)),
        retreat_priority=1.0 if lift_first else 3.0,
        lift_priority=3.0 if lift_first else 1.0,
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


def _policy_for_mode(
    mode: str,
    *,
    lower_side: bool = False,
) -> ArmExtractPolicyValue:
    if mode == TOP_SUCTION:
        return _top_policy(require_full_detachment=True)
    return _front_policy(
        front_clearance_levels=2 if lower_side else 1,
        lift_first=lower_side,
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

    mixed = left_mode != right_mode
    if mixed:
        if abs(height_difference) <= tolerance:
            return DualGraspStrategyValue(
                task_type=MIXED_EQUAL,
                name="mixed_equal",
                left=_policy_for_mode(left_mode),
                right=_policy_for_mode(right_mode),
                height_difference_m=height_difference,
            )
        if height_difference > 0.0:
            return DualGraspStrategyValue(
                task_type=MIXED_LEFT_HIGH,
                name="mixed_left_high",
                left=_policy_for_mode(left_mode),
                right=_policy_for_mode(right_mode, lower_side=True),
                height_difference_m=height_difference,
            )
        return DualGraspStrategyValue(
            task_type=MIXED_RIGHT_HIGH,
            name="mixed_right_high",
            left=_policy_for_mode(left_mode, lower_side=True),
            right=_policy_for_mode(right_mode),
            height_difference_m=height_difference,
        )

    front_strategy = left_mode == FRONT
    if front_strategy:
        if abs(height_difference) <= tolerance:
            return DualGraspStrategyValue(
                task_type=DUAL_FRONT_EQUAL,
                name="dual_front_equal",
                left=_front_policy(),
                right=_front_policy(),
                height_difference_m=height_difference,
            )
        if height_difference > 0.0:
            return DualGraspStrategyValue(
                task_type=DUAL_FRONT_LEFT_HIGH,
                name="dual_front_left_high",
                left=_front_policy(front_clearance_levels=1),
                right=_front_policy(front_clearance_levels=2, lift_first=True),
                height_difference_m=height_difference,
            )
        return DualGraspStrategyValue(
            task_type=DUAL_FRONT_RIGHT_HIGH,
            name="dual_front_right_high",
            left=_front_policy(front_clearance_levels=2, lift_first=True),
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


def normalize_quaternion_xyzw(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm <= 1e-12:
        raise ValueError("quaternion norm is zero")
    return tuple(value / norm for value in quaternion)


def quaternion_multiply_xyzw(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    left_x, left_y, left_z, left_w = left
    right_x, right_y, right_z, right_w = right
    return (
        left_w * right_x + left_x * right_w + left_y * right_z - left_z * right_y,
        left_w * right_y - left_x * right_z + left_y * right_w + left_z * right_x,
        left_w * right_z + left_x * right_y - left_y * right_x + left_z * right_w,
        left_w * right_w - left_x * right_x - left_y * right_y - left_z * right_z,
    )


def rotate_vector_by_quaternion(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    quaternion = normalize_quaternion_xyzw(quaternion)
    conjugate = (-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3])
    rotated = quaternion_multiply_xyzw(
        quaternion_multiply_xyzw(quaternion, (*vector, 0.0)),
        conjugate,
    )
    return rotated[0], rotated[1], rotated[2]


def rpy_from_quaternion_xyzw(
    quaternion: tuple[float, float, float, float],
) -> tuple[float, float, float]:
    x, y, z, w = normalize_quaternion_xyzw(quaternion)
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    if abs(sin_pitch) >= 1.0 - 1e-10:
        pitch = math.copysign(math.pi / 2.0, sin_pitch)
        roll = 2.0 * math.atan2(x, w)
        roll = math.atan2(math.sin(roll), math.cos(roll))
        return roll, pitch, 0.0
    sin_roll_cos_pitch = 2.0 * (w * x + y * z)
    cos_roll_cos_pitch = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll_cos_pitch, cos_roll_cos_pitch)
    pitch = math.asin(sin_pitch)
    sin_yaw_cos_pitch = 2.0 * (w * z + x * y)
    cos_yaw_cos_pitch = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch)
    return roll, pitch, yaw


def tool_z_axis(pose: Pose6DValue) -> tuple[float, float, float]:
    x, y, z, w = quaternion_xyzw(pose)
    return (
        2.0 * (x * z + w * y),
        2.0 * (y * z - w * x),
        1.0 - 2.0 * (x * x + y * y),
    )


def match_box_row(
    front_face_z_m: float,
    *,
    row_count: int = BOX_ROW_COUNT,
    row_pitch_m: float = BOX_ROW_PITCH_M,
    bottom_row_center_z_m: float = BOTTOM_ROW_FRONT_CENTER_Z_M,
    tolerance_m: float = ROW_MATCH_TOLERANCE_M,
) -> BoxRowMatch:
    if row_count <= 0:
        raise ValueError("row_count must be positive")
    if not math.isfinite(row_pitch_m) or row_pitch_m <= 0.0:
        raise ValueError("row_pitch_m must be finite and positive")
    if not math.isfinite(front_face_z_m):
        raise ValueError("front_face_z_m must be finite")
    tolerance_m = float(tolerance_m)
    if tolerance_m < 0.0 or tolerance_m >= 0.5 * row_pitch_m:
        raise ValueError("row tolerance must be non-negative and smaller than half a row")
    matches = [
        BoxRowMatch(
            row_from_top=row,
            center_z_m=float(bottom_row_center_z_m) + float(row_count - row) * row_pitch_m,
            residual_m=0.0,
        )
        for row in range(1, row_count + 1)
    ]
    nearest = min(matches, key=lambda match: abs(front_face_z_m - match.center_z_m))
    nearest = replace(nearest, residual_m=front_face_z_m - nearest.center_z_m)
    if abs(nearest.residual_m) > tolerance_m:
        raise ValueError(
            f"front-face z={front_face_z_m:.4f}m does not match a box row; "
            f"nearest=row{nearest.row_from_top} center={nearest.center_z_m:.4f}m "
            f"residual={nearest.residual_m:+.4f}m tolerance={tolerance_m:.4f}m"
        )
    return nearest


def grasp_mode_for_row(
    row_from_top: int,
    *,
    top_suction_first_row: int = TOP_SUCTION_FIRST_ROW,
) -> str:
    if row_from_top <= 0:
        raise ValueError("row_from_top must be positive")
    return TOP_SUCTION if row_from_top >= top_suction_first_row else FRONT


def front_face_to_tool_contact(
    front_face_pose: Pose6DValue,
    grasp_mode: str,
    *,
    box_depth_m: float = BOX_DEPTH_M,
    box_height_m: float = BOX_HEIGHT_M,
) -> Pose6DValue:
    grasp_mode = normalize_grasp_mode(grasp_mode)
    if grasp_mode == FRONT:
        return front_face_pose

    front_quaternion = normalize_quaternion_xyzw(quaternion_xyzw(front_face_pose))
    nominal_front = Pose6DValue(0.0, 0.0, 0.0, *FRONT_TOOL_RPY)
    nominal_top = Pose6DValue(0.0, 0.0, 0.0, *TOP_TOOL_RPY)
    nominal_front_quaternion = normalize_quaternion_xyzw(quaternion_xyzw(nominal_front))
    nominal_top_quaternion = normalize_quaternion_xyzw(quaternion_xyzw(nominal_top))
    nominal_front_inverse = (
        -nominal_front_quaternion[0],
        -nominal_front_quaternion[1],
        -nominal_front_quaternion[2],
        nominal_front_quaternion[3],
    )
    box_rotation = normalize_quaternion_xyzw(
        quaternion_multiply_xyzw(front_quaternion, nominal_front_inverse)
    )
    inward = rotate_vector_by_quaternion(front_quaternion, (0.0, 0.0, 1.0))
    upward = rotate_vector_by_quaternion(box_rotation, (0.0, 0.0, 1.0))
    top_quaternion = normalize_quaternion_xyzw(
        quaternion_multiply_xyzw(box_rotation, nominal_top_quaternion)
    )
    top_rpy = rpy_from_quaternion_xyzw(top_quaternion)
    return Pose6DValue(
        x=front_face_pose.x + 0.5 * box_depth_m * inward[0] + 0.5 * box_height_m * upward[0],
        y=front_face_pose.y + 0.5 * box_depth_m * inward[1] + 0.5 * box_height_m * upward[1],
        z=front_face_pose.z + 0.5 * box_depth_m * inward[2] + 0.5 * box_height_m * upward[2],
        roll=top_rpy[0],
        pitch=top_rpy[1],
        yaw=top_rpy[2],
    )


def suction_surface_to_box_geometry(
    suction_surface_pose: Pose6DValue,
    grasp_mode: str,
    *,
    box_depth_m: float = BOX_DEPTH_M,
    box_height_m: float = BOX_HEIGHT_M,
) -> tuple[Pose6DValue, Pose6DValue]:
    """Return box-center and front-face-center poses from an actual suction pose."""
    grasp_mode = normalize_grasp_mode(grasp_mode)
    quaternion = normalize_quaternion_xyzw(quaternion_xyzw(suction_surface_pose))
    if grasp_mode == FRONT:
        inward = rotate_vector_by_quaternion(quaternion, (0.0, 0.0, 1.0))
        box_center = (
            suction_surface_pose.x + 0.5 * box_depth_m * inward[0],
            suction_surface_pose.y + 0.5 * box_depth_m * inward[1],
            suction_surface_pose.z + 0.5 * box_depth_m * inward[2],
        )
    else:
        downward = rotate_vector_by_quaternion(quaternion, (0.0, 0.0, 1.0))
        inward = rotate_vector_by_quaternion(quaternion, (1.0, 0.0, 0.0))
        box_center = (
            suction_surface_pose.x + 0.5 * box_height_m * downward[0],
            suction_surface_pose.y + 0.5 * box_height_m * downward[1],
            suction_surface_pose.z + 0.5 * box_height_m * downward[2],
        )
    front_center = (
        box_center[0] - 0.5 * box_depth_m * inward[0],
        box_center[1] - 0.5 * box_depth_m * inward[1],
        box_center[2] - 0.5 * box_depth_m * inward[2],
    )
    return (
        Pose6DValue(*box_center, *FRONT_TOOL_RPY),
        Pose6DValue(*front_center, *FRONT_TOOL_RPY),
    )


def resolve_suction_surface_dual_grasp_strategy(
    left_suction_surface_pose: Pose6DValue,
    right_suction_surface_pose: Pose6DValue,
    left_grasp_mode: str,
    right_grasp_mode: str,
    *,
    row_count: int = BOX_ROW_COUNT,
    row_pitch_m: float = BOX_ROW_PITCH_M,
    box_height_m: float = BOX_HEIGHT_M,
    box_depth_m: float = BOX_DEPTH_M,
    bottom_row_center_z_m: float = BOTTOM_ROW_FRONT_CENTER_Z_M,
    row_match_tolerance_m: float = ROW_MATCH_TOLERANCE_M,
) -> SuctionSurfaceTaskResolution:
    left_grasp_mode = normalize_grasp_mode(left_grasp_mode)
    right_grasp_mode = normalize_grasp_mode(right_grasp_mode)
    left_center, left_front = suction_surface_to_box_geometry(
        left_suction_surface_pose,
        left_grasp_mode,
        box_depth_m=box_depth_m,
        box_height_m=box_height_m,
    )
    right_center, right_front = suction_surface_to_box_geometry(
        right_suction_surface_pose,
        right_grasp_mode,
        box_depth_m=box_depth_m,
        box_height_m=box_height_m,
    )
    left_row = match_box_row(
        left_center.z,
        row_count=row_count,
        row_pitch_m=row_pitch_m,
        bottom_row_center_z_m=bottom_row_center_z_m,
        tolerance_m=row_match_tolerance_m,
    )
    right_row = match_box_row(
        right_center.z,
        row_count=row_count,
        row_pitch_m=row_pitch_m,
        bottom_row_center_z_m=bottom_row_center_z_m,
        tolerance_m=row_match_tolerance_m,
    )
    strategy, left_tool_pose, right_tool_pose = resolve_dual_grasp_strategy(
        left_grasp_mode,
        right_grasp_mode,
        left_suction_surface_pose,
        right_suction_surface_pose,
        equal_height_tolerance_m=0.5 * row_pitch_m,
    )
    return SuctionSurfaceTaskResolution(
        strategy=replace(
            strategy,
            height_difference_m=left_center.z - right_center.z,
        ),
        left_tool_pose=left_tool_pose,
        right_tool_pose=right_tool_pose,
        left_box_center_pose=left_center,
        right_box_center_pose=right_center,
        left_front_face_pose=left_front,
        right_front_face_pose=right_front,
        left_row=left_row,
        right_row=right_row,
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
        roll=FRONT_TOOL_RPY[0],
        pitch=FRONT_TOOL_RPY[1],
        yaw=FRONT_TOOL_RPY[2],
    )


def promote_front_target_to_top(
    pose: Pose6DValue,
    *,
    box_depth_m: float = BOX_DEPTH_M,
    box_height_m: float = BOX_HEIGHT_M,
) -> Pose6DValue:
    return front_face_to_tool_contact(
        pose,
        TOP_SUCTION,
        box_depth_m=box_depth_m,
        box_height_m=box_height_m,
    )


def resolve_front_face_dual_grasp_strategy(
    left_front_face_pose: Pose6DValue,
    right_front_face_pose: Pose6DValue,
    *,
    row_count: int = BOX_ROW_COUNT,
    row_pitch_m: float = BOX_ROW_PITCH_M,
    box_height_m: float = BOX_HEIGHT_M,
    box_depth_m: float = BOX_DEPTH_M,
    bottom_row_center_z_m: float = BOTTOM_ROW_FRONT_CENTER_Z_M,
    row_match_tolerance_m: float = ROW_MATCH_TOLERANCE_M,
    top_suction_first_row: int = TOP_SUCTION_FIRST_ROW,
) -> FrontFaceTaskResolution:
    left_row = match_box_row(
        left_front_face_pose.z,
        row_count=row_count,
        row_pitch_m=row_pitch_m,
        bottom_row_center_z_m=bottom_row_center_z_m,
        tolerance_m=row_match_tolerance_m,
    )
    right_row = match_box_row(
        right_front_face_pose.z,
        row_count=row_count,
        row_pitch_m=row_pitch_m,
        bottom_row_center_z_m=bottom_row_center_z_m,
        tolerance_m=row_match_tolerance_m,
    )
    left_mode = grasp_mode_for_row(
        left_row.row_from_top,
        top_suction_first_row=top_suction_first_row,
    )
    right_mode = grasp_mode_for_row(
        right_row.row_from_top,
        top_suction_first_row=top_suction_first_row,
    )
    left_mode, right_mode = promote_mixed_grasp_modes_to_top(left_mode, right_mode)
    left_tool_pose = front_face_to_tool_contact(
        left_front_face_pose,
        left_mode,
        box_depth_m=box_depth_m,
        box_height_m=box_height_m,
    )
    right_tool_pose = front_face_to_tool_contact(
        right_front_face_pose,
        right_mode,
        box_depth_m=box_depth_m,
        box_height_m=box_height_m,
    )
    strategy = classify_dual_grasp_strategy(
        left_mode,
        right_mode,
        left_row.center_z_m,
        right_row.center_z_m,
        equal_height_tolerance_m=0.5 * box_height_m,
    )
    strategy = replace(
        strategy,
        height_difference_m=left_front_face_pose.z - right_front_face_pose.z,
    )
    return FrontFaceTaskResolution(
        strategy=strategy,
        left_tool_pose=left_tool_pose,
        right_tool_pose=right_tool_pose,
        left_row=left_row,
        right_row=right_row,
    )


def resolve_dual_grasp_strategy(
    left_mode: str,
    right_mode: str,
    left_pose: Pose6DValue,
    right_pose: Pose6DValue,
    equal_height_tolerance_m: float = 0.02,
) -> tuple[DualGraspStrategyValue, Pose6DValue, Pose6DValue]:
    """Promote mixed front/top tasks to dual top suction before classification."""
    left_mode = normalize_grasp_mode(left_mode)
    right_mode = normalize_grasp_mode(right_mode)
    mixed = left_mode != right_mode
    effective_left = (
        promote_front_target_to_top(left_pose)
        if mixed and left_mode == FRONT else left_pose
    )
    effective_right = (
        promote_front_target_to_top(right_pose)
        if mixed and right_mode == FRONT else right_pose
    )
    effective_left_mode, effective_right_mode = promote_mixed_grasp_modes_to_top(
        left_mode, right_mode
    )
    strategy = classify_dual_grasp_strategy(
        effective_left_mode,
        effective_right_mode,
        effective_left.z,
        effective_right.z,
        equal_height_tolerance_m,
    )
    return strategy, effective_left, effective_right
