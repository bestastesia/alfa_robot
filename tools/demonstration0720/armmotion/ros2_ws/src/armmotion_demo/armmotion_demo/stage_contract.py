from __future__ import annotations

import copy
import math
from dataclasses import dataclass

from geometry_msgs.msg import Pose
from robot_motion_interfaces.msg import DualArmPoseTargets
from robot_motion_runtime.dual_grasp_strategy import (
    FRONT_TOOL_RPY,
    Pose6DValue,
    TOP_TOOL_RPY,
    quaternion_xyzw,
    rpy_from_quaternion_xyzw,
)

from .common import PoseTaskSpec, planning_task_from_suction_surface_poses


@dataclass(frozen=True)
class ResolvedStageTargets:
    left_pose: Pose
    right_pose: Pose
    left_grasp_mode: int
    right_grasp_mode: int
    mirrored_from: str | None = None


def _mirrored_pose_y(source: Pose) -> Pose:
    mirrored = copy.deepcopy(source)
    mirrored.position.y = -float(source.position.y)
    return mirrored


def resolve_dual_stage_targets(request) -> ResolvedStageTargets:
    left_grasp_mode = int(request.targets.left_grasp_mode)
    right_grasp_mode = int(request.targets.right_grasp_mode)
    left_no_move = left_grasp_mode == DualArmPoseTargets.GRASP_MODE_NO_MOVE
    right_no_move = right_grasp_mode == DualArmPoseTargets.GRASP_MODE_NO_MOVE
    if left_no_move and right_no_move:
        raise ValueError("左右臂不能同时为 NO_MOVE")
    if left_no_move:
        return ResolvedStageTargets(
            left_pose=_mirrored_pose_y(request.targets.right_pose),
            right_pose=copy.deepcopy(request.targets.right_pose),
            left_grasp_mode=right_grasp_mode,
            right_grasp_mode=right_grasp_mode,
            mirrored_from="right",
        )
    if right_no_move:
        return ResolvedStageTargets(
            left_pose=copy.deepcopy(request.targets.left_pose),
            right_pose=_mirrored_pose_y(request.targets.left_pose),
            left_grasp_mode=left_grasp_mode,
            right_grasp_mode=left_grasp_mode,
            mirrored_from="left",
        )
    return ResolvedStageTargets(
        left_pose=copy.deepcopy(request.targets.left_pose),
        right_pose=copy.deepcopy(request.targets.right_pose),
        left_grasp_mode=left_grasp_mode,
        right_grasp_mode=right_grasp_mode,
    )


def grasp_mode_from_stage(value: int) -> str:
    if int(value) == DualArmPoseTargets.GRASP_MODE_SIDE_SUCTION:
        return "front"
    if int(value) == DualArmPoseTargets.GRASP_MODE_TOP_SUCTION:
        return "top_suction"
    raise ValueError(f"吸附目标不支持阶段类型: {value}")


def canonicalize_grasp_pose_orientation(message: Pose, stage: int) -> tuple[Pose, float]:
    grasp_mode = grasp_mode_from_stage(stage)
    canonical_rpy = FRONT_TOOL_RPY if grasp_mode == "front" else TOP_TOOL_RPY
    canonical = quaternion_xyzw(Pose6DValue(0.0, 0.0, 0.0, *canonical_rpy))
    source = (
        float(message.orientation.x),
        float(message.orientation.y),
        float(message.orientation.z),
        float(message.orientation.w),
    )
    source_norm = math.sqrt(sum(value * value for value in source))
    if source_norm <= 1e-8:
        raise ValueError("抓取目标四元数为零")
    source = tuple(value / source_norm for value in source)
    dot = min(1.0, max(-1.0, sum(a * b for a, b in zip(source, canonical))))
    angular_deviation = 2.0 * math.acos(abs(dot))
    corrected = copy.deepcopy(message)
    (
        corrected.orientation.x,
        corrected.orientation.y,
        corrected.orientation.z,
        corrected.orientation.w,
    ) = canonical
    return corrected, angular_deviation


def canonicalize_stage_target_orientations(request):
    corrected = copy.deepcopy(request)
    deviations: dict[str, float] = {}
    for side in ("left", "right"):
        grasp_mode = int(getattr(corrected.targets, f"{side}_grasp_mode"))
        if grasp_mode == DualArmPoseTargets.GRASP_MODE_NO_MOVE:
            continue
        pose = getattr(corrected.targets, f"{side}_pose")
        pose, deviation = canonicalize_grasp_pose_orientation(pose, grasp_mode)
        setattr(corrected.targets, f"{side}_pose", pose)
        deviations[side] = deviation
    return corrected, deviations


def align_target_pair_to_lower_height(
    targets: ResolvedStageTargets,
) -> ResolvedStageTargets:
    left_pose = copy.deepcopy(targets.left_pose)
    right_pose = copy.deepcopy(targets.right_pose)
    aligned_z = min(float(left_pose.position.z), float(right_pose.position.z))
    left_pose.position.z = aligned_z
    right_pose.position.z = aligned_z
    return ResolvedStageTargets(
        left_pose=left_pose,
        right_pose=right_pose,
        left_grasp_mode=targets.left_grasp_mode,
        right_grasp_mode=targets.right_grasp_mode,
        mirrored_from=targets.mirrored_from,
    )


def pose6d_from_pose(message: Pose, label: str) -> Pose6DValue:
    position = (message.position.x, message.position.y, message.position.z)
    quaternion = (
        message.orientation.x,
        message.orientation.y,
        message.orientation.z,
        message.orientation.w,
    )
    if not all(math.isfinite(float(value)) for value in (*position, *quaternion)):
        raise ValueError(f"{label} 包含非有限数值")
    norm = math.sqrt(sum(float(value) * float(value) for value in quaternion))
    if norm <= 1e-8:
        raise ValueError(f"{label} 四元数为零")
    normalized = tuple(float(value) / norm for value in quaternion)
    roll, pitch, yaw = rpy_from_quaternion_xyzw(normalized)
    return Pose6DValue(
        x=float(position[0]),
        y=float(position[1]),
        z=float(position[2]),
        roll=roll,
        pitch=pitch,
        yaw=yaw,
    )


def planning_task_from_stage_goal(request, task_code: str) -> PoseTaskSpec:
    return planning_task_from_resolved_targets(
        resolve_dual_stage_targets(request),
        task_code,
    )


def planning_task_from_resolved_targets(
    targets: ResolvedStageTargets,
    task_code: str,
) -> PoseTaskSpec:
    left = pose6d_from_pose(
        targets.left_pose,
        "targets.left_pose",
    )
    right = pose6d_from_pose(
        targets.right_pose,
        "targets.right_pose",
    )
    return planning_task_from_suction_surface_poses(
        task_code,
        left,
        right,
        grasp_mode_from_stage(targets.left_grasp_mode),
        grasp_mode_from_stage(targets.right_grasp_mode),
    )


def validate_stage_pose_targets(request) -> None:
    valid_modes = {
        DualArmPoseTargets.GRASP_MODE_TOP_SUCTION,
        DualArmPoseTargets.GRASP_MODE_SIDE_SUCTION,
        DualArmPoseTargets.GRASP_MODE_NO_MOVE,
    }
    for name, mode, pose in (
        (
            "targets.left",
            request.targets.left_grasp_mode,
            request.targets.left_pose,
        ),
        (
            "targets.right",
            request.targets.right_grasp_mode,
            request.targets.right_pose,
        ),
    ):
        if int(mode) not in valid_modes:
            raise ValueError(f"{name}_grasp_mode 不支持: {mode}")
        if int(mode) != DualArmPoseTargets.GRASP_MODE_NO_MOVE:
            pose6d_from_pose(pose, f"{name}_pose")
    resolve_dual_stage_targets(request)
