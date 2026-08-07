from __future__ import annotations

import math

from alfa_motion_interfaces.msg import DualArmPoseTargets
from geometry_msgs.msg import Pose
from robot_motion_runtime.dual_grasp_strategy import (
    Pose6DValue,
    rpy_from_quaternion_xyzw,
)

from .common import PoseTaskSpec, planning_task_from_front_face_poses


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
    if int(request.targets.left_stage) == DualArmPoseTargets.STAGE_NO_MOVE:
        raise ValueError("当前双臂抓取流程暂不支持左臂 NO_MOVE")
    if int(request.targets.right_stage) == DualArmPoseTargets.STAGE_NO_MOVE:
        raise ValueError("当前双臂抓取流程暂不支持右臂 NO_MOVE")
    left = pose6d_from_pose(
        request.targets.left_pose,
        "targets.left_pose",
    )
    right = pose6d_from_pose(
        request.targets.right_pose,
        "targets.right_pose",
    )
    return planning_task_from_front_face_poses(task_code, left, right)


def validate_stage_pose_targets(request) -> None:
    valid_modes = {
        DualArmPoseTargets.STAGE_TOP_SUCTION,
        DualArmPoseTargets.STAGE_SIDE_SUCTION,
        DualArmPoseTargets.STAGE_NO_MOVE,
    }
    for name, mode, pose in (
        ("targets.left", request.targets.left_stage, request.targets.left_pose),
        ("targets.right", request.targets.right_stage, request.targets.right_pose),
    ):
        if int(mode) not in valid_modes:
            raise ValueError(f"{name}_stage 不支持: {mode}")
        if int(mode) != DualArmPoseTargets.STAGE_NO_MOVE:
            pose6d_from_pose(pose, f"{name}_pose")
