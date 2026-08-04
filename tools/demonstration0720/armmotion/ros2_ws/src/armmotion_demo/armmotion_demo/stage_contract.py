from __future__ import annotations

import math

from alfa_motion_interfaces.msg import MotionPoseTarget
from geometry_msgs.msg import PoseStamped
from robot_motion_runtime.dual_grasp_strategy import (
    Pose6DValue,
    rpy_from_quaternion_xyzw,
)

from .common import PoseTaskSpec, planning_task_from_front_face_poses


def pose6d_from_pose_stamped(message: PoseStamped, label: str) -> Pose6DValue:
    if message.header.frame_id != "base_link":
        raise ValueError(
            f"{label}.header.frame_id 必须是 base_link，当前 {message.header.frame_id!r}"
        )
    pose = message.pose
    position = (pose.position.x, pose.position.y, pose.position.z)
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
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


def planning_task_from_stage_goal(request) -> PoseTaskSpec:
    task_id = str(request.context.task_id).strip()
    request_id = str(request.context.request_id).strip()
    if not request_id or not task_id:
        raise ValueError("context.request_id/task_id 不能为空")
    if int(request.context.sequence_id) <= 0:
        raise ValueError("context.sequence_id 必须从1开始")
    left = pose6d_from_pose_stamped(
        request.left_target.pose,
        "left_target.pose",
    )
    right = pose6d_from_pose_stamped(
        request.right_target.pose,
        "right_target.pose",
    )
    return planning_task_from_front_face_poses(task_id, left, right)


def validate_stage_pose_targets(request) -> None:
    valid_modes = {
        MotionPoseTarget.NO_MOVE,
        MotionPoseTarget.SIDE_SUCTION,
        MotionPoseTarget.TOP_SUCTION,
    }
    for name, target in (
        ("left_target", request.left_target),
        ("right_target", request.right_target),
    ):
        if int(target.grasp_mode) not in valid_modes:
            raise ValueError(f"{name}.grasp_mode 不支持: {target.grasp_mode}")
        pose6d_from_pose_stamped(target.pose, f"{name}.pose")
