from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

from alfa_task_interfaces.msg import PickTarget


SUPPORTED_GRASP_MODES = frozenset(("front", "top_suction"))


def _finite(values) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def _pose_dict(pose_stamped) -> dict[str, Any]:
    pose = pose_stamped.pose.pose
    position = [pose.position.x, pose.position.y, pose.position.z]
    orientation = [
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ]
    if not _finite((*position, *orientation)):
        raise ValueError("吸附位姿包含非有限数值")
    quaternion_norm = math.sqrt(sum(value * value for value in orientation))
    if quaternion_norm <= 1e-8:
        raise ValueError("吸附位姿四元数为零")
    orientation = [value / quaternion_norm for value in orientation]
    return {
        "frame_id": pose_stamped.header.frame_id,
        "position": position,
        "orientation": orientation,
    }


def _normalise_mode(value: str) -> str:
    mode = str(value).strip().lower()
    if mode not in SUPPORTED_GRASP_MODES:
        raise ValueError(f"不支持的吸附方式: {value!r}")
    return mode


def _validate_target(target: PickTarget, expected_arm: int, label: str) -> None:
    if int(target.arm_id) != int(expected_arm):
        raise ValueError(f"{label}.arm_id 与目标顺序不一致")
    if int(target.box_id) <= 0:
        raise ValueError(f"{label}.box_id 必须为正数")
    if int(target.row) <= 0 or int(target.column) <= 0:
        raise ValueError(f"{label}.row/column 必须从1开始")
    mode = _normalise_mode(target.suction_mode)
    body_pose = target.refined_geometry.body_pose
    if body_pose.header.frame_id != "base_link":
        raise ValueError(
            f"{label}.refined_geometry.body_pose 必须在 base_link，"
            f"当前 {body_pose.header.frame_id!r}"
        )
    _pose_dict(body_pose)
    pose = target.refined_geometry.suction_surface_pose
    if pose.header.frame_id != "base_link":
        raise ValueError(
            f"{label}.refined_geometry.suction_surface_pose 必须在 base_link，"
            f"当前 {pose.header.frame_id!r}"
        )
    _pose_dict(pose)
    size = target.refined_geometry.size_m
    if not _finite((size.x, size.y, size.z)) or min(size.x, size.y, size.z) <= 0.0:
        raise ValueError(f"{label}.refined_geometry.size_m 必须是有限正数")
    if not all(target.refined_geometry.size_valid):
        raise ValueError(f"{label}.refined_geometry.size_valid 必须全部有效")
    if mode == "top_suction" and pose.pose.pose.position.z <= 0.0:
        raise ValueError(f"{label} 顶吸目标 z 必须为正数")


@dataclass(frozen=True)
class DomainTaskSpec:
    code: str
    index: int
    task_id: str
    sequence_id: int
    left_box_id: int
    right_box_id: int
    left_grasp_mode: str
    right_grasp_mode: str
    explicit_targets: dict[str, dict[str, Any]]
    scene_y_shift: float
    effective_distance_m: float
    front_distance_m: float
    top_distance_m: float
    extract_box_pose_rrt_max_iterations: int = 240

    @property
    def layout(self) -> str:
        return "B"

    @property
    def task_layout(self) -> str:
        return "centered"

    @property
    def grasp_family(self) -> str:
        if self.left_grasp_mode == self.right_grasp_mode:
            return self.left_grasp_mode
        return "mixed"

    @property
    def extraction_mode(self) -> str:
        if "top_suction" in (self.left_grasp_mode, self.right_grasp_mode):
            return "direct_updown_lift"
        return "box_pose_rrt"


def task_spec_from_pick_targets(
    request_id: str,
    task_id: str,
    sequence_id: int,
    targets: list[PickTarget],
) -> DomainTaskSpec:
    if len(targets) != 2:
        raise ValueError("首版 Motion 容器只接受恰好两个抓取目标")
    by_arm = {int(target.arm_id): target for target in targets}
    if set(by_arm) != {PickTarget.ARM_LEFT, PickTarget.ARM_RIGHT}:
        raise ValueError("双箱请求必须恰好包含 LEFT 和 RIGHT")
    if [int(target.arm_id) for target in targets] != [
        PickTarget.ARM_LEFT,
        PickTarget.ARM_RIGHT,
    ]:
        raise ValueError("双箱目标顺序必须是 LEFT、RIGHT")
    left = by_arm[PickTarget.ARM_LEFT]
    right = by_arm[PickTarget.ARM_RIGHT]
    _validate_target(left, PickTarget.ARM_LEFT, "left")
    _validate_target(right, PickTarget.ARM_RIGHT, "right")

    left_mode = _normalise_mode(left.suction_mode)
    right_mode = _normalise_mode(right.suction_mode)
    left_pose = _pose_dict(left.refined_geometry.suction_surface_pose)
    right_pose = _pose_dict(right.refined_geometry.suction_surface_pose)
    left_body = left.refined_geometry.body_pose.pose.pose.position
    right_body = right.refined_geometry.body_pose.pose.pose.position
    body_values = (left_body.x, left_body.y, left_body.z, right_body.x, right_body.y, right_body.z)
    if not _finite(body_values):
        raise ValueError("箱体中心位姿包含非有限数值")
    box_front_x = 0.5 * (float(left_body.x) + float(right_body.x))
    if box_front_x <= 0.0:
        raise ValueError(f"箱体中心 x 必须为正数，当前 {box_front_x:.4f}")
    scene_y_shift = 0.5 * (float(left_body.y) + float(right_body.y))
    row_index = max(int(left.row), int(right.row))
    safe_request = "".join(
        character if character.isalnum() or character == "_" else "_"
        for character in str(request_id)
    ).strip("_") or "request"
    return DomainTaskSpec(
        code=f"M02_{sequence_id}_{safe_request}",
        index=max(1, row_index),
        task_id=str(task_id),
        sequence_id=int(sequence_id),
        left_box_id=int(left.box_id),
        right_box_id=int(right.box_id),
        left_grasp_mode=left_mode,
        right_grasp_mode=right_mode,
        explicit_targets={"left": left_pose, "right": right_pose},
        scene_y_shift=scene_y_shift,
        effective_distance_m=box_front_x,
        front_distance_m=box_front_x,
        top_distance_m=box_front_x,
        extract_box_pose_rrt_max_iterations=400 if row_index >= 5 else 240,
    )
