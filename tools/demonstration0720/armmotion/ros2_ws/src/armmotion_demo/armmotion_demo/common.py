from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable

from robot_motion_runtime.dual_grasp_strategy import (
    BOTTOM_ROW_FRONT_CENTER_Z_M,
    BOX_DEPTH_M,
    BOX_HEIGHT_M,
    BOX_ROW_COUNT,
    FRONT_TOOL_RPY,
    OUTER_BOX_GRASP_TARGET_Y_M,
    Pose6DValue,
    ROW_MATCH_TOLERANCE_M,
    TOP_SUCTION_FIRST_ROW,
    quaternion_xyzw,
    resolve_front_face_dual_grasp_strategy,
)


TASK_PAIRS = {
    1: (1, 3),
    2: (4, 6),
    3: (7, 9),
    4: (10, 12),
    5: (13, 15),
}
TASK_LAYOUTS = {
    "A": "right_shift_0p1",
    "B": "centered",
}
FRONT_TASKS = frozenset({1, 2, 3})
DIRECT_LIFT_TASKS = frozenset({3, 4, 5})
STAGE_LABELS = {
    1: "负重位到 IK 前 5cm 预吸附位",
    2: "笛卡尔前进 5cm 并打开双侧电磁阀和真空泵",
    3: "吸附后抽离并回负重姿态（updown 仅在高于0.45m时降至0.45m）",
    4: "双臂与 updown 同步运动到放置位",
    5: "关闭双侧电磁阀和真空泵",
    6: "双臂与 updown 同步回负重位",
}
STAGE_COUNT = len(STAGE_LABELS)
GRASP_ENABLE_STAGE = 2
GRASP_DISABLE_STAGE = 5
FINAL_STAGE = STAGE_COUNT
LOADED_ARM_POSE_DEG = (0.0, -45.0, 120.0, -75.0, 0.0, 0.0)


@dataclass(frozen=True)
class TaskSpec:
    code: str
    layout: str
    index: int
    left_box_id: int
    right_box_id: int
    front_distance_m: float
    top_distance_m: float

    @property
    def task_layout(self) -> str:
        return TASK_LAYOUTS[self.layout]

    @property
    def grasp_family(self) -> str:
        return "front" if self.index in FRONT_TASKS else "top_suction"

    @property
    def extraction_mode(self) -> str:
        return "direct_updown_lift" if self.index in DIRECT_LIFT_TASKS else "box_pose_rrt"

    @property
    def uses_direct_updown_lift(self) -> bool:
        return self.index in DIRECT_LIFT_TASKS

    @property
    def left_grasp_mode(self) -> str:
        return self.grasp_family

    @property
    def right_grasp_mode(self) -> str:
        return self.grasp_family

    @property
    def explicit_targets(self) -> None:
        return None

    @property
    def scene_y_shift(self) -> None:
        return None

    @property
    def extract_box_pose_rrt_max_iterations(self) -> int:
        return 400 if self.index == 5 else 160

    @property
    def effective_distance_m(self) -> float:
        return self.front_distance_m if self.index in FRONT_TASKS else self.top_distance_m


@dataclass(frozen=True)
class PoseTaskSpec:
    code: str
    left_front_face_pose: Pose6DValue
    right_front_face_pose: Pose6DValue
    left_row: int
    right_row: int
    left_row_residual_m: float
    right_row_residual_m: float
    left_grasp_mode: str
    right_grasp_mode: str
    left_tool_pose: Pose6DValue
    right_tool_pose: Pose6DValue
    strategy: Any
    scene_y_shift: float
    effective_distance_m: float

    @property
    def task_layout(self) -> str:
        return "pose_driven"

    @property
    def layout(self) -> str:
        return "pose_driven"

    @property
    def index(self) -> int:
        return 0

    @property
    def left_scene_slot_id(self) -> int:
        return 1

    @property
    def right_scene_slot_id(self) -> int:
        return 3

    @property
    def left_box_id(self) -> int:
        return self.left_scene_slot_id

    @property
    def right_box_id(self) -> int:
        return self.right_scene_slot_id

    @property
    def grasp_family(self) -> str:
        if self.left_grasp_mode != self.right_grasp_mode:
            raise ValueError("算法内部吸附模式未收敛为双侧同模式")
        return self.left_grasp_mode

    @property
    def extraction_mode(self) -> str:
        return "direct_updown_lift" if self.uses_direct_updown_lift else "box_pose_rrt"

    @property
    def uses_direct_updown_lift(self) -> bool:
        return self.grasp_family == "front" and max(self.left_row, self.right_row) >= 3

    @property
    def front_distance_m(self) -> float:
        return self.effective_distance_m

    @property
    def top_distance_m(self) -> float:
        return self.effective_distance_m

    @property
    def extract_box_pose_rrt_max_iterations(self) -> int:
        return 400 if max(self.left_row, self.right_row) >= BOX_ROW_COUNT else 160

    @property
    def explicit_targets(self) -> dict[str, dict[str, Any]]:
        return {
            "left": pose6d_target_dict(self.left_tool_pose),
            "right": pose6d_target_dict(self.right_tool_pose),
        }


@dataclass
class MotionSample:
    time_s: float
    joints: dict[str, float]
    updown_m: float
    context: dict[str, Any]
    joint_velocities: dict[str, float] = field(default_factory=dict)
    updown_velocity_m_s: float = 0.0
    joint_accelerations: dict[str, float] = field(default_factory=dict)
    updown_acceleration_m_s2: float = 0.0


@dataclass
class ExecutionPlan:
    task: TaskSpec
    snapshot_path: Path
    summary_path: Path
    stages: dict[int, list[list[MotionSample]]]
    metrics: dict[str, Any]


def parse_task_code(
    raw_code: str,
    front_distance_m: float,
    top_distance_m: float,
) -> TaskSpec:
    code = raw_code.strip().upper()
    if len(code) != 2 or code[0] not in TASK_LAYOUTS or not code[1].isdigit():
        raise ValueError("任务编号必须是 A1..A5 或 B1..B5")
    index = int(code[1])
    if index not in TASK_PAIRS:
        raise ValueError("任务编号必须是 A1..A5 或 B1..B5")
    for value, label in (
        (front_distance_m, "侧吸距离"),
        (top_distance_m, "顶吸距离"),
    ):
        if not math.isfinite(value) or not 0.4 <= value <= 1.3:
            raise ValueError(f"{label}必须在 0.4~1.3m 内，当前 {value}")
    left_box_id, right_box_id = TASK_PAIRS[index]
    return TaskSpec(
        code=code,
        layout=code[0],
        index=index,
        left_box_id=left_box_id,
        right_box_id=right_box_id,
        front_distance_m=float(front_distance_m),
        top_distance_m=float(top_distance_m),
    )


def pose6d_dict(pose: Pose6DValue) -> dict[str, float]:
    return {
        "x": pose.x,
        "y": pose.y,
        "z": pose.z,
        "roll": pose.roll,
        "pitch": pose.pitch,
        "yaw": pose.yaw,
    }


def pose6d_from_dict(value: dict[str, Any], label: str) -> Pose6DValue:
    if not isinstance(value, dict):
        raise ValueError(f"{label} 必须是6D位姿对象")
    try:
        pose = Pose6DValue(
            x=float(value["x"]),
            y=float(value["y"]),
            z=float(value["z"]),
            roll=float(value["roll"]),
            pitch=float(value["pitch"]),
            yaw=float(value["yaw"]),
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"{label} 缺少合法的 x/y/z/roll/pitch/yaw: {exc}") from exc
    if not all(math.isfinite(item) for item in pose.__dict__.values()):
        raise ValueError(f"{label} 含非有限数值")
    return pose


def pose6d_target_dict(pose: Pose6DValue) -> dict[str, Any]:
    orientation = quaternion_xyzw(pose)
    return {
        "frame_id": "base_link",
        "position": [pose.x, pose.y, pose.z],
        "orientation": list(orientation),
    }


def front_face_poses_for_task(task: TaskSpec) -> tuple[Pose6DValue, Pose6DValue]:
    scene_y_shift = 0.05 if task.layout == "A" else 0.0

    def make_pose(box_id: int, side: str) -> Pose6DValue:
        row_from_top = (box_id - 1) // 3 + 1
        center_z = BOTTOM_ROW_FRONT_CENTER_Z_M + (
            BOX_ROW_COUNT - row_from_top
        ) * BOX_HEIGHT_M
        lateral = OUTER_BOX_GRASP_TARGET_Y_M if side == "left" else -OUTER_BOX_GRASP_TARGET_Y_M
        return Pose6DValue(
            x=task.effective_distance_m,
            y=lateral + scene_y_shift,
            z=center_z,
            roll=FRONT_TOOL_RPY[0],
            pitch=FRONT_TOOL_RPY[1],
            yaw=FRONT_TOOL_RPY[2],
        )

    return make_pose(task.left_box_id, "left"), make_pose(task.right_box_id, "right")


def planning_task_from_front_face_poses(
    request_id: str,
    left_front_face_pose: Pose6DValue,
    right_front_face_pose: Pose6DValue,
    *,
    row_count: int = BOX_ROW_COUNT,
    box_height_m: float = BOX_HEIGHT_M,
    box_depth_m: float = BOX_DEPTH_M,
    bottom_row_center_z_m: float = BOTTOM_ROW_FRONT_CENTER_Z_M,
    row_match_tolerance_m: float = ROW_MATCH_TOLERANCE_M,
    top_suction_first_row: int = TOP_SUCTION_FIRST_ROW,
) -> PoseTaskSpec:
    resolution = resolve_front_face_dual_grasp_strategy(
        left_front_face_pose,
        right_front_face_pose,
        row_count=row_count,
        box_height_m=box_height_m,
        box_depth_m=box_depth_m,
        bottom_row_center_z_m=bottom_row_center_z_m,
        row_match_tolerance_m=row_match_tolerance_m,
        top_suction_first_row=top_suction_first_row,
    )
    scene_y_shift = 0.5 * (
        left_front_face_pose.y - OUTER_BOX_GRASP_TARGET_Y_M
        + right_front_face_pose.y + OUTER_BOX_GRASP_TARGET_Y_M
    )
    return PoseTaskSpec(
        code=str(request_id),
        left_front_face_pose=left_front_face_pose,
        right_front_face_pose=right_front_face_pose,
        left_row=resolution.left_row.row_from_top,
        right_row=resolution.right_row.row_from_top,
        left_row_residual_m=resolution.left_row.residual_m,
        right_row_residual_m=resolution.right_row.residual_m,
        left_grasp_mode=resolution.strategy.left.grasp_mode,
        right_grasp_mode=resolution.strategy.right.grasp_mode,
        left_tool_pose=resolution.left_tool_pose,
        right_tool_pose=resolution.right_tool_pose,
        strategy=resolution.strategy,
        scene_y_shift=scene_y_shift,
        effective_distance_m=0.5 * (
            left_front_face_pose.x + right_front_face_pose.x
        ),
    )


def front_face_task_request_fields(
    request_id: str,
    left_front_face_pose: Pose6DValue,
    right_front_face_pose: Pose6DValue,
) -> dict[str, Any]:
    return {
        "request_id": str(request_id),
        "left": {"pose_6d": pose6d_dict(left_front_face_pose)},
        "right": {"pose_6d": pose6d_dict(right_front_face_pose)},
    }


def encode_message(event: str, **fields: Any) -> str:
    return json.dumps({"event": event, **fields}, ensure_ascii=False, separators=(",", ":"))


def decode_message(payload: str) -> dict[str, Any]:
    value = json.loads(payload)
    if not isinstance(value, dict):
        raise ValueError("消息必须是 JSON object")
    return value


def loaded_joint_map(joint_names: Iterable[str]) -> dict[str, float]:
    result: dict[str, float] = {}
    for name in joint_names:
        if name == "turn":
            result[name] = 0.0
            continue
        matched = False
        for side in ("left", "right"):
            prefix = f"{side}_joint"
            if name.startswith(prefix):
                index = int(name.removeprefix(prefix)) - 1
                result[name] = math.radians(LOADED_ARM_POSE_DEG[index])
                matched = True
                break
        if not matched:
            raise ValueError(f"未知执行关节: {name}")
    return result


def _stage_matches(sample: MotionSample, suffix: str) -> bool:
    return str(sample.context.get("stage", "")).endswith(suffix)


def _copy_sample(sample: MotionSample, time_s: float | None = None) -> MotionSample:
    return MotionSample(
        time_s=sample.time_s if time_s is None else float(time_s),
        joints=dict(sample.joints),
        updown_m=float(sample.updown_m),
        context=dict(sample.context),
        joint_velocities=dict(sample.joint_velocities),
        updown_velocity_m_s=float(sample.updown_velocity_m_s),
        joint_accelerations=dict(sample.joint_accelerations),
        updown_acceleration_m_s2=float(sample.updown_acceleration_m_s2),
    )


def _select_samples(samples: list[MotionSample], suffix: str) -> list[MotionSample]:
    return [_copy_sample(sample) for sample in samples if _stage_matches(sample, suffix)]


def _select_extract_samples(samples: list[MotionSample]) -> list[MotionSample]:
    return [
        _copy_sample(sample)
        for sample in samples
        if "/selected_extract_" in str(sample.context.get("stage", ""))
    ]


def _prepend_boundary(current: list[MotionSample], previous: list[MotionSample]) -> list[MotionSample]:
    if not current or not previous:
        return current
    boundary = _copy_sample(previous[-1], current[0].time_s)
    boundary.context.update(current[0].context)
    boundary.context["updown"] = previous[-1].updown_m
    return [boundary, *current]


def split_execution_stages(samples: list[MotionSample]) -> dict[int, list[list[MotionSample]]]:
    pre_contact = _select_samples(samples, "/selected_pre_attach_loaded_to_pre_contact")
    contact = _select_samples(samples, "/selected_pre_attach_pre_contact_to_ik")
    extract = _select_extract_samples(samples)
    loaded = _select_samples(samples, "/selected_loaded_plan")
    place = _select_samples(samples, "/selected_loaded_to_place")
    return_loaded = _select_samples(samples, "/selected_place_to_loaded")
    named = {
        "pre_contact": pre_contact,
        "contact": contact,
        "extract": extract,
        "loaded": loaded,
        "place": place,
        "return_loaded": return_loaded,
    }
    missing = [name for name, stage in named.items() if not stage]
    if missing:
        raise ValueError(f"规划快照缺少阶段: {', '.join(missing)}")
    contact = _prepend_boundary(contact, pre_contact)
    extract = _prepend_boundary(extract, contact)
    loaded = _prepend_boundary(loaded, extract)
    place = _prepend_boundary(place, loaded)
    return_loaded = _prepend_boundary(return_loaded, place)
    return {
        1: [pre_contact],
        2: [contact],
        3: [extract, loaded],
        4: [place],
        5: [],
        6: [return_loaded],
    }


def _max_joint_change(lhs: MotionSample, rhs: MotionSample, joint_names: Iterable[str]) -> float:
    return max(abs(rhs.joints[name] - lhs.joints[name]) for name in joint_names)


def _constant(values: Iterable[float], tolerance: float = 1e-6) -> bool:
    values = list(values)
    return not values or max(values) - min(values) <= tolerance


def validate_stage_contracts(
    task: TaskSpec | PoseTaskSpec,
    stages: dict[int, list[list[MotionSample]]],
    joint_names: list[str],
) -> None:
    def flattened(stage_number: int) -> list[MotionSample]:
        return [sample for segment in stages[stage_number] for sample in segment]

    for stage_number in (1, 2, 3, 4, 6):
        if not flattened(stage_number):
            raise ValueError(f"第 {stage_number} 阶段为空")
    if not _constant(sample.updown_m for sample in flattened(2)):
        raise ValueError("第2阶段违反合同：预接触到吸附时 updown 发生运动")
    extract = stages[3][0]
    if isinstance(task, TaskSpec):
        if task.uses_direct_updown_lift:
            if _max_joint_change(extract[0], extract[-1], joint_names) > 1e-6:
                raise ValueError("直升抽离违反合同：12轴发生运动")
            lift = extract[-1].updown_m - extract[0].updown_m
            if abs(lift - 0.4) > 1e-4:
                raise ValueError(f"直升抽离违反合同：updown 抬升 {lift:.4f}m，不是 0.4m")
        elif not _constant(sample.updown_m for sample in extract):
            raise ValueError("侧吸 RRT 抽离违反合同：updown 发生运动")
    expected_stage3_updown = min(extract[-1].updown_m, 0.45)
    actual_stage3_updown = flattened(3)[-1].updown_m
    if abs(actual_stage3_updown - expected_stage3_updown) > 1e-4:
        raise ValueError(
            f"第3阶段终点 updown={actual_stage3_updown:.4f}m，"
            f"期望 min(抽离终态, 0.45)={expected_stage3_updown:.4f}m"
        )
    expected_updown = {4: 0.1, 6: actual_stage3_updown}
    for stage_number, target in expected_updown.items():
        actual = flattened(stage_number)[-1].updown_m
        if abs(actual - target) > 1e-4:
            raise ValueError(
                f"第{stage_number}阶段终点 updown={actual:.4f}m，期望 {target:.4f}m"
            )


def _motion_vector(
    start: MotionSample,
    goal: MotionSample,
    joint_names: list[str],
) -> list[float]:
    return [goal.joints[name] - start.joints[name] for name in joint_names] + [
        goal.updown_m - start.updown_m
    ]


def _simplify_collinear_samples(
    samples: list[MotionSample],
    joint_names: list[str],
) -> list[MotionSample]:
    if len(samples) <= 2:
        return [_copy_sample(sample) for sample in samples]
    result = [_copy_sample(samples[0])]
    for index in range(1, len(samples) - 1):
        current = samples[index]
        following = samples[index + 1]
        incoming = _motion_vector(result[-1], current, joint_names)
        outgoing = _motion_vector(current, following, joint_names)
        incoming_norm = math.sqrt(sum(value * value for value in incoming))
        outgoing_norm = math.sqrt(sum(value * value for value in outgoing))
        if incoming_norm <= 1e-12 or outgoing_norm <= 1e-12:
            continue
        cosine = sum(
            lhs * rhs for lhs, rhs in zip(incoming, outgoing)
        ) / (incoming_norm * outgoing_norm)
        if cosine >= 1.0 - 1e-10:
            continue
        result.append(_copy_sample(current))
    result.append(_copy_sample(samples[-1]))
    return result


def _minimum_trapezoid_duration(
    distance: float,
    maximum_velocity: float,
    maximum_acceleration: float,
) -> float:
    if distance <= 1e-12:
        return 0.0
    if distance >= maximum_velocity * maximum_velocity / maximum_acceleration:
        return distance / maximum_velocity + maximum_velocity / maximum_acceleration
    return 2.0 * math.sqrt(distance / maximum_acceleration)


def _trapezoid_state(
    elapsed: float,
    duration: float,
    maximum_acceleration: float,
    distance: float,
) -> tuple[float, float]:
    discriminant = max(
        0.0,
        (maximum_acceleration * duration) ** 2
        - 4.0 * maximum_acceleration * distance,
    )
    peak_velocity = (
        maximum_acceleration * duration - math.sqrt(discriminant)
    ) / 2.0
    acceleration_time = peak_velocity / maximum_acceleration
    cruise_end = duration - acceleration_time
    if elapsed <= acceleration_time:
        return (
            0.5 * maximum_acceleration * elapsed * elapsed,
            maximum_acceleration * elapsed,
        )
    if elapsed < cruise_end:
        acceleration_distance = (
            0.5 * maximum_acceleration * acceleration_time * acceleration_time
        )
        return (
            acceleration_distance
            + peak_velocity * (elapsed - acceleration_time),
            peak_velocity,
        )
    remaining = max(0.0, duration - elapsed)
    return (
        distance - 0.5 * maximum_acceleration * remaining * remaining,
        maximum_acceleration * remaining,
    )


def _assign_continuous_joint_velocities(
    samples: list[MotionSample],
    joint_names: list[str],
    maximum_velocity: float,
    maximum_acceleration: float,
) -> None:
    count = len(samples)
    for sample in samples:
        sample.joint_velocities = {name: 0.0 for name in joint_names}
    if count <= 2:
        return
    for name in joint_names:
        candidates = [0.0] * count
        forced_zero = [False] * count
        forced_zero[0] = True
        forced_zero[-1] = True
        edge_changes = [
            samples[index + 1].joints[name] - samples[index].joints[name]
            for index in range(count - 1)
        ]
        for index in range(1, count - 1):
            previous_dt = samples[index].time_s - samples[index - 1].time_s
            next_dt = samples[index + 1].time_s - samples[index].time_s
            previous_slope = (
                samples[index].joints[name] - samples[index - 1].joints[name]
            ) / previous_dt
            next_slope = (
                samples[index + 1].joints[name] - samples[index].joints[name]
            ) / next_dt
            previous_direction = next(
                (
                    edge_changes[edge_index]
                    for edge_index in range(index - 1, -1, -1)
                    if abs(edge_changes[edge_index]) > 1e-10
                ),
                0.0,
            )
            next_direction = next(
                (
                    edge_changes[edge_index]
                    for edge_index in range(index, len(edge_changes))
                    if abs(edge_changes[edge_index]) > 1e-10
                ),
                0.0,
            )
            if previous_direction * next_direction < 0.0:
                forced_zero[index] = True
                continue
            if abs(previous_slope) <= 1e-12 or abs(next_slope) <= 1e-12:
                velocity = (
                    samples[index + 1].joints[name]
                    - samples[index - 1].joints[name]
                ) / (previous_dt + next_dt)
            else:
                previous_weight = 2.0 * next_dt + previous_dt
                next_weight = next_dt + 2.0 * previous_dt
                velocity = (previous_weight + next_weight) / (
                    previous_weight / previous_slope
                    + next_weight / next_slope
                )
            candidates[index] = max(
                -maximum_velocity,
                min(maximum_velocity, velocity),
            )

        for _ in range(4):
            for index in range(1, count):
                if forced_zero[index]:
                    candidates[index] = 0.0
                    continue
                dt = samples[index].time_s - samples[index - 1].time_s
                lower = candidates[index - 1] - maximum_acceleration * dt
                upper = candidates[index - 1] + maximum_acceleration * dt
                candidates[index] = max(lower, min(upper, candidates[index]))
            for index in range(count - 2, -1, -1):
                if forced_zero[index]:
                    candidates[index] = 0.0
                    continue
                dt = samples[index + 1].time_s - samples[index].time_s
                lower = candidates[index + 1] - maximum_acceleration * dt
                upper = candidates[index + 1] + maximum_acceleration * dt
                candidates[index] = max(lower, min(upper, candidates[index]))
        for index, sample in enumerate(samples):
            sample.joint_velocities[name] = candidates[index]


def _assign_continuous_updown_velocities(
    samples: list[MotionSample],
    maximum_velocity: float,
    maximum_acceleration: float,
) -> None:
    count = len(samples)
    for sample in samples:
        sample.updown_velocity_m_s = 0.0
    if count <= 2:
        return
    candidates = [0.0] * count
    forced_zero = [False] * count
    forced_zero[0] = True
    forced_zero[-1] = True
    edge_changes = [
        samples[index + 1].updown_m - samples[index].updown_m
        for index in range(count - 1)
    ]
    for index in range(1, count - 1):
        previous_dt = samples[index].time_s - samples[index - 1].time_s
        next_dt = samples[index + 1].time_s - samples[index].time_s
        previous_slope = (
            samples[index].updown_m - samples[index - 1].updown_m
        ) / previous_dt
        next_slope = (
            samples[index + 1].updown_m - samples[index].updown_m
        ) / next_dt
        previous_direction = next(
            (
                edge_changes[edge_index]
                for edge_index in range(index - 1, -1, -1)
                if abs(edge_changes[edge_index]) > 1e-10
            ),
            0.0,
        )
        next_direction = next(
            (
                edge_changes[edge_index]
                for edge_index in range(index, len(edge_changes))
                if abs(edge_changes[edge_index]) > 1e-10
            ),
            0.0,
        )
        if previous_direction * next_direction < 0.0:
            forced_zero[index] = True
            continue
        if abs(previous_slope) <= 1e-12 or abs(next_slope) <= 1e-12:
            velocity = (
                samples[index + 1].updown_m - samples[index - 1].updown_m
            ) / (previous_dt + next_dt)
        else:
            previous_weight = 2.0 * next_dt + previous_dt
            next_weight = next_dt + 2.0 * previous_dt
            velocity = (previous_weight + next_weight) / (
                previous_weight / previous_slope + next_weight / next_slope
            )
        candidates[index] = max(-maximum_velocity, min(maximum_velocity, velocity))

    for _ in range(4):
        for index in range(1, count):
            if forced_zero[index]:
                candidates[index] = 0.0
                continue
            dt = samples[index].time_s - samples[index - 1].time_s
            lower = candidates[index - 1] - maximum_acceleration * dt
            upper = candidates[index - 1] + maximum_acceleration * dt
            candidates[index] = max(lower, min(upper, candidates[index]))
        for index in range(count - 2, -1, -1):
            if forced_zero[index]:
                candidates[index] = 0.0
                continue
            dt = samples[index + 1].time_s - samples[index].time_s
            lower = candidates[index + 1] - maximum_acceleration * dt
            upper = candidates[index + 1] + maximum_acceleration * dt
            candidates[index] = max(lower, min(upper, candidates[index]))
    for index, sample in enumerate(samples):
        sample.updown_velocity_m_s = candidates[index]


def _assign_accelerations(
    samples: list[MotionSample],
    joint_names: list[str],
    maximum_joint_acceleration: float,
    maximum_updown_acceleration: float,
) -> None:
    for sample in samples:
        sample.joint_accelerations = {name: 0.0 for name in joint_names}
        sample.updown_acceleration_m_s2 = 0.0
    for index in range(1, len(samples) - 1):
        duration = samples[index + 1].time_s - samples[index - 1].time_s
        if duration <= 0.0:
            continue
        for name in joint_names:
            acceleration = (
                samples[index + 1].joint_velocities[name]
                - samples[index - 1].joint_velocities[name]
            ) / duration
            samples[index].joint_accelerations[name] = max(
                -maximum_joint_acceleration,
                min(maximum_joint_acceleration, acceleration),
            )
        updown_acceleration = (
            samples[index + 1].updown_velocity_m_s
            - samples[index - 1].updown_velocity_m_s
        ) / duration
        samples[index].updown_acceleration_m_s2 = max(
            -maximum_updown_acceleration,
            min(maximum_updown_acceleration, updown_acceleration),
        )


def retime_segment(
    samples: list[MotionSample],
    joint_names: list[str],
    *,
    rate_hz: float,
    max_joint_speed_deg_s: float,
    max_updown_speed_m_s: float,
    speed_scale: float = 1.0,
    max_joint_acceleration_deg_s2: float = 60.0,
    max_updown_acceleration_m_s2: float = 0.05,
) -> list[MotionSample]:
    if len(samples) < 2:
        raise ValueError("轨迹段至少需要两个采样点")
    if (
        rate_hz <= 0.0
        or max_joint_speed_deg_s <= 0.0
        or max_joint_acceleration_deg_s2 <= 0.0
        or max_updown_acceleration_m_s2 <= 0.0
        or max_updown_speed_m_s <= 0.0
        or speed_scale <= 0.0
    ):
        raise ValueError("轨迹频率和速度上限必须为正数")
    period = 1.0 / rate_hz
    effective_max_joint_speed = math.radians(max_joint_speed_deg_s) * speed_scale
    maximum_joint_acceleration = math.radians(max_joint_acceleration_deg_s2)
    effective_max_updown_speed = max_updown_speed_m_s
    control_samples = _simplify_collinear_samples(samples, joint_names)
    edge_durations: list[float] = []
    maximum_path_acceleration = math.inf
    has_path_motion = False
    for start, goal in zip(control_samples, control_samples[1:]):
        original_dt = max(0.0, goal.time_s - start.time_s) / speed_scale
        maximum_joint_change = _max_joint_change(start, goal, joint_names)
        updown_change = goal.updown_m - start.updown_m
        duration = max(
            period,
            original_dt,
            maximum_joint_change / effective_max_joint_speed,
            abs(updown_change) / effective_max_updown_speed,
        )
        edge_durations.append(duration)
        for name in joint_names:
            slope = abs(goal.joints[name] - start.joints[name]) / duration
            if slope > 1e-12:
                has_path_motion = True
                maximum_path_acceleration = min(
                    maximum_path_acceleration,
                    maximum_joint_acceleration / slope,
                )
        updown_slope = abs(updown_change) / duration
        if updown_slope > 1e-12:
            has_path_motion = True
            maximum_path_acceleration = min(
                maximum_path_acceleration,
                max_updown_acceleration_m_s2 / updown_slope,
            )

    path_offsets = [0.0]
    for duration in edge_durations:
        path_offsets.append(path_offsets[-1] + duration)
    path_length = path_offsets[-1]
    if has_path_motion:
        duration = _minimum_trapezoid_duration(
            path_length,
            1.0,
            maximum_path_acceleration,
        )
    else:
        duration = path_length
    frame_count = max(1, int(math.ceil(duration * rate_hz - 1e-9)))
    duration = frame_count * period
    result: list[MotionSample] = []
    edge_index = 0
    for frame_index in range(frame_count + 1):
        elapsed = frame_index * period
        if has_path_motion:
            path_position, _ = _trapezoid_state(
                elapsed,
                duration,
                maximum_path_acceleration,
                path_length,
            )
        else:
            path_position = path_length * min(1.0, elapsed / duration)
        while (
            edge_index + 1 < len(edge_durations)
            and path_position >= path_offsets[edge_index + 1] - 1e-12
        ):
            edge_index += 1
        start = control_samples[edge_index]
        goal = control_samples[edge_index + 1]
        edge_duration = edge_durations[edge_index]
        ratio = min(
            1.0,
            max(0.0, (path_position - path_offsets[edge_index]) / edge_duration),
        )
        result.append(
            MotionSample(
                time_s=elapsed,
                joints={
                    name: start.joints[name]
                    + (goal.joints[name] - start.joints[name]) * ratio
                    for name in joint_names
                },
                updown_m=start.updown_m + (goal.updown_m - start.updown_m) * ratio,
                context=dict(goal.context),
            )
        )
    _assign_continuous_joint_velocities(
        result,
        joint_names,
        effective_max_joint_speed,
        maximum_joint_acceleration,
    )
    _assign_continuous_updown_velocities(
        result,
        effective_max_updown_speed,
        max_updown_acceleration_m_s2,
    )
    _assign_accelerations(
        result,
        joint_names,
        maximum_joint_acceleration,
        max_updown_acceleration_m_s2,
    )
    return result


def retime_all_stages(
    stages: dict[int, list[list[MotionSample]]],
    joint_names: list[str],
    *,
    rate_hz: float,
    max_joint_speed_deg_s: float,
    max_updown_speed_m_s: float,
    speed_scale: float = 1.0,
    max_joint_acceleration_deg_s2: float = 60.0,
    max_updown_acceleration_m_s2: float = 0.05,
) -> dict[int, list[list[MotionSample]]]:
    return {
        stage_number: [
            retime_segment(
                segment,
                joint_names,
                rate_hz=rate_hz,
                max_joint_speed_deg_s=max_joint_speed_deg_s,
                max_updown_speed_m_s=max_updown_speed_m_s,
                speed_scale=speed_scale,
                max_joint_acceleration_deg_s2=max_joint_acceleration_deg_s2,
                max_updown_acceleration_m_s2=max_updown_acceleration_m_s2,
            )
            for segment in segments
        ]
        for stage_number, segments in stages.items()
    }
