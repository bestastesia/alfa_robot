from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable


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
    2: "笛卡尔前进 5cm 并打开双侧电磁阀",
    3: "吸附后抽离并回负重姿态（updown 仅在高于0.45m时降至0.45m）",
    4: "updown 单独下降到 0.1m",
    5: "双臂运动到放置位",
    6: "关闭双侧电磁阀",
    7: "双臂与 updown 同步回负重位",
}
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
    def effective_distance_m(self) -> float:
        return self.front_distance_m if self.index in FRONT_TASKS else self.top_distance_m


@dataclass
class MotionSample:
    time_s: float
    joints: dict[str, float]
    updown_m: float
    context: dict[str, Any]
    joint_velocities: dict[str, float] = field(default_factory=dict)


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


def split_seven_stages(samples: list[MotionSample]) -> dict[int, list[list[MotionSample]]]:
    pre_contact = _select_samples(samples, "/selected_pre_attach_loaded_to_pre_contact")
    contact = _select_samples(samples, "/selected_pre_attach_pre_contact_to_ik")
    extract = _select_extract_samples(samples)
    loaded = _select_samples(samples, "/selected_loaded_plan")
    place_height = _select_samples(samples, "/selected_loaded_to_place_height")
    place_arms = _select_samples(samples, "/selected_loaded_to_place_arms")
    return_loaded = _select_samples(samples, "/selected_place_to_loaded")
    named = {
        "pre_contact": pre_contact,
        "contact": contact,
        "extract": extract,
        "loaded": loaded,
        "place_height": place_height,
        "place_arms": place_arms,
        "return_loaded": return_loaded,
    }
    missing = [name for name, stage in named.items() if not stage]
    if missing:
        raise ValueError(f"规划快照缺少阶段: {', '.join(missing)}")
    contact = _prepend_boundary(contact, pre_contact)
    extract = _prepend_boundary(extract, contact)
    loaded = _prepend_boundary(loaded, extract)
    place_height = _prepend_boundary(place_height, loaded)
    place_arms = _prepend_boundary(place_arms, place_height)
    return_loaded = _prepend_boundary(return_loaded, place_arms)
    return {
        1: [pre_contact],
        2: [contact],
        3: [extract, loaded],
        4: [place_height],
        5: [place_arms],
        6: [],
        7: [return_loaded],
    }


def _max_joint_change(lhs: MotionSample, rhs: MotionSample, joint_names: Iterable[str]) -> float:
    return max(abs(rhs.joints[name] - lhs.joints[name]) for name in joint_names)


def _constant(values: Iterable[float], tolerance: float = 1e-6) -> bool:
    values = list(values)
    return not values or max(values) - min(values) <= tolerance


def validate_stage_contracts(
    task: TaskSpec,
    stages: dict[int, list[list[MotionSample]]],
    joint_names: list[str],
) -> None:
    def flattened(stage_number: int) -> list[MotionSample]:
        return [sample for segment in stages[stage_number] for sample in segment]

    for stage_number in (1, 2, 3, 4, 5, 7):
        if not flattened(stage_number):
            raise ValueError(f"第 {stage_number} 阶段为空")
    if not _constant(sample.updown_m for sample in flattened(2)):
        raise ValueError("第2阶段违反合同：预接触到吸附时 updown 发生运动")
    if _max_joint_change(flattened(4)[0], flattened(4)[-1], joint_names) > 1e-6:
        raise ValueError("第4阶段违反合同：updown 下降时双臂发生运动")
    if not _constant(sample.updown_m for sample in flattened(5)):
        raise ValueError("第5阶段违反合同：运动到放置位时 updown 发生运动")
    extract = stages[3][0]
    if task.index in DIRECT_LIFT_TASKS:
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
    expected_updown = {4: 0.1, 5: 0.1, 7: 0.3}
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


def retime_segment(
    samples: list[MotionSample],
    joint_names: list[str],
    *,
    rate_hz: float,
    max_joint_speed_deg_s: float,
    max_updown_speed_m_s: float,
    speed_scale: float = 1.0,
    max_joint_acceleration_deg_s2: float = 60.0,
) -> list[MotionSample]:
    if len(samples) < 2:
        raise ValueError("轨迹段至少需要两个采样点")
    if (
        rate_hz <= 0.0
        or max_joint_speed_deg_s <= 0.0
        or max_joint_acceleration_deg_s2 <= 0.0
        or max_updown_speed_m_s <= 0.0
        or speed_scale <= 0.0
    ):
        raise ValueError("轨迹频率和速度上限必须为正数")
    period = 1.0 / rate_hz
    effective_max_joint_speed = math.radians(max_joint_speed_deg_s) * speed_scale
    maximum_joint_acceleration = math.radians(max_joint_acceleration_deg_s2)
    effective_max_updown_speed = max_updown_speed_m_s * speed_scale
    control_samples = _simplify_collinear_samples(samples, joint_names)
    edge_durations: list[float] = []
    maximum_path_acceleration = math.inf
    has_joint_motion = False
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
                has_joint_motion = True
                maximum_path_acceleration = min(
                    maximum_path_acceleration,
                    maximum_joint_acceleration / slope,
                )

    path_offsets = [0.0]
    for duration in edge_durations:
        path_offsets.append(path_offsets[-1] + duration)
    path_length = path_offsets[-1]
    if has_joint_motion:
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
        if has_joint_motion:
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
            )
            for segment in segments
        ]
        for stage_number, segments in stages.items()
    }
