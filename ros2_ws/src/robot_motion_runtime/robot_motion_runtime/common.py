from __future__ import annotations

import json
import math
import time
from copy import deepcopy
from typing import Iterable

from builtin_interfaces.msg import Time
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


DEFAULT_MOTION_JOINTS = [
    "updown",
    "turn",
    "pitch",
    "leftjoint1",
    "leftjoint2",
    "leftjoint3",
    "leftjoint4",
    "leftjoint5",
    "leftjoint6",
    "rightjoint1",
    "rightjoint2",
    "rightjoint3",
    "rightjoint4",
    "rightjoint5",
    "rightjoint6",
]

HARDWARE_TO_MODEL_JOINT_ALIASES = {
    "left_joint1": "leftjoint1",
    "left_joint2": "leftjoint2",
    "left_joint3": "leftjoint3",
    "left_joint4": "leftjoint4",
    "left_joint5": "leftjoint5",
    "left_joint6": "leftjoint6",
    "right_joint1": "rightjoint1",
    "right_joint2": "rightjoint2",
    "right_joint3": "rightjoint3",
    "right_joint4": "rightjoint4",
    "right_joint5": "rightjoint5",
    "right_joint6": "rightjoint6",
}

MODEL_TO_HARDWARE_JOINT_ALIASES = {
    model: hardware for hardware, model in HARDWARE_TO_MODEL_JOINT_ALIASES.items()
}

# 13 轴 EtherCAT 硬件 action 期望的 joint 名字与顺序（右臂在前，含 turn）。
# 唯一权威方向/顺序定义在 alfa_robot_execution_bridge.joints；这里只复用命名别名做
# model<->hardware 转换，不重新定义方向表。
REAL_ARM_JOINT_NAMES = [
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "turn",
]


def canonical_joint_name(name: str) -> str:
    """Map a hardware-aliased joint name (left_joint1...) to its model name.

    Unlike ``model_joint_name``, this returns the input unchanged when it is
    not a recognized hardware alias (e.g. it is already a model name), rather
    than returning ``None``.
    """
    return HARDWARE_TO_MODEL_JOINT_ALIASES.get(str(name), str(name))


def hardware_joint_name(name: str) -> str:
    """Map a model joint name (leftjoint1...) to its hardware alias."""
    return MODEL_TO_HARDWARE_JOINT_ALIASES.get(str(name), str(name))


def clamp_motion_scale(requested: float, default: float = 1.0) -> float:
    """Resolve a velocity/acceleration scale field and clamp it to [0.0, 1.0].

    Callers treat <= 0.0 as "unset" and fall back to ``default`` (the existing
    convention in the adapter nodes). Anything above 1.0 previously passed
    through unclamped and was silently ignored by the execution layer, which
    let callers believe an out-of-range speedup was honored.
    """
    value = requested if requested > 0.0 else default
    return max(0.0, min(1.0, value))


def now_ms() -> int:
    return int(time.time() * 1000)


def stamp_is_zero(stamp: Time) -> bool:
    return stamp.sec == 0 and stamp.nanosec == 0


def duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1e-9


def make_duration(seconds: float):
    msg = JointTrajectoryPoint().time_from_start
    whole = math.floor(max(0.0, seconds))
    msg.sec = int(whole)
    msg.nanosec = int(round((max(0.0, seconds) - whole) * 1e9))
    if msg.nanosec >= 1_000_000_000:
        msg.sec += 1
        msg.nanosec -= 1_000_000_000
    return msg


def joint_state_positions(joint_state: JointState) -> dict[str, float]:
    return {
        str(name): float(joint_state.position[index])
        for index, name in enumerate(joint_state.name)
        if index < len(joint_state.position)
    }


def canonical_joint_names(states: Iterable[JointState]) -> list[str]:
    names: list[str] = []
    for state in states:
        for name in state.name:
            if name not in names:
                names.append(str(name))
    return names or list(DEFAULT_MOTION_JOINTS)


def merge_joint_state(base: JointState, overlay: JointState) -> JointState:
    merged = JointState()
    merged.header = overlay.header if overlay.name else base.header
    values = joint_state_positions(base)
    values.update(joint_state_positions(overlay))
    merged.name = list(values.keys())
    merged.position = [values[name] for name in merged.name]
    merged.velocity = [0.0] * len(merged.name)
    merged.effort = [0.0] * len(merged.name)
    return merged


def joint_distance(lhs: JointState, rhs: JointState) -> float:
    lhs_values = joint_state_positions(lhs)
    rhs_values = joint_state_positions(rhs)
    total = 0.0
    for name, rhs_value in rhs_values.items():
        if name in lhs_values:
            total += abs(rhs_value - lhs_values[name])
    return total


def make_interpolated_trajectory(
    start: JointState,
    goal: JointState,
    duration_s: float = 1.0,
    max_joint_step_rad: float = math.radians(5.0),
    max_linear_step_m: float = 0.01,
) -> JointTrajectory:
    names = canonical_joint_names([start, goal])
    start_values = joint_state_positions(start)
    goal_values = joint_state_positions(goal)
    max_ratio = 1.0
    for name in names:
        delta = abs(goal_values.get(name, start_values.get(name, 0.0)) - start_values.get(name, 0.0))
        step = max_linear_step_m if name == "updown" else max_joint_step_rad
        max_ratio = max(max_ratio, delta / max(step, 1e-9))
    steps = max(2, int(math.ceil(max_ratio)) + 1)

    trajectory = JointTrajectory()
    trajectory.joint_names = names
    for step_index in range(steps):
        ratio = 0.0 if steps <= 1 else float(step_index) / float(steps - 1)
        point = JointTrajectoryPoint()
        point.time_from_start = make_duration(duration_s * ratio)
        for name in names:
            start_value = start_values.get(name, 0.0)
            goal_value = goal_values.get(name, start_value)
            point.positions.append(start_value + (goal_value - start_value) * ratio)
        trajectory.points.append(point)
    return trajectory


def point_positions_equal(lhs: JointTrajectoryPoint, rhs: JointTrajectoryPoint, tolerance: float = 1e-9) -> bool:
    if len(lhs.positions) != len(rhs.positions):
        return False
    return all(abs(float(a) - float(b)) <= tolerance for a, b in zip(lhs.positions, rhs.positions))


def copy_point_with_time(point: JointTrajectoryPoint, seconds: float) -> JointTrajectoryPoint:
    out = deepcopy(point)
    out.time_from_start = make_duration(seconds)
    return out


def concatenate_trajectories(first: JointTrajectory, second: JointTrajectory) -> JointTrajectory:
    if not first.points:
        return deepcopy(second)
    if not second.points:
        return deepcopy(first)
    if list(first.joint_names) != list(second.joint_names):
        raise ValueError(
            "cannot concatenate trajectories with different joint_names: "
            f"{list(first.joint_names)} != {list(second.joint_names)}"
        )

    out = JointTrajectory()
    out.header = first.header
    out.joint_names = list(first.joint_names)
    out.points = [deepcopy(point) for point in first.points]
    offset_s = duration_seconds(out.points[-1].time_from_start)
    first_tail = out.points[-1]
    for index, point in enumerate(second.points):
        point_s = duration_seconds(point.time_from_start)
        if index == 0 and point_s <= 1e-9 and point_positions_equal(first_tail, point):
            continue
        out.points.append(copy_point_with_time(point, offset_s + point_s))
    return out


def sample_trajectory_positions(trajectory: JointTrajectory, sample_time_s: float) -> list[float]:
    if not trajectory.points:
        return []
    if sample_time_s <= 0.0:
        return list(trajectory.points[0].positions)
    previous = trajectory.points[0]
    previous_time = duration_seconds(previous.time_from_start)
    for point in trajectory.points[1:]:
        point_time = duration_seconds(point.time_from_start)
        if sample_time_s <= point_time:
            if point_time <= previous_time:
                return list(point.positions)
            alpha = (sample_time_s - previous_time) / (point_time - previous_time)
            return [
                float(a) + (float(b) - float(a)) * alpha
                for a, b in zip(previous.positions, point.positions)
            ]
        previous = point
        previous_time = point_time
    return list(trajectory.points[-1].positions)


def resample_trajectory(trajectory: JointTrajectory, rate_hz: float) -> JointTrajectory:
    if not trajectory.points or rate_hz <= 0.0:
        return deepcopy(trajectory)
    duration_s = duration_seconds(trajectory.points[-1].time_from_start)
    if duration_s <= 0.0:
        return deepcopy(trajectory)
    period_s = 1.0 / rate_hz
    sample_count = max(2, int(math.ceil(duration_s / period_s)) + 1)
    out = JointTrajectory()
    out.header = trajectory.header
    out.joint_names = list(trajectory.joint_names)
    for index in range(sample_count):
        t = duration_s if index == sample_count - 1 else min(duration_s, index * period_s)
        point = JointTrajectoryPoint()
        point.time_from_start = make_duration(t)
        point.positions = sample_trajectory_positions(trajectory, t)
        out.points.append(point)
    return out


class RuntimeStatusPublisher:
    def __init__(self, node: Node, service_name: str, role: str) -> None:
        self.node = node
        self.service_name = service_name
        self.role = role
        self.publisher = node.create_publisher(String, "/robot_motion/runtime_status", 10)
        self.last_state = "starting"
        self.last_detail = ""
        self.request_count = 0
        self.success_count = 0
        self.failure_count = 0
        self.last_started_ms = 0
        self.last_finished_ms = 0
        self.timer = node.create_timer(1.0, self.publish)

    def mark_ready(self, detail: str = "") -> None:
        self.last_state = "ready"
        self.last_detail = detail
        self.publish()

    def mark_running(self, detail: str = "") -> None:
        self.request_count += 1
        self.last_started_ms = now_ms()
        self.last_state = "running"
        self.last_detail = detail
        self.publish()

    def mark_done(self, ok: bool, detail: str = "") -> None:
        self.last_finished_ms = now_ms()
        self.last_state = "ready" if ok else "error"
        self.last_detail = detail
        if ok:
            self.success_count += 1
        else:
            self.failure_count += 1
        self.publish()

    def publish(self) -> None:
        msg = String()
        msg.data = json.dumps(
            {
                "node": self.node.get_name(),
                "service": self.service_name,
                "role": self.role,
                "state": self.last_state,
                "detail": self.last_detail,
                "request_count": self.request_count,
                "success_count": self.success_count,
                "failure_count": self.failure_count,
                "last_started_ms": self.last_started_ms,
                "last_finished_ms": self.last_finished_ms,
                "stamp_ms": now_ms(),
            },
            ensure_ascii=False,
        )
        self.publisher.publish(msg)
