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

# EtherCAT<->ROS 方向符号的唯一权威来源是 alfa_robot_execution_bridge.joints
# （FLIPPED_JOINT_NAMES = left_joint3/left_joint5/right_joint2/right_joint4）。
# 从 /joint_states 读回来的是硬件(EtherCAT)符号，喂给 MoveIt/规划前必须与发送侧
# (ros_to_ethercat_position) 对称地校准回 ROS 符号，否则被翻转的关节姿态是反的，
# 会导致镜像姿态错误与自碰撞误报。这里复用该表，不重新定义方向。
try:
    from alfa_robot_execution_bridge.joints import (
        ROS_TO_ETHERCAT_SIGN_BY_JOINT as _EC_SIGN_BY_JOINT,
        ethercat_to_ros_position as _ethercat_to_ros_position,
        ros_to_ethercat_position as _ros_to_ethercat_position,
    )
except Exception:  # pragma: no cover - bridge 不可用时退化为不校准(并在使用处告警)
    _EC_SIGN_BY_JOINT = {}

    def _ethercat_to_ros_position(joint_name: str, value: float) -> float:
        return float(value)

    def _ros_to_ethercat_position(joint_name: str, value: float) -> float:
        return float(value)


def ethercat_to_ros_hardware_position(hardware_name: str, value: float) -> float:
    """把 /joint_states 上的硬件(EtherCAT)符号值校准回 ROS 符号。
    只对方向表里已知的关节(13 轴机械臂 + turn)生效；updown/track/pitch 等不在表里的
    保持原值(它们没有方向翻转)。"""
    if hardware_name in _EC_SIGN_BY_JOINT:
        return _ethercat_to_ros_position(hardware_name, value)
    return float(value)


def ros_to_ethercat_hardware_position(hardware_name: str, value: float) -> float:
    """发送轨迹到硬件 action 前，把 ROS 符号值翻成硬件(EtherCAT)符号，与读取侧对称。
    只对方向表里已知关节生效。符号是自身逆运算，与 ethercat_to_ros 同表。"""
    if hardware_name in _EC_SIGN_BY_JOINT:
        return _ros_to_ethercat_position(hardware_name, value)
    return float(value)


DEFAULT_MOTION_JOINTS = [
    "updown",
    "turn",
    "pitch",
    "left_joint1",
    "left_joint2",
    "left_joint3",
    "left_joint4",
    "left_joint5",
    "left_joint6",
    "right_joint1",
    "right_joint2",
    "right_joint3",
    "right_joint4",
    "right_joint5",
    "right_joint6",
]

HARDWARE_TO_MODEL_JOINT_ALIASES = {
    "left_joint1": "left_joint1",
    "left_joint2": "left_joint2",
    "left_joint3": "left_joint3",
    "left_joint4": "left_joint4",
    "left_joint5": "left_joint5",
    "left_joint6": "left_joint6",
    "right_joint1": "right_joint1",
    "right_joint2": "right_joint2",
    "right_joint3": "right_joint3",
    "right_joint4": "right_joint4",
    "right_joint5": "right_joint5",
    "right_joint6": "right_joint6",
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

MODEL_JOINT_SET = set(DEFAULT_MOTION_JOINTS)


def model_joint_name(name: str) -> str | None:
    normalized = HARDWARE_TO_MODEL_JOINT_ALIASES.get(str(name), str(name))
    return normalized if normalized in MODEL_JOINT_SET else None


def canonical_joint_name(name: str) -> str:
    """Map a hardware-aliased joint name (left_joint1...) to its model name.

    Unlike ``model_joint_name``, this returns the input unchanged when it is
    not a recognized hardware alias (e.g. it is already a model name), rather
    than returning ``None``.
    """
    return HARDWARE_TO_MODEL_JOINT_ALIASES.get(str(name), str(name))


def hardware_joint_name(name: str) -> str:
    """Map a model joint name (left_joint1...) to its hardware alias."""
    return MODEL_TO_HARDWARE_JOINT_ALIASES.get(str(name), str(name))


def normalize_joint_state_for_model(joint_state: JointState) -> JointState:
    values: dict[str, float] = {}
    velocities: dict[str, float] = {}
    efforts: dict[str, float] = {}

    def record(index: int, prefer_existing: bool) -> None:
        if index >= len(joint_state.name) or index >= len(joint_state.position):
            return
        hardware_name = str(joint_state.name[index])
        model_name = model_joint_name(hardware_name)
        if model_name is None:
            return
        if prefer_existing and model_name in values:
            return
        # /joint_states 是硬件(EtherCAT)符号；用硬件名把被翻转的关节校准回 ROS 符号，
        # 与发送轨迹侧 ros_to_ethercat_position 对称。非翻转关节为原值。
        values[model_name] = ethercat_to_ros_hardware_position(
            hardware_name, float(joint_state.position[index])
        )
        if index < len(joint_state.velocity):
            velocities[model_name] = ethercat_to_ros_hardware_position(
                hardware_name, float(joint_state.velocity[index])
            )
        if index < len(joint_state.effort):
            efforts[model_name] = float(joint_state.effort[index])

    for index, name in enumerate(joint_state.name):
        if str(name) in MODEL_JOINT_SET:
            record(index, prefer_existing=False)
    for index, name in enumerate(joint_state.name):
        if str(name) not in MODEL_JOINT_SET:
            record(index, prefer_existing=True)

    out = JointState()
    out.header = joint_state.header
    out.name = [name for name in DEFAULT_MOTION_JOINTS if name in values]
    out.position = [values[name] for name in out.name]
    if joint_state.velocity:
        out.velocity = [velocities.get(name, 0.0) for name in out.name]
    if joint_state.effort:
        out.effort = [efforts.get(name, 0.0) for name in out.name]
    return out


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


def make_fixed_rate_interpolated_trajectory(
    start: JointState,
    goal: JointState,
    rate_hz: float = 10.0,
    max_joint_step_rad: float = math.radians(4.5),
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
    segment_count = max(1, int(math.ceil(max_ratio)))
    period_s = 1.0 / max(rate_hz, 1e-9)

    trajectory = JointTrajectory()
    trajectory.joint_names = names
    for step_index in range(segment_count + 1):
        ratio = float(step_index) / float(segment_count)
        point = JointTrajectoryPoint()
        point.time_from_start = make_duration(step_index * period_s)
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
