"""GPU-independent request validation and serialized planner orchestration."""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Protocol, Sequence


AUTHORITY_JOINT_NAMES = (
    "updown",
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
)


@dataclass(frozen=True)
class JointValues:
    names: tuple[str, ...]
    positions: tuple[float, ...]


@dataclass(frozen=True)
class AttachedPayload:
    object_id: str
    box_id: int
    side: str
    link_name: str
    center_xyz: tuple[float, float, float]
    orientation_xyzw: tuple[float, float, float, float]
    size_xyz: tuple[float, float, float]


@dataclass(frozen=True)
class SegmentRequest:
    request_id: str
    scene_id: str
    start: JointValues
    goal: JointValues
    attached_payloads: tuple[AttachedPayload, ...] = ()
    force_graph: bool = True
    max_attempts: int = 3
    timeout_s: float = 0.5


@dataclass(frozen=True)
class BackendTrajectory:
    joint_names: tuple[str, ...]
    positions: tuple[tuple[float, ...], ...]
    times_s: tuple[float, ...]
    velocities: tuple[tuple[float, ...], ...] = ()
    accelerations: tuple[tuple[float, ...], ...] = ()


@dataclass(frozen=True)
class BackendResult:
    success: bool
    message: str
    planner_method: str = "curobo_plan_cspace"
    trajectory: BackendTrajectory | None = None
    solve_time_ms: float = 0.0
    endpoint_check_time_ms: float = 0.0
    graph_time_ms: float = 0.0
    trajopt_time_ms: float = 0.0
    interpolation_time_ms: float = 0.0


@dataclass(frozen=True)
class SegmentResponse:
    success: bool
    message: str
    planner_method: str = ""
    trajectory: BackendTrajectory | None = None
    total_time_ms: float = 0.0
    solve_time_ms: float = 0.0
    queue_time_ms: float = 0.0
    endpoint_check_time_ms: float = 0.0
    graph_time_ms: float = 0.0
    trajopt_time_ms: float = 0.0
    interpolation_time_ms: float = 0.0


class SegmentBackend(Protocol):
    joint_names: Sequence[str]
    joint_limits: dict[str, tuple[float, float]]

    def plan_cspace(
        self,
        start_positions: Sequence[float],
        goal_positions: Sequence[float],
        attached_payloads: Sequence[AttachedPayload],
        *,
        scene_id: str,
        force_graph: bool,
        max_attempts: int,
        timeout_s: float,
    ) -> BackendResult: ...


def reorder_joint_values(
    values: JointValues,
    *,
    authority_names: Sequence[str] = AUTHORITY_JOINT_NAMES,
) -> tuple[float, ...]:
    if len(values.names) != len(values.positions):
        raise ValueError("joint_state_name_position_size_mismatch")
    if len(set(values.names)) != len(values.names):
        raise ValueError("joint_state_duplicate_joint")
    if set(values.names) != set(authority_names):
        missing = [name for name in authority_names if name not in values.names]
        extra = [name for name in values.names if name not in authority_names]
        raise ValueError(f"joint_state_contract_mismatch missing={missing} extra={extra}")
    mapping = dict(zip(values.names, values.positions))
    ordered = tuple(float(mapping[name]) for name in authority_names)
    if not all(math.isfinite(value) for value in ordered):
        raise ValueError("joint_state_non_finite")
    return ordered


def validate_payloads(payloads: Sequence[AttachedPayload]) -> None:
    seen_sides: set[str] = set()
    for payload in payloads:
        if not payload.object_id:
            raise ValueError("attached_payload_empty_id")
        if payload.side not in ("left", "right"):
            raise ValueError(f"attached_payload_invalid_side:{payload.side}")
        if payload.side in seen_sides:
            raise ValueError(f"attached_payload_duplicate_side:{payload.side}")
        seen_sides.add(payload.side)
        expected_link = f"{payload.side}_tool0"
        if payload.link_name != expected_link:
            raise ValueError(
                f"attached_payload_link_mismatch:{payload.link_name}!={expected_link}"
            )
        values = (*payload.center_xyz, *payload.orientation_xyzw, *payload.size_xyz)
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f"attached_payload_non_finite:{payload.object_id}")
        if not all(value > 0.0 for value in payload.size_xyz):
            raise ValueError(f"attached_payload_invalid_size:{payload.object_id}")
        norm = math.sqrt(sum(value * value for value in payload.orientation_xyzw))
        if abs(norm - 1.0) > 1e-3:
            raise ValueError(f"attached_payload_invalid_orientation:{payload.object_id}")


def _validate_limits(
    positions: Sequence[float],
    limits: dict[str, tuple[float, float]],
) -> None:
    for name, value in zip(AUTHORITY_JOINT_NAMES, positions):
        if name not in limits:
            raise ValueError(f"backend_missing_joint_limit:{name}")
        lower, upper = limits[name]
        if value < lower - 1e-6 or value > upper + 1e-6:
            raise ValueError(f"joint_limit_violation:{name}={value}")


def validate_backend_trajectory(
    trajectory: BackendTrajectory,
    expected_start: Sequence[float],
    expected_goal: Sequence[float],
    limits: dict[str, tuple[float, float]],
    *,
    endpoint_tolerance: float = 1e-4,
) -> BackendTrajectory:
    names = tuple(trajectory.joint_names)
    if len(set(names)) != len(names):
        raise ValueError("trajectory_duplicate_joint")
    if set(names) != set(AUTHORITY_JOINT_NAMES):
        raise ValueError(
            "trajectory_joint_contract_mismatch "
            f"expected={AUTHORITY_JOINT_NAMES} actual={names}"
        )
    if len(trajectory.positions) < 2:
        raise ValueError("trajectory_too_short")
    if len(trajectory.times_s) != len(trajectory.positions):
        raise ValueError("trajectory_time_size_mismatch")
    if trajectory.velocities and len(trajectory.velocities) != len(trajectory.positions):
        raise ValueError("trajectory_velocity_size_mismatch")
    if trajectory.accelerations and len(trajectory.accelerations) != len(trajectory.positions):
        raise ValueError("trajectory_acceleration_size_mismatch")

    index = {name: i for i, name in enumerate(names)}
    ordered_rows: list[tuple[float, ...]] = []
    ordered_velocities: list[tuple[float, ...]] = []
    ordered_accelerations: list[tuple[float, ...]] = []
    previous_time = -math.inf
    for row_index, (row, time_s) in enumerate(zip(trajectory.positions, trajectory.times_s)):
        if len(row) != len(names) or not all(math.isfinite(value) for value in row):
            raise ValueError(f"trajectory_invalid_positions@{row_index}")
        if not math.isfinite(time_s) or (row_index > 0 and time_s <= previous_time):
            raise ValueError(f"trajectory_non_monotonic_time@{row_index}")
        previous_time = time_s
        ordered = tuple(float(row[index[name]]) for name in AUTHORITY_JOINT_NAMES)
        _validate_limits(ordered, limits)
        ordered_rows.append(ordered)
        if trajectory.velocities:
            velocity = trajectory.velocities[row_index]
            if len(velocity) != len(names) or not all(math.isfinite(v) for v in velocity):
                raise ValueError(f"trajectory_invalid_velocity@{row_index}")
            ordered_velocities.append(tuple(float(velocity[index[name]]) for name in AUTHORITY_JOINT_NAMES))
        if trajectory.accelerations:
            acceleration = trajectory.accelerations[row_index]
            if len(acceleration) != len(names) or not all(math.isfinite(v) for v in acceleration):
                raise ValueError(f"trajectory_invalid_acceleration@{row_index}")
            ordered_accelerations.append(tuple(float(acceleration[index[name]]) for name in AUTHORITY_JOINT_NAMES))

    for name, actual, expected in zip(AUTHORITY_JOINT_NAMES, ordered_rows[0], expected_start):
        if abs(actual - expected) > endpoint_tolerance:
            raise ValueError(f"trajectory_start_mismatch:{name}")
    for name, actual, expected in zip(AUTHORITY_JOINT_NAMES, ordered_rows[-1], expected_goal):
        if abs(actual - expected) > endpoint_tolerance:
            raise ValueError(f"trajectory_goal_mismatch:{name}")

    return BackendTrajectory(
        joint_names=AUTHORITY_JOINT_NAMES,
        positions=tuple(ordered_rows),
        times_s=tuple(float(value) for value in trajectory.times_s),
        velocities=tuple(ordered_velocities),
        accelerations=tuple(ordered_accelerations),
    )


class JointSegmentPlannerCore:
    """Validate, serialize, execute, and post-validate one explicit segment."""

    def __init__(
        self,
        backend: SegmentBackend,
        *,
        expected_scene_id: str = "",
        default_timeout_s: float = 0.5,
        default_max_attempts: int = 3,
    ) -> None:
        self._backend = backend
        self._expected_scene_id = expected_scene_id
        self._default_timeout_s = default_timeout_s
        self._default_max_attempts = default_max_attempts
        self._lock = threading.Lock()
        if tuple(backend.joint_names) != AUTHORITY_JOINT_NAMES:
            raise ValueError(
                "curobo_joint_contract_mismatch "
                f"expected={AUTHORITY_JOINT_NAMES} actual={tuple(backend.joint_names)}"
            )

    def plan(self, request: SegmentRequest) -> SegmentResponse:
        total_started = time.perf_counter()
        try:
            if self._expected_scene_id and request.scene_id != self._expected_scene_id:
                raise ValueError(
                    f"scene_id_mismatch:{request.scene_id}!={self._expected_scene_id}"
                )
            start = reorder_joint_values(request.start)
            goal = reorder_joint_values(request.goal)
            _validate_limits(start, self._backend.joint_limits)
            _validate_limits(goal, self._backend.joint_limits)
            validate_payloads(request.attached_payloads)
            timeout_s = request.timeout_s if request.timeout_s > 0.0 else self._default_timeout_s
            max_attempts = request.max_attempts if request.max_attempts > 0 else self._default_max_attempts
            if not math.isfinite(timeout_s) or timeout_s <= 0.0:
                raise ValueError("invalid_timeout")
        except Exception as exc:
            return SegmentResponse(
                False,
                str(exc),
                total_time_ms=(time.perf_counter() - total_started) * 1000.0,
            )

        queue_started = time.perf_counter()
        with self._lock:
            queue_time_ms = (time.perf_counter() - queue_started) * 1000.0
            if queue_time_ms > timeout_s * 1000.0:
                return SegmentResponse(
                    False,
                    "planning_timeout_in_queue",
                    total_time_ms=(time.perf_counter() - total_started) * 1000.0,
                    queue_time_ms=queue_time_ms,
                )
            remaining_timeout = max(1e-3, timeout_s - queue_time_ms / 1000.0)
            try:
                result = self._backend.plan_cspace(
                    start,
                    goal,
                    request.attached_payloads,
                    scene_id=request.scene_id,
                    force_graph=request.force_graph,
                    max_attempts=max_attempts,
                    timeout_s=remaining_timeout,
                )
            except Exception as exc:
                return SegmentResponse(
                    False,
                    f"curobo_backend_exception:{exc}",
                    total_time_ms=(time.perf_counter() - total_started) * 1000.0,
                    queue_time_ms=queue_time_ms,
                )

        total_time_ms = (time.perf_counter() - total_started) * 1000.0
        backend_timings = {
            "endpoint_check_time_ms": result.endpoint_check_time_ms,
            "graph_time_ms": result.graph_time_ms,
            "trajopt_time_ms": result.trajopt_time_ms,
            "interpolation_time_ms": result.interpolation_time_ms,
        }
        if not result.success or result.trajectory is None:
            return SegmentResponse(
                False,
                result.message or "curobo_plan_cspace_failed",
                planner_method=result.planner_method,
                total_time_ms=total_time_ms,
                solve_time_ms=result.solve_time_ms,
                queue_time_ms=queue_time_ms,
                **backend_timings,
            )
        if total_time_ms > timeout_s * 1000.0:
            return SegmentResponse(
                False,
                "curobo_plan_cspace_exceeded_timeout",
                planner_method=result.planner_method,
                total_time_ms=total_time_ms,
                solve_time_ms=result.solve_time_ms,
                queue_time_ms=queue_time_ms,
                **backend_timings,
            )
        try:
            trajectory = validate_backend_trajectory(
                result.trajectory,
                start,
                goal,
                self._backend.joint_limits,
            )
        except Exception as exc:
            return SegmentResponse(
                False,
                f"curobo_trajectory_rejected:{exc}",
                planner_method=result.planner_method,
                total_time_ms=total_time_ms,
                solve_time_ms=result.solve_time_ms,
                queue_time_ms=queue_time_ms,
                **backend_timings,
            )
        return SegmentResponse(
            True,
            "ok",
            planner_method=result.planner_method,
            trajectory=trajectory,
            total_time_ms=total_time_ms,
            solve_time_ms=result.solve_time_ms,
            queue_time_ms=queue_time_ms,
            **backend_timings,
        )
