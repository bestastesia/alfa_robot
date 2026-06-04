"""Position-overwrite trajectory execution on top of the PLC axis driver."""

from __future__ import annotations

import time
from contextlib import nullcontext
from dataclasses import dataclass
from threading import Event
from typing import Callable, ContextManager, Iterable

from .driver import PlcDriver
from .models import AxisStatus


@dataclass(frozen=True)
class TrajectoryPoint:
    time_from_start_s: float
    positions_deg: dict[int, float]


@dataclass(frozen=True)
class TrajectoryExecutionConfig:
    velocity_limit_deg_s: float = 50.0
    acceleration_limit_deg_s2: float = 100.0
    deceleration_limit_deg_s2: float = 100.0
    emergency_deceleration_deg_s2: float = 120.0
    command_hz: float = 20.0
    feedback_hz: float = 2.0
    max_position_step_deg: float = 5.0


@dataclass(frozen=True)
class TrajectoryExecutionReport:
    axes: tuple[int, ...]
    command_count: int
    duration_s: float
    final_targets_deg: dict[int, float]
    final_statuses: tuple[AxisStatus, ...]


FeedbackCallback = Callable[[float, int, dict[int, float], dict[int, AxisStatus]], None]
CancelChecker = Callable[[], bool]


class TrajectoryCancelledError(RuntimeError):
    """Raised when a trajectory is interrupted by cancel/stop/emergency."""


class PositionOverwriteTrajectoryExecutor:
    """Execute trajectories by periodically overwriting PLC absolute targets.

    The PLC-side contract is that `ControlWord=5` stays active and every changed
    target + CommandID is treated as the latest target to follow, not as a fresh
    point-to-point motion that must stop before the next command.
    """

    def __init__(
        self,
        driver: PlcDriver,
        config: TrajectoryExecutionConfig | None = None,
        operation_lock: ContextManager | None = None,
    ):
        self.driver = driver
        self.config = config or TrajectoryExecutionConfig()
        self.operation_lock = operation_lock

    def execute(
        self,
        points: Iterable[TrajectoryPoint],
        feedback_callback: FeedbackCallback | None = None,
        stop_event: Event | None = None,
        cancel_checker: CancelChecker | None = None,
    ) -> TrajectoryExecutionReport:
        normalized = self._normalize_points(points)
        axes = tuple(sorted(normalized[0].positions_deg))
        self._validate_points(normalized, axes)

        command_period = 1.0 / self.config.command_hz
        feedback_period = 1.0 / self.config.feedback_hz if self.config.feedback_hz > 0 else None
        final_targets = dict(normalized[-1].positions_deg)

        with self._operation_context():
            if hasattr(self.driver.transport, "open"):
                self.driver.transport.open()
        try:
            with self._operation_context():
                command_ids = self.driver.prepare_move_abs_stream(
                    axes,
                    velocity=self.config.velocity_limit_deg_s,
                    acceleration=self.config.acceleration_limit_deg_s2,
                    deceleration=self.config.deceleration_limit_deg_s2,
                    emergency_deceleration=self.config.emergency_deceleration_deg_s2,
                )
            start_time = time.monotonic()
            next_tick = start_time
            next_feedback = start_time
            command_count = 0
            point_index = 0
            duration_s = normalized[-1].time_from_start_s

            while True:
                self._raise_if_cancelled(stop_event, cancel_checker)
                now = time.monotonic()
                elapsed = min(now - start_time, duration_s)
                while point_index + 1 < len(normalized) and normalized[point_index + 1].time_from_start_s <= elapsed:
                    point_index += 1
                targets = self._interpolate_targets(normalized, axes, point_index, elapsed)
                with self._operation_context():
                    self.driver.stream_move_abs_tick(targets, command_ids)
                command_count += 1
                command_ids = self.driver.next_command_ids(command_ids)

                if feedback_period is not None and (now >= next_feedback or elapsed >= duration_s):
                    with self._operation_context():
                        statuses = {status.axis: status for status in self.driver.read_axes(axes)}
                    if feedback_callback is not None:
                        feedback_callback(elapsed, command_count, targets, statuses)
                    next_feedback += feedback_period

                if elapsed >= duration_s:
                    break
                next_tick += command_period
                sleep_s = next_tick - time.monotonic()
                if sleep_s > 0:
                    if stop_event is None:
                        time.sleep(sleep_s)
                    elif stop_event.wait(sleep_s):
                        self._raise_if_cancelled(stop_event, cancel_checker)

            with self._operation_context():
                final_statuses = tuple(self.driver.read_axes(axes))
        finally:
            with self._operation_context():
                if hasattr(self.driver.transport, "close"):
                    self.driver.transport.close()

        return TrajectoryExecutionReport(
            axes=axes,
            command_count=command_count,
            duration_s=duration_s,
            final_targets_deg=final_targets,
            final_statuses=final_statuses,
        )

    def _interpolate_targets(
        self,
        points: list[TrajectoryPoint],
        axes: tuple[int, ...],
        point_index: int,
        elapsed_s: float,
    ) -> dict[int, float]:
        current = points[point_index]
        if point_index + 1 >= len(points):
            return dict(current.positions_deg)
        following = points[point_index + 1]
        span = following.time_from_start_s - current.time_from_start_s
        if span <= 0:
            return dict(following.positions_deg)
        ratio = max(0.0, min(1.0, (elapsed_s - current.time_from_start_s) / span))
        return {
            axis: current.positions_deg[axis] + (following.positions_deg[axis] - current.positions_deg[axis]) * ratio
            for axis in axes
        }

    def _normalize_points(self, points: Iterable[TrajectoryPoint]) -> list[TrajectoryPoint]:
        normalized = sorted(points, key=lambda point: point.time_from_start_s)
        if not normalized:
            raise ValueError("trajectory points must not be empty")
        if normalized[0].time_from_start_s < 0:
            raise ValueError("trajectory point time_from_start_s must be non-negative")
        return normalized

    def _validate_points(self, points: list[TrajectoryPoint], axes: tuple[int, ...]) -> None:
        if self.config.command_hz <= 0:
            raise ValueError("command_hz must be > 0")
        if self.config.velocity_limit_deg_s <= 0:
            raise ValueError("velocity_limit_deg_s must be > 0")
        self.driver._validate_active_axes(axes)
        previous = points[0]
        for point in points:
            if tuple(sorted(point.positions_deg)) != axes:
                raise ValueError("all trajectory points must contain the same axes")
            if point.time_from_start_s < previous.time_from_start_s:
                raise ValueError("trajectory point times must be sorted")
            for axis in axes:
                step = abs(point.positions_deg[axis] - previous.positions_deg[axis])
                if step > self.config.max_position_step_deg:
                    raise ValueError(
                        f"Axis{axis} position step {step:.2f} deg exceeds max_position_step_deg={self.config.max_position_step_deg:.2f}"
                    )
            previous = point

    def _operation_context(self):
        return self.operation_lock if self.operation_lock is not None else nullcontext()

    def _raise_if_cancelled(self, stop_event: Event | None, cancel_checker: CancelChecker | None) -> None:
        if stop_event is not None and stop_event.is_set():
            raise TrajectoryCancelledError("trajectory interrupted by stop event")
        if cancel_checker is not None and cancel_checker():
            raise TrajectoryCancelledError("trajectory cancelled")
