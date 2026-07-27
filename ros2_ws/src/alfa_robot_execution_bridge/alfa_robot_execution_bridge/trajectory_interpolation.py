from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import Sequence


@dataclass(frozen=True)
class TrajectorySample:
    time_from_start: float
    positions: list[float]
    velocities: list[float] | None = None
    accelerations: list[float] | None = None


@dataclass(frozen=True)
class InterpolatedState:
    time_from_start: float
    positions: list[float]
    velocities: list[float]
    accelerations: list[float]


def _validate_sample(sample: TrajectorySample) -> None:
    joint_count = len(sample.positions)
    if joint_count == 0:
        raise ValueError("trajectory sample positions must not be empty")
    if not math.isfinite(sample.time_from_start):
        raise ValueError("trajectory sample time must be finite")
    for label, values in (
        ("positions", sample.positions),
        ("velocities", sample.velocities),
        ("accelerations", sample.accelerations),
    ):
        if values is None:
            continue
        if len(values) != joint_count:
            raise ValueError(f"trajectory sample {label} length mismatch")
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError(f"trajectory sample {label} must be finite")


def _endpoint_state(sample: TrajectorySample) -> InterpolatedState:
    joint_count = len(sample.positions)
    return InterpolatedState(
        time_from_start=float(sample.time_from_start),
        positions=[float(value) for value in sample.positions],
        velocities=(
            [float(value) for value in sample.velocities]
            if sample.velocities is not None
            else [0.0] * joint_count
        ),
        accelerations=(
            [float(value) for value in sample.accelerations]
            if sample.accelerations is not None
            else [0.0] * joint_count
        ),
    )


def _linear_state(
    start: TrajectorySample,
    goal: TrajectorySample,
    elapsed: float,
    duration: float,
) -> InterpolatedState:
    ratio = elapsed / duration
    velocities = [
        (float(goal_value) - float(start_value)) / duration
        for start_value, goal_value in zip(start.positions, goal.positions)
    ]
    return InterpolatedState(
        time_from_start=start.time_from_start + elapsed,
        positions=[
            float(start_value) + (float(goal_value) - float(start_value)) * ratio
            for start_value, goal_value in zip(start.positions, goal.positions)
        ],
        velocities=velocities,
        accelerations=[0.0] * len(start.positions),
    )


def _cubic_state(
    start: TrajectorySample,
    goal: TrajectorySample,
    elapsed: float,
    duration: float,
) -> InterpolatedState:
    assert start.velocities is not None
    assert goal.velocities is not None
    ratio = elapsed / duration
    ratio2 = ratio * ratio
    ratio3 = ratio2 * ratio
    h00 = 2.0 * ratio3 - 3.0 * ratio2 + 1.0
    h10 = ratio3 - 2.0 * ratio2 + ratio
    h01 = -2.0 * ratio3 + 3.0 * ratio2
    h11 = ratio3 - ratio2
    positions: list[float] = []
    velocities: list[float] = []
    accelerations: list[float] = []
    for start_position, start_velocity, goal_position, goal_velocity in zip(
        start.positions,
        start.velocities,
        goal.positions,
        goal.velocities,
    ):
        p0 = float(start_position)
        v0 = float(start_velocity)
        p1 = float(goal_position)
        v1 = float(goal_velocity)
        positions.append(
            h00 * p0
            + h10 * duration * v0
            + h01 * p1
            + h11 * duration * v1
        )
        velocities.append(
            ((6.0 * ratio2 - 6.0 * ratio) / duration) * p0
            + (3.0 * ratio2 - 4.0 * ratio + 1.0) * v0
            + ((-6.0 * ratio2 + 6.0 * ratio) / duration) * p1
            + (3.0 * ratio2 - 2.0 * ratio) * v1
        )
        accelerations.append(
            ((12.0 * ratio - 6.0) / (duration * duration)) * p0
            + ((6.0 * ratio - 4.0) / duration) * v0
            + ((-12.0 * ratio + 6.0) / (duration * duration)) * p1
            + ((6.0 * ratio - 2.0) / duration) * v1
        )
    return InterpolatedState(
        time_from_start=start.time_from_start + elapsed,
        positions=positions,
        velocities=velocities,
        accelerations=accelerations,
    )


def _quintic_state(
    start: TrajectorySample,
    goal: TrajectorySample,
    elapsed: float,
    duration: float,
) -> InterpolatedState:
    assert start.velocities is not None
    assert goal.velocities is not None
    assert start.accelerations is not None
    assert goal.accelerations is not None
    positions: list[float] = []
    velocities: list[float] = []
    accelerations: list[float] = []
    elapsed2 = elapsed * elapsed
    elapsed3 = elapsed2 * elapsed
    elapsed4 = elapsed3 * elapsed
    elapsed5 = elapsed4 * elapsed
    duration2 = duration * duration
    duration3 = duration2 * duration
    duration4 = duration3 * duration
    duration5 = duration4 * duration
    for p0, v0, a0, p1, v1, a1 in zip(
        start.positions,
        start.velocities,
        start.accelerations,
        goal.positions,
        goal.velocities,
        goal.accelerations,
    ):
        p0 = float(p0)
        v0 = float(v0)
        a0 = float(a0)
        p1 = float(p1)
        v1 = float(v1)
        a1 = float(a1)
        delta = p1 - p0
        c0 = p0
        c1 = v0
        c2 = 0.5 * a0
        c3 = (
            20.0 * delta
            - (8.0 * v1 + 12.0 * v0) * duration
            - (3.0 * a0 - a1) * duration2
        ) / (2.0 * duration3)
        c4 = (
            -30.0 * delta
            + (14.0 * v1 + 16.0 * v0) * duration
            + (3.0 * a0 - 2.0 * a1) * duration2
        ) / (2.0 * duration4)
        c5 = (
            12.0 * delta
            - 6.0 * (v1 + v0) * duration
            - (a0 - a1) * duration2
        ) / (2.0 * duration5)
        positions.append(
            c0 + c1 * elapsed + c2 * elapsed2 + c3 * elapsed3 + c4 * elapsed4 + c5 * elapsed5
        )
        velocities.append(
            c1
            + 2.0 * c2 * elapsed
            + 3.0 * c3 * elapsed2
            + 4.0 * c4 * elapsed3
            + 5.0 * c5 * elapsed4
        )
        accelerations.append(
            2.0 * c2 + 6.0 * c3 * elapsed + 12.0 * c4 * elapsed2 + 20.0 * c5 * elapsed3
        )
    return InterpolatedState(
        time_from_start=start.time_from_start + elapsed,
        positions=positions,
        velocities=velocities,
        accelerations=accelerations,
    )


def interpolate_between(
    start: TrajectorySample,
    goal: TrajectorySample,
    time_from_start: float,
) -> InterpolatedState:
    _validate_sample(start)
    _validate_sample(goal)
    if len(start.positions) != len(goal.positions):
        raise ValueError("trajectory endpoint joint counts differ")
    duration = float(goal.time_from_start) - float(start.time_from_start)
    if duration <= 0.0:
        raise ValueError("trajectory endpoint times must be strictly increasing")
    elapsed = min(duration, max(0.0, float(time_from_start) - start.time_from_start))
    has_velocity = start.velocities is not None and goal.velocities is not None
    has_acceleration = (
        has_velocity
        and start.accelerations is not None
        and goal.accelerations is not None
    )
    if has_acceleration:
        return _quintic_state(start, goal, elapsed, duration)
    if has_velocity:
        return _cubic_state(start, goal, elapsed, duration)
    return _linear_state(start, goal, elapsed, duration)


def sample_trajectory(
    samples: Sequence[TrajectorySample],
    time_from_start: float,
) -> InterpolatedState:
    if not samples:
        raise ValueError("trajectory samples must not be empty")
    for sample in samples:
        _validate_sample(sample)
    times = [float(sample.time_from_start) for sample in samples]
    if any(goal <= start for start, goal in zip(times, times[1:])):
        raise ValueError("trajectory sample times must be strictly increasing")
    if time_from_start <= times[0]:
        return _endpoint_state(samples[0])
    if time_from_start >= times[-1]:
        return _endpoint_state(samples[-1])
    goal_index = bisect.bisect_left(times, float(time_from_start))
    return interpolate_between(
        samples[goal_index - 1],
        samples[goal_index],
        time_from_start,
    )


def sample_fixed_rate(
    samples: Sequence[TrajectorySample],
    rate_hz: float,
) -> list[InterpolatedState]:
    if rate_hz <= 0.0 or not math.isfinite(rate_hz):
        raise ValueError("sample rate must be finite and positive")
    if not samples:
        raise ValueError("trajectory samples must not be empty")
    start_time = float(samples[0].time_from_start)
    final_time = float(samples[-1].time_from_start)
    if final_time < start_time:
        raise ValueError("trajectory final time precedes start time")
    period = 1.0 / rate_hz
    tick_count = int(math.floor((final_time - start_time) * rate_hz + 1e-9))
    result = [
        sample_trajectory(samples, start_time + tick * period)
        for tick in range(tick_count + 1)
    ]
    if not result or final_time - result[-1].time_from_start > 1e-9:
        result.append(sample_trajectory(samples, final_time))
    return result


def downsample_controller_trace(
    controller_trace: Sequence[InterpolatedState],
    output_rate_hz: float,
) -> list[InterpolatedState]:
    if output_rate_hz <= 0.0 or not math.isfinite(output_rate_hz):
        raise ValueError("output rate must be finite and positive")
    if not controller_trace:
        return []
    times = [state.time_from_start for state in controller_trace]
    start_time = times[0]
    final_time = times[-1]
    period = 1.0 / output_rate_hz
    requested_count = int(math.floor((final_time - start_time) * output_rate_hz + 1e-9))
    selected_indices: list[int] = []
    for tick in range(requested_count + 1):
        target = start_time + tick * period
        right = bisect.bisect_left(times, target)
        candidates = [index for index in (right - 1, right) if 0 <= index < len(times)]
        selected = min(candidates, key=lambda index: abs(times[index] - target))
        if not selected_indices or selected_indices[-1] != selected:
            selected_indices.append(selected)
    if selected_indices[-1] != len(controller_trace) - 1:
        selected_indices.append(len(controller_trace) - 1)
    return [controller_trace[index] for index in selected_indices]
