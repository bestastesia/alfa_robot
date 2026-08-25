from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

from alfa_robot_execution_bridge.trajectory_interpolation import (
    TrajectorySample,
    sample_fixed_rate,
)

from .common import MotionSample


@dataclass(frozen=True)
class ControllerTrajectoryMetrics:
    duration_s: float
    max_joint_velocity_rad_s: float
    max_joint_acceleration_rad_s2: float
    max_joint_jerk_rad_s3: float
    max_updown_velocity_m_s: float
    max_updown_acceleration_m_s2: float
    max_updown_jerk_m_s3: float


def _trajectory_joint_names(joint_names: Sequence[str]) -> list[str]:
    return ["updown", *joint_names]


def _as_trajectory_samples(
    samples: Sequence[MotionSample],
    joint_names: Sequence[str],
) -> list[TrajectorySample]:
    names = _trajectory_joint_names(joint_names)
    return [
        TrajectorySample(
            time_from_start=float(sample.time_s),
            positions=[
                float(sample.updown_m)
                if name == "updown"
                else float(sample.joints[name])
                for name in names
            ],
            velocities=[
                float(sample.updown_velocity_m_s)
                if name == "updown"
                else float(sample.joint_velocities.get(name, 0.0))
                for name in names
            ],
            accelerations=[
                float(sample.updown_acceleration_m_s2)
                if name == "updown"
                else float(sample.joint_accelerations.get(name, 0.0))
                for name in names
            ],
        )
        for sample in samples
    ]


def _source_context(
    source: Sequence[MotionSample],
    time_s: float,
    duration_s: float,
) -> dict:
    if len(source) == 1 or duration_s <= 1e-12:
        return dict(source[-1].context)
    progress = min(1.0, max(0.0, time_s / duration_s))
    index = min(len(source) - 1, int(round(progress * (len(source) - 1))))
    return dict(source[index].context)


def scale_motion_trajectory(
    samples: Sequence[MotionSample],
    joint_names: Sequence[str],
    scale: float,
) -> list[MotionSample]:
    if scale <= 0.0 or not math.isfinite(scale):
        raise ValueError("trajectory time scale must be finite and positive")
    result: list[MotionSample] = []
    for sample in samples:
        result.append(
            MotionSample(
                time_s=float(sample.time_s) * scale,
                joints={name: float(sample.joints[name]) for name in joint_names},
                updown_m=float(sample.updown_m),
                context=dict(sample.context),
                joint_velocities={
                    name: float(sample.joint_velocities.get(name, 0.0)) / scale
                    for name in joint_names
                },
                updown_velocity_m_s=float(sample.updown_velocity_m_s) / scale,
                joint_accelerations={
                    name: float(sample.joint_accelerations.get(name, 0.0))
                    / (scale * scale)
                    for name in joint_names
                },
                updown_acceleration_m_s2=float(sample.updown_acceleration_m_s2)
                / (scale * scale),
            )
        )
    return result


def resample_motion_trajectory(
    samples: Sequence[MotionSample],
    joint_names: Sequence[str],
    rate_hz: float,
) -> list[MotionSample]:
    if len(samples) < 2:
        raise ValueError("trajectory requires at least two samples")
    interpolated = sample_fixed_rate(_as_trajectory_samples(samples, joint_names), rate_hz)
    duration_s = float(interpolated[-1].time_from_start)
    result: list[MotionSample] = []
    for state in interpolated:
        context = _source_context(samples, float(state.time_from_start), duration_s)
        context["updown"] = float(state.positions[0])
        result.append(
            MotionSample(
                time_s=float(state.time_from_start),
                joints={
                    name: float(state.positions[index + 1])
                    for index, name in enumerate(joint_names)
                },
                updown_m=float(state.positions[0]),
                context=context,
                joint_velocities={
                    name: float(state.velocities[index + 1])
                    for index, name in enumerate(joint_names)
                },
                updown_velocity_m_s=float(state.velocities[0]),
                joint_accelerations={
                    name: float(state.accelerations[index + 1])
                    for index, name in enumerate(joint_names)
                },
                updown_acceleration_m_s2=float(state.accelerations[0]),
            )
        )
    return result


def controller_trajectory_metrics(
    samples: Sequence[MotionSample],
    joint_names: Sequence[str],
    *,
    controller_rate_hz: float = 250.0,
) -> ControllerTrajectoryMetrics:
    if len(samples) < 2:
        raise ValueError("trajectory requires at least two samples")
    trace = sample_fixed_rate(
        _as_trajectory_samples(samples, joint_names),
        controller_rate_hz,
    )
    max_joint_velocity = 0.0
    max_joint_acceleration = 0.0
    max_joint_jerk = 0.0
    max_updown_velocity = 0.0
    max_updown_acceleration = 0.0
    max_updown_jerk = 0.0
    previous = None
    for state in trace:
        max_updown_velocity = max(max_updown_velocity, abs(float(state.velocities[0])))
        max_updown_acceleration = max(
            max_updown_acceleration,
            abs(float(state.accelerations[0])),
        )
        for index in range(1, len(state.velocities)):
            max_joint_velocity = max(
                max_joint_velocity,
                abs(float(state.velocities[index])),
            )
            max_joint_acceleration = max(
                max_joint_acceleration,
                abs(float(state.accelerations[index])),
            )
        if previous is not None:
            dt = float(state.time_from_start) - float(previous.time_from_start)
            if dt > 1e-12:
                max_updown_jerk = max(
                    max_updown_jerk,
                    abs(float(state.accelerations[0]) - float(previous.accelerations[0]))
                    / dt,
                )
                for index in range(1, len(state.accelerations)):
                    max_joint_jerk = max(
                        max_joint_jerk,
                        abs(
                            float(state.accelerations[index])
                            - float(previous.accelerations[index])
                        )
                        / dt,
                    )
        previous = state
    return ControllerTrajectoryMetrics(
        duration_s=float(trace[-1].time_from_start),
        max_joint_velocity_rad_s=max_joint_velocity,
        max_joint_acceleration_rad_s2=max_joint_acceleration,
        max_joint_jerk_rad_s3=max_joint_jerk,
        max_updown_velocity_m_s=max_updown_velocity,
        max_updown_acceleration_m_s2=max_updown_acceleration,
        max_updown_jerk_m_s3=max_updown_jerk,
    )


def required_time_scale(
    metrics: ControllerTrajectoryMetrics,
    *,
    max_joint_velocity_rad_s: float,
    max_joint_acceleration_rad_s2: float,
    max_joint_jerk_rad_s3: float,
    max_updown_velocity_m_s: float,
    max_updown_acceleration_m_s2: float,
    max_updown_jerk_m_s3: float,
) -> float:
    limits = (
        max_joint_velocity_rad_s,
        max_joint_acceleration_rad_s2,
        max_joint_jerk_rad_s3,
        max_updown_velocity_m_s,
        max_updown_acceleration_m_s2,
        max_updown_jerk_m_s3,
    )
    if not all(math.isfinite(value) and value > 0.0 for value in limits):
        raise ValueError("trajectory limits must be finite and positive")
    return max(
        1.0,
        metrics.max_joint_velocity_rad_s / max_joint_velocity_rad_s,
        math.sqrt(
            metrics.max_joint_acceleration_rad_s2 / max_joint_acceleration_rad_s2
        ),
        (metrics.max_joint_jerk_rad_s3 / max_joint_jerk_rad_s3) ** (1.0 / 3.0),
        metrics.max_updown_velocity_m_s / max_updown_velocity_m_s,
        math.sqrt(
            metrics.max_updown_acceleration_m_s2 / max_updown_acceleration_m_s2
        ),
        (metrics.max_updown_jerk_m_s3 / max_updown_jerk_m_s3) ** (1.0 / 3.0),
    )


def controller_limited_resample(
    samples: Sequence[MotionSample],
    joint_names: Sequence[str],
    *,
    output_rate_hz: float,
    controller_rate_hz: float,
    max_joint_velocity_rad_s: float,
    max_joint_acceleration_rad_s2: float,
    max_joint_jerk_rad_s3: float,
    max_updown_velocity_m_s: float,
    max_updown_acceleration_m_s2: float,
    max_updown_jerk_m_s3: float,
    max_iterations: int = 6,
) -> tuple[list[MotionSample], ControllerTrajectoryMetrics, float]:
    if len(samples) < 2:
        raise ValueError("trajectory requires at least two samples")
    scale = 1.0
    output: list[MotionSample] = []
    metrics = controller_trajectory_metrics(
        samples,
        joint_names,
        controller_rate_hz=controller_rate_hz,
    )
    for _ in range(max_iterations):
        scaled = scale_motion_trajectory(samples, joint_names, scale)
        output = resample_motion_trajectory(scaled, joint_names, output_rate_hz)
        metrics = controller_trajectory_metrics(
            output,
            joint_names,
            controller_rate_hz=controller_rate_hz,
        )
        additional_scale = required_time_scale(
            metrics,
            max_joint_velocity_rad_s=max_joint_velocity_rad_s,
            max_joint_acceleration_rad_s2=max_joint_acceleration_rad_s2,
            max_joint_jerk_rad_s3=max_joint_jerk_rad_s3,
            max_updown_velocity_m_s=max_updown_velocity_m_s,
            max_updown_acceleration_m_s2=max_updown_acceleration_m_s2,
            max_updown_jerk_m_s3=max_updown_jerk_m_s3,
        )
        if additional_scale <= 1.001:
            for sample in output:
                sample.context["trajectory_smoothing"] = "totg_controller_limited"
                sample.context["trajectory_duration_scale"] = scale
            return output, metrics, scale
        scale *= additional_scale * 1.01
    raise RuntimeError(
        "controller-interpolated trajectory still exceeds velocity/acceleration/jerk limits"
    )
