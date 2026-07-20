"""Updown ROS command contract shared by jog and full-flow execution."""

from __future__ import annotations

import math
from collections.abc import Sequence

from alfa_robot_execution_bridge.joints import (
    logical_to_physical_updown,
    require_updown_logical_in_range,
)


UPDOWN_CONTROLLER_POSITION_LOWER_M = 0.0
UPDOWN_CONTROLLER_POSITION_UPPER_M = 0.92
UPDOWN_PROFILE_VELOCITY_UPPER_MPS = 0.2
DEFAULT_UPDOWN_VELOCITY_MPS = 0.05
DEFAULT_UPDOWN_ACCELERATION_MPS2 = 0.05
DEFAULT_UPDOWN_DECELERATION_MPS2 = 0.05
MIN_UPDOWN_PROFILE_VELOCITY_MPS = 1e-4


def require_finite_positive(value: float, label: str, upper: float | None = None) -> float:
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{label} must be finite and > 0, got {value}")
    if upper is not None and value > upper + 1e-12:
        raise ValueError(f"{label} must be <= {upper}, got {value}")
    return value


def validate_updown_command_data(data: Sequence[float]) -> tuple[float, float, float, float]:
    """Validate the production controller's fixed four-element wire contract."""
    if len(data) != 4:
        raise ValueError(
            f"updown command length must be 4, got {len(data)}; expected "
            "[position_m, velocity_mps, acceleration_mps2, deceleration_mps2]"
        )
    position_m, velocity_mps, acceleration_mps2, deceleration_mps2 = (
        float(value) for value in data
    )
    if not math.isfinite(position_m) or not (
        UPDOWN_CONTROLLER_POSITION_LOWER_M
        <= position_m
        <= UPDOWN_CONTROLLER_POSITION_UPPER_M
    ):
        raise ValueError(
            f"updown physical position_m must be finite and in "
            f"[{UPDOWN_CONTROLLER_POSITION_LOWER_M}, {UPDOWN_CONTROLLER_POSITION_UPPER_M}], "
            f"got {position_m}"
        )
    velocity_mps = require_finite_positive(
        velocity_mps,
        "updown velocity_mps",
        UPDOWN_PROFILE_VELOCITY_UPPER_MPS,
    )
    acceleration_mps2 = require_finite_positive(
        acceleration_mps2,
        "updown acceleration_mps2",
    )
    deceleration_mps2 = require_finite_positive(
        deceleration_mps2,
        "updown deceleration_mps2",
    )
    return position_m, velocity_mps, acceleration_mps2, deceleration_mps2


def make_updown_command_data(
    logical_position_m: float,
    velocity_mps: float = DEFAULT_UPDOWN_VELOCITY_MPS,
    acceleration_mps2: float = DEFAULT_UPDOWN_ACCELERATION_MPS2,
    deceleration_mps2: float = DEFAULT_UPDOWN_DECELERATION_MPS2,
) -> list[float]:
    """Build the fixed four-element command required by the CANopen controller."""
    logical_position_m = float(logical_position_m)
    require_updown_logical_in_range(logical_position_m)
    physical_position_m = logical_to_physical_updown(logical_position_m)
    return list(
        validate_updown_command_data(
            [physical_position_m, velocity_mps, acceleration_mps2, deceleration_mps2]
        )
    )


def synchronized_updown_velocity_mps(
    timed_positions: Sequence[tuple[float, float]],
    max_velocity_mps: float,
) -> float | None:
    """Choose one PP velocity so the final target arrives near the phase end.

    The caller sends only the final position target. ``None`` means the phase does
    not move updown and therefore should not publish a new command.
    """
    max_velocity_mps = require_finite_positive(
        max_velocity_mps,
        "updown max_velocity_mps",
        UPDOWN_PROFILE_VELOCITY_UPPER_MPS,
    )
    if len(timed_positions) < 2:
        return None
    start_time, start_position = timed_positions[0]
    end_time, end_position = timed_positions[-1]
    distance = abs(float(end_position) - float(start_position))
    if distance <= 1e-9:
        return None
    duration_s = float(end_time) - float(start_time)
    if not math.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError(f"updown timed move duration must be > 0, got {duration_s}")
    required_velocity = distance / duration_s
    if required_velocity > max_velocity_mps + 1e-9:
        raise ValueError(
            f"updown trajectory requires {required_velocity:.6f}m/s but configured maximum is "
            f"{max_velocity_mps:.6f}m/s"
        )
    return max(MIN_UPDOWN_PROFILE_VELOCITY_MPS, required_velocity)
