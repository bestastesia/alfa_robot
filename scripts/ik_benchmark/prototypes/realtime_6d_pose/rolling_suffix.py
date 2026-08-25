#!/usr/bin/python3
"""PROTOTYPE pure future-suffix planning for ELECTRI-102 protocol 1.0.

Question answered by this throwaway prototype: can Motion turn a continuously
changing, collision-checked 14-axis target into an authoritative short future
whose splice position/velocity remains continuous when RT replaces it at 30 Hz?

There is deliberately no ROS I/O here.  The interactive shell and the real RT
adapter both use these same immutable trajectory values.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


AXIS_NAMES = (
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
    "updown",
)
AXIS_COUNT = len(AXIS_NAMES)
AXIS_SET_SHA256 = "25c6e82bf505ca9eb99db1c645ab75d7ecde0153faaf6a7492c6210c4d362526"
EXPECTED_LIMITS_SHA256 = "e355f72990a1c73b62d591f733cd4d2e743b78298f541dc0e92ce0ec16ccd0c4"
PROTOCOL_MAJOR = 1
PROTOCOL_MINOR = 0


@dataclass(frozen=True)
class AxisLimit:
    lower: float
    upper: float
    velocity: float
    acceleration: float
    margin: float
    stop_acceleration: float | None = None

    @property
    def stopping_acceleration(self) -> float:
        return self.acceleration if self.stop_acceleration is None else self.stop_acceleration


def _rotary(lower: float, upper: float) -> AxisLimit:
    return AxisLimit(
        lower=lower,
        upper=upper,
        velocity=0.2617993877991494,
        acceleration=0.75,
        margin=0.008726646259971648,
    )


# Snapshot of the provisional ELECTRI-102 envelope identified by
# EXPECTED_LIMITS_SHA256.  Real mode refuses a different limits hash.
PROVISIONAL_LIMITS = (
    _rotary(-math.pi / 2.0, math.pi / 2.0),
    _rotary(-math.pi / 2.0, math.pi / 2.0),
    _rotary(-2.44346095279, 2.44346095279),
    _rotary(-3.14159265, 3.14159265),
    _rotary(-2.18166156499, 2.18166156499),
    _rotary(-3.12413936107, 3.12413936107),
    _rotary(-math.pi / 2.0, math.pi / 2.0),
    _rotary(-math.pi / 2.0, math.pi / 2.0),
    _rotary(-2.44346095279, 2.44346095279),
    _rotary(-3.14159265, 3.14159265),
    _rotary(-2.18166156499, 2.18166156499),
    _rotary(-3.12413936107, 3.12413936107),
    _rotary(-3.12413936107, 3.12413936107),
    AxisLimit(lower=0.0, upper=0.8, velocity=0.09, acceleration=0.5, margin=0.005),
)

# Motion-only preview envelope.  These values mirror the current MoveIt
# joint_limits.yaml and are intentionally distinct from the provisional
# ELECTRI-102 envelope above.  A real rolling session must use the exact
# RT-Control-owned envelope identified by its negotiated limits hash.
MOTION_PREVIEW_LIMITS = tuple(
    AxisLimit(
        lower=source.lower,
        upper=0.7 if index == 13 else source.upper,
        velocity=(
            1.2
            if index % 6 in (0, 1, 2) and index < 12
            else 1.5
            if index < 12
            else 0.5
        ),
        acceleration=1.0 if index < 12 else 0.3,
        margin=source.margin,
    )
    for index, source in enumerate(PROVISIONAL_LIMITS)
)


class SuffixPlanningError(RuntimeError):
    """A target cannot be converted into a conservative admissible suffix."""


def target_tracking_allowed(*, enabled: bool, armed: bool, target_is_fresh: bool) -> bool:
    """Return whether a Motion joint target may replace current-pose HOLD."""

    return bool(enabled and armed and target_is_fresh)


def choose_replace_from_ns(
    *,
    public_replaceable_from_ns: int,
    accepted_replace_from_ns: int,
    public_state_age_ns: int,
    replace_guard_ns: int,
    horizon_ns: int,
    max_horizon_ns: int,
    replace_lead_ns: int,
    controller_period_ns: int,
) -> int:
    """Choose a future splice from an asynchronously published RT frontier.

    ``public_replaceable_from_ns`` is already old by the time Motion plans and
    publishes a batch.  Compensating for the observed state age prevents the
    RT admission frontier from overtaking a fixed guard.  The compensation is
    bounded by the negotiated maximum horizon, with one controller period
    reserved for admission jitter.
    """

    values = {
        "public_replaceable_from_ns": public_replaceable_from_ns,
        "accepted_replace_from_ns": accepted_replace_from_ns,
        "public_state_age_ns": public_state_age_ns,
        "replace_guard_ns": replace_guard_ns,
        "horizon_ns": horizon_ns,
        "max_horizon_ns": max_horizon_ns,
        "replace_lead_ns": replace_lead_ns,
        "controller_period_ns": controller_period_ns,
    }
    if any(not isinstance(value, int) or value < 0 for value in values.values()):
        raise ValueError("replace-frontier times must be non-negative integers")

    maximum_extra_ns = (
        max_horizon_ns - horizon_ns - replace_lead_ns - controller_period_ns
    )
    requested_extra_ns = public_state_age_ns + replace_guard_ns
    if maximum_extra_ns < 0 or requested_extra_ns > maximum_extra_ns:
        raise SuffixPlanningError(
            "rt_public_state_too_stale:"
            f"age_ms={public_state_age_ns * 1e-6:.3f},"
            f"guard_ms={replace_guard_ns * 1e-6:.3f},"
            f"budget_ms={max(0, maximum_extra_ns) * 1e-6:.3f}"
        )

    replace_from_ns = max(
        public_replaceable_from_ns + requested_extra_ns,
        accepted_replace_from_ns,
    )
    if replace_from_ns - public_replaceable_from_ns > maximum_extra_ns:
        raise SuffixPlanningError("accepted_suffix_exceeds_rt_horizon_budget")
    return replace_from_ns


def _vector(values: Iterable[float], label: str) -> tuple[float, ...]:
    result = tuple(float(value) for value in values)
    if len(result) != AXIS_COUNT:
        raise ValueError(f"{label} must contain exactly {AXIS_COUNT} values")
    if not all(math.isfinite(value) for value in result):
        raise ValueError(f"{label} contains NaN or Inf")
    return result


@dataclass(frozen=True)
class RollingPoint:
    time_ns: int
    positions: tuple[float, ...]
    velocities: tuple[float, ...]

    def __post_init__(self) -> None:
        if self.time_ns < 0:
            raise ValueError("time_ns must be non-negative")
        object.__setattr__(self, "positions", _vector(self.positions, "positions"))
        object.__setattr__(self, "velocities", _vector(self.velocities, "velocities"))


@dataclass(frozen=True)
class RollingSuffix:
    sequence: int
    replace_from_ns: int
    points: tuple[RollingPoint, ...]
    target_scale: float = 1.0

    def __post_init__(self) -> None:
        if self.sequence <= 0:
            raise ValueError("sequence must be positive")
        if len(self.points) < 2:
            raise ValueError("suffix requires at least two points")
        if self.points[0].time_ns != self.replace_from_ns:
            raise ValueError("first point time must equal replace_from_ns")
        if any(a.time_ns >= b.time_ns for a, b in zip(self.points, self.points[1:])):
            raise ValueError("point times must be strictly increasing")

    @property
    def buffered_until_ns(self) -> int:
        return self.points[-1].time_ns

    def sample(self, time_ns: int) -> tuple[tuple[float, ...], tuple[float, ...]]:
        if time_ns <= self.points[0].time_ns:
            return self.points[0].positions, self.points[0].velocities
        if time_ns >= self.points[-1].time_ns:
            return self.points[-1].positions, self.points[-1].velocities
        for left, right in zip(self.points, self.points[1:]):
            if time_ns <= right.time_ns:
                duration_s = (right.time_ns - left.time_ns) * 1e-9
                u = (time_ns - left.time_ns) / (right.time_ns - left.time_ns)
                positions = []
                velocities = []
                for q0, v0, q1, v1 in zip(
                    left.positions, left.velocities, right.positions, right.velocities
                ):
                    q, velocity = _hermite(q0, v0, q1, v1, duration_s, float(u))
                    positions.append(q)
                    velocities.append(velocity)
                return tuple(positions), tuple(velocities)
        raise AssertionError("unreachable suffix sample")


@dataclass(frozen=True)
class PlanDiagnostics:
    target_scale: float
    peak_velocity: float
    peak_acceleration: float
    profile_duration_s: float = 0.0
    replanned: bool = False


@dataclass(frozen=True)
class _RollingMotionProfile:
    start_ns: int
    duration_s: float
    start_positions: tuple[float, ...]
    start_velocities: tuple[float, ...]
    goal_positions: tuple[float, ...]
    active_indices: tuple[int, ...]

    def sample(self, time_ns: int) -> tuple[tuple[float, ...], tuple[float, ...]]:
        elapsed_s = max(0.0, (time_ns - self.start_ns) * 1e-9)
        positions = list(self.start_positions)
        velocities = list(self.start_velocities)
        for axis in self.active_indices:
            positions[axis], velocities[axis] = _quintic_boundary_state(
                self.start_positions[axis],
                self.start_velocities[axis],
                self.goal_positions[axis],
                self.duration_s,
                min(elapsed_s, self.duration_s),
            )
        return tuple(positions), tuple(velocities)


class RollingTargetPlanner:
    """Stateful C1 rolling planner for a changing joint-position target.

    A fixed target owns one absolute-time motion profile.  Subsequent batches
    preserve the overlapping portion of the last transmitted suffix and only
    append its future.  A materially changed target replans from the exact
    splice q/v, so a 30 Hz update does not imply a stop every 500 ms.
    """

    def __init__(
        self,
        *,
        active_indices: Sequence[int],
        limits: Sequence[AxisLimit] = PROVISIONAL_LIMITS,
        knot_ns: int = 100_000_000,
        horizon_ns: int = 500_000_000,
        limit_fraction: float = 1.0,
        target_change_tolerance_rad: float = math.radians(0.02),
        validation_step_ns: int = 20_000_000,
    ) -> None:
        self.active_indices = tuple(sorted(set(int(index) for index in active_indices)))
        if not self.active_indices or any(
            index < 0 or index >= AXIS_COUNT for index in self.active_indices
        ):
            raise ValueError("active_indices must select valid axes")
        self.limits = tuple(limits)
        if len(self.limits) != AXIS_COUNT:
            raise ValueError(f"limits must contain exactly {AXIS_COUNT} axes")
        if knot_ns <= 0 or horizon_ns < knot_ns:
            raise ValueError("horizon must contain at least one knot interval")
        if not 0.0 < limit_fraction <= 1.0:
            raise ValueError("limit_fraction must be in (0, 1]")
        self.knot_ns = int(knot_ns)
        self.horizon_ns = int(horizon_ns)
        self.limit_fraction = float(limit_fraction)
        self.target_change_tolerance_rad = float(target_change_tolerance_rad)
        self.validation_step_ns = max(1_000_000, int(validation_step_ns))
        self.profile: _RollingMotionProfile | None = None
        self.goal_positions: tuple[float, ...] | None = None

    def plan(
        self,
        *,
        accepted: RollingSuffix,
        sequence: int,
        replace_from_ns: int,
        target_positions: Sequence[float],
    ) -> tuple[RollingSuffix, PlanDiagnostics]:
        target = _vector(target_positions, "target_positions")
        splice_q, splice_v = accepted.sample(replace_from_ns)
        goal = list(splice_q)
        for axis in self.active_indices:
            limit = self.limits[axis]
            goal[axis] = min(
                max(target[axis], limit.lower + limit.margin),
                limit.upper - limit.margin,
            )
        goal_tuple = tuple(goal)
        replan = self.profile is None or self.goal_positions is None or any(
            abs(goal_tuple[axis] - self.goal_positions[axis])
            > self.target_change_tolerance_rad
            for axis in self.active_indices
        )
        if replan:
            self.profile = self._make_profile(
                replace_from_ns,
                splice_q,
                splice_v,
                goal_tuple,
            )
            self.goal_positions = goal_tuple
        assert self.profile is not None

        count = int(math.ceil(self.horizon_ns / self.knot_ns)) + 1
        points: list[RollingPoint] = []
        for knot_index in range(count):
            point_time_ns = replace_from_ns + knot_index * self.knot_ns
            if knot_index == 0:
                positions, velocities = splice_q, splice_v
            elif not replan and point_time_ns <= accepted.buffered_until_ns:
                positions, velocities = accepted.sample(point_time_ns)
            else:
                base_q, base_v = accepted.sample(point_time_ns)
                profile_q, profile_v = self.profile.sample(point_time_ns)
                positions = tuple(
                    profile_q[index] if index in self.active_indices else base_q[index]
                    for index in range(AXIS_COUNT)
                )
                velocities = tuple(
                    profile_v[index] if index in self.active_indices else base_v[index]
                    for index in range(AXIS_COUNT)
                )
            points.append(RollingPoint(point_time_ns, positions, velocities))

        candidate = RollingSuffix(
            sequence=sequence,
            replace_from_ns=replace_from_ns,
            points=tuple(points),
            target_scale=self._endpoint_progress(splice_q, goal_tuple, points[-1].positions),
        )
        valid, reason, peak_v, peak_a = validate_suffix(
            candidate,
            active_indices=self.active_indices,
            limit_fraction=self.limit_fraction,
            limits=self.limits,
        )
        if not valid:
            # A target update can shift the knot phase relative to the stored
            # profile.  Replan more slowly from the authoritative splice once.
            self.profile = self._make_profile(
                replace_from_ns,
                splice_q,
                splice_v,
                goal_tuple,
                minimum_duration_s=self.profile.duration_s * 1.2,
            )
            replan = True
            points = self._sample_new_profile(accepted, replace_from_ns, splice_q, splice_v)
            candidate = RollingSuffix(
                sequence=sequence,
                replace_from_ns=replace_from_ns,
                points=tuple(points),
                target_scale=self._endpoint_progress(
                    splice_q, goal_tuple, points[-1].positions
                ),
            )
            valid, reason, peak_v, peak_a = validate_suffix(
                candidate,
                active_indices=self.active_indices,
                limit_fraction=self.limit_fraction,
                limits=self.limits,
            )
            if not valid:
                raise SuffixPlanningError(f"continuous suffix validation failed: {reason}")
        return candidate, PlanDiagnostics(
            candidate.target_scale,
            peak_v,
            peak_a,
            profile_duration_s=self.profile.duration_s,
            replanned=replan,
        )

    def _sample_new_profile(
        self,
        accepted: RollingSuffix,
        replace_from_ns: int,
        splice_q: tuple[float, ...],
        splice_v: tuple[float, ...],
    ) -> list[RollingPoint]:
        assert self.profile is not None
        count = int(math.ceil(self.horizon_ns / self.knot_ns)) + 1
        points: list[RollingPoint] = []
        for knot_index in range(count):
            point_time_ns = replace_from_ns + knot_index * self.knot_ns
            if knot_index == 0:
                positions, velocities = splice_q, splice_v
            else:
                base_q, base_v = accepted.sample(point_time_ns)
                profile_q, profile_v = self.profile.sample(point_time_ns)
                positions = tuple(
                    profile_q[index] if index in self.active_indices else base_q[index]
                    for index in range(AXIS_COUNT)
                )
                velocities = tuple(
                    profile_v[index] if index in self.active_indices else base_v[index]
                    for index in range(AXIS_COUNT)
                )
            points.append(RollingPoint(point_time_ns, positions, velocities))
        return points

    def _make_profile(
        self,
        start_ns: int,
        start_q: tuple[float, ...],
        start_v: tuple[float, ...],
        goal: tuple[float, ...],
        *,
        minimum_duration_s: float = 0.2,
    ) -> _RollingMotionProfile:
        duration_s = max(0.2, float(minimum_duration_s))
        last_reason = "unknown"
        for _ in range(48):
            profile = _RollingMotionProfile(
                start_ns=start_ns,
                duration_s=duration_s,
                start_positions=start_q,
                start_velocities=start_v,
                goal_positions=goal,
                active_indices=self.active_indices,
            )
            valid, last_reason = self._profile_is_valid(profile)
            if valid:
                return profile
            duration_s *= 1.2
        raise SuffixPlanningError(
            f"no continuous profile within envelope; last validation={last_reason}"
        )

    def _profile_is_valid(self, profile: _RollingMotionProfile) -> tuple[bool, str]:
        duration_ns = max(
            self.validation_step_ns,
            int(math.ceil(profile.duration_s * 1e9)),
        )
        point_times = list(
            range(profile.start_ns, profile.start_ns + duration_ns, self.validation_step_ns)
        )
        final_time_ns = profile.start_ns + duration_ns
        if not point_times or point_times[-1] != final_time_ns:
            point_times.append(final_time_ns)
        points = []
        for point_time_ns in point_times:
            q, v = profile.sample(point_time_ns)
            points.append(RollingPoint(point_time_ns, q, v))
        probe = RollingSuffix(
            sequence=1,
            replace_from_ns=profile.start_ns,
            points=tuple(points),
        )
        valid, reason, _, _ = validate_suffix(
            probe,
            active_indices=self.active_indices,
            limit_fraction=self.limit_fraction,
            limits=self.limits,
        )
        return valid, reason

    def _endpoint_progress(
        self,
        start: Sequence[float],
        goal: Sequence[float],
        endpoint: Sequence[float],
    ) -> float:
        progress = []
        for axis in self.active_indices:
            distance = goal[axis] - start[axis]
            if abs(distance) > self.target_change_tolerance_rad:
                progress.append((endpoint[axis] - start[axis]) / distance)
        if not progress:
            return 1.0
        return min(1.0, max(0.0, min(progress)))


def build_hold_suffix(
    *,
    sequence: int,
    replace_from_ns: int,
    hold_positions: Sequence[float],
    knot_ns: int = 100_000_000,
    horizon_ns: int = 500_000_000,
) -> RollingSuffix:
    positions = _vector(hold_positions, "hold_positions")
    zeros = (0.0,) * AXIS_COUNT
    count = int(math.ceil(horizon_ns / knot_ns)) + 1
    points = tuple(
        RollingPoint(replace_from_ns + index * knot_ns, positions, zeros)
        for index in range(count)
    )
    return RollingSuffix(sequence, replace_from_ns, points, target_scale=0.0)


def plan_target_suffix(
    *,
    accepted: RollingSuffix,
    sequence: int,
    replace_from_ns: int,
    target_positions: Sequence[float],
    active_indices: Sequence[int],
    knot_ns: int = 100_000_000,
    horizon_ns: int = 500_000_000,
    limit_fraction: float = 0.60,
) -> tuple[RollingSuffix, PlanDiagnostics]:
    """Create a bounded replacement whose first point exactly matches accepted.

    Non-servo axes are sampled from the accepted future at every new knot.
    Servo axes follow a quintic q/v/a boundary curve.  The target displacement
    is reduced until every resulting RT Hermite segment remains within a
    conservative fraction of the provisional envelope.
    """

    if not 0.0 < limit_fraction <= 1.0:
        raise ValueError("limit_fraction must be in (0, 1]")
    target = _vector(target_positions, "target_positions")
    active = tuple(sorted(set(int(index) for index in active_indices)))
    if not active or any(index < 0 or index >= AXIS_COUNT for index in active):
        raise ValueError("active_indices must select valid axes")
    start_q, start_v = accepted.sample(replace_from_ns)
    duration_s = horizon_ns * 1e-9
    count = int(math.ceil(horizon_ns / knot_ns)) + 1
    if count < 2:
        raise ValueError("horizon must contain at least one segment")

    scale = 1.0
    last_reason = "unknown"
    for _ in range(18):
        goal = list(start_q)
        for index in active:
            limit = PROVISIONAL_LIMITS[index]
            requested = start_q[index] + scale * (target[index] - start_q[index])
            goal[index] = min(
                max(requested, limit.lower + limit.margin),
                limit.upper - limit.margin,
            )

        points: list[RollingPoint] = []
        for knot_index in range(count):
            point_time_ns = replace_from_ns + knot_index * knot_ns
            elapsed_s = min(duration_s, knot_index * knot_ns * 1e-9)
            base_q, base_v = accepted.sample(point_time_ns)
            q = list(base_q)
            velocity = list(base_v)
            for axis in active:
                q[axis], velocity[axis] = _quintic_boundary_state(
                    start_q[axis], start_v[axis], goal[axis], duration_s, elapsed_s
                )
            points.append(RollingPoint(point_time_ns, tuple(q), tuple(velocity)))

        candidate = RollingSuffix(
            sequence=sequence,
            replace_from_ns=replace_from_ns,
            points=tuple(points),
            target_scale=scale,
        )
        valid, reason, peak_v, peak_a = validate_suffix(
            candidate, active_indices=active, limit_fraction=limit_fraction
        )
        if valid:
            return candidate, PlanDiagnostics(scale, peak_v, peak_a)
        last_reason = reason
        scale *= 0.5

    raise SuffixPlanningError(
        f"no conservative suffix after target scaling; last validation={last_reason}"
    )


def validate_suffix(
    suffix: RollingSuffix,
    *,
    active_indices: Sequence[int],
    limit_fraction: float = 1.0,
    limits: Sequence[AxisLimit] = PROVISIONAL_LIMITS,
) -> tuple[bool, str, float, float]:
    envelope = tuple(limits)
    if len(envelope) != AXIS_COUNT:
        raise ValueError(f"limits must contain exactly {AXIS_COUNT} axes")
    active = set(active_indices)
    peak_velocity = 0.0
    peak_acceleration = 0.0
    for point in suffix.points:
        for axis, position in enumerate(point.positions):
            limit = envelope[axis]
            if position < limit.lower + limit.margin or position > limit.upper - limit.margin:
                return False, f"position_limit:{AXIS_NAMES[axis]}", peak_velocity, peak_acceleration
            if axis in active:
                peak_velocity = max(peak_velocity, abs(point.velocities[axis]))
                if abs(point.velocities[axis]) > limit.velocity * limit_fraction + 1e-9:
                    return False, f"velocity_limit:{AXIS_NAMES[axis]}", peak_velocity, peak_acceleration

    for left, right in zip(suffix.points, suffix.points[1:]):
        duration_s = (right.time_ns - left.time_ns) * 1e-9
        segment_extrema = []
        for axis in range(AXIS_COUNT):
            limit = envelope[axis]
            extrema = _hermite_extrema(
                left.positions[axis],
                left.velocities[axis],
                right.positions[axis],
                right.velocities[axis],
                duration_s,
            )
            segment_extrema.append(extrema)
            position_min, position_max, velocity_positive, velocity_negative, acceleration_positive, acceleration_negative = extrema
            peak_velocity = max(peak_velocity, velocity_positive, velocity_negative)
            peak_acceleration = max(
                peak_acceleration, acceleration_positive, acceleration_negative
            )
            fraction = limit_fraction if axis in active else 1.0
            if (
                position_min < limit.lower + limit.margin
                or position_max > limit.upper - limit.margin
            ):
                return False, f"interior_position:{AXIS_NAMES[axis]}", peak_velocity, peak_acceleration
            if max(velocity_positive, velocity_negative) > limit.velocity * fraction + 1e-9:
                return False, f"interior_velocity:{AXIS_NAMES[axis]}", peak_velocity, peak_acceleration
            if max(acceleration_positive, acceleration_negative) > limit.acceleration * fraction + 1e-9:
                return False, f"acceleration_limit:{AXIS_NAMES[axis]}", peak_velocity, peak_acceleration

        # Match rt-control's conservative per-segment stopping envelope.  The
        # slowest axis determines the shared stop duration, then every axis
        # must remain inside its position margin throughout that stop.
        stop_duration_s = max(
            max(velocity_positive, velocity_negative) /
            envelope[axis].stopping_acceleration
            for axis, (_, _, velocity_positive, velocity_negative, _, _) in
            enumerate(segment_extrema)
        )
        for axis, (position_min, position_max, velocity_positive, velocity_negative, _, _) in enumerate(segment_extrema):
            limit = envelope[axis]
            stop_lower = position_min - 0.5 * velocity_negative * stop_duration_s
            stop_upper = position_max + 0.5 * velocity_positive * stop_duration_s
            if (
                stop_lower < limit.lower + limit.margin
                or stop_upper > limit.upper - limit.margin
            ):
                return False, f"stopping_viability:{AXIS_NAMES[axis]}", peak_velocity, peak_acceleration
    return True, "ok", peak_velocity, peak_acceleration


def _real_roots(a: float, b: float, c: float) -> tuple[float, ...]:
    scale = max(abs(a), abs(b), abs(c))
    if not math.isfinite(scale) or scale == 0.0:
        return ()
    a, b, c = a / scale, b / scale, c / scale
    tolerance = 64.0 * float.fromhex("0x1.0000000000000p-52")
    if abs(a) <= tolerance:
        return () if abs(b) <= tolerance else (-c / b,)
    discriminant = b * b - 4.0 * a * c
    discriminant_scale = b * b + abs(4.0 * a * c)
    if discriminant < 0.0 and -discriminant <= tolerance * discriminant_scale:
        discriminant = 0.0
    if discriminant < 0.0 or not math.isfinite(discriminant):
        return ()
    square_root = math.sqrt(discriminant)
    if square_root == 0.0:
        return (-b / (2.0 * a),)
    q = -0.5 * (b + math.copysign(square_root, b))
    roots = [q / a]
    if q != 0.0:
        roots.append(c / q)
    return tuple(roots)


def _hermite_extrema(
    q0: float, v0: float, q1: float, v1: float, duration_s: float
) -> tuple[float, float, float, float, float, float]:
    h = max(duration_s, 1e-12)
    c0 = q0
    c1 = h * v0
    c2 = -3.0 * q0 - 2.0 * h * v0 + 3.0 * q1 - h * v1
    c3 = 2.0 * q0 + h * v0 - 2.0 * q1 + h * v1
    samples = [_hermite(q0, v0, q1, v1, h, 0.0), _hermite(q0, v0, q1, v1, h, 1.0)]
    for root in _real_roots(3.0 * c3, 2.0 * c2, c1):
        if -1e-14 <= root <= 1.0 + 1e-14:
            samples.append(_hermite(q0, v0, q1, v1, h, min(max(root, 0.0), 1.0)))
    velocity_samples = [samples[0][1], samples[1][1]]
    acceleration_root = _hermite_acceleration_root(q0, v0, q1, v1, h)
    if acceleration_root is not None and -1e-14 <= acceleration_root <= 1.0 + 1e-14:
        _, velocity = _hermite(
            q0, v0, q1, v1, h, min(max(acceleration_root, 0.0), 1.0)
        )
        velocity_samples.append(velocity)
    accelerations = (
        _hermite_acceleration(q0, v0, q1, v1, h, 0.0),
        _hermite_acceleration(q0, v0, q1, v1, h, 1.0),
    )
    positions = [position for position, _ in samples]
    return (
        min(positions),
        max(positions),
        max(0.0, *velocity_samples),
        max(0.0, *(-value for value in velocity_samples)),
        max(0.0, *accelerations),
        max(0.0, *(-value for value in accelerations)),
    )


def _quintic_boundary_state(
    q0: float, v0: float, q1: float, duration_s: float, elapsed_s: float
) -> tuple[float, float]:
    """Quintic with a(0)=v(T)=a(T)=0 and the supplied v(0)."""

    t = min(max(elapsed_s, 0.0), duration_s)
    duration = max(duration_s, 1e-9)
    delta = q1 - q0
    a0 = q0
    a1 = v0
    a2 = 0.0
    a3 = (20.0 * delta - 12.0 * v0 * duration) / (2.0 * duration**3)
    a4 = (-30.0 * delta + 16.0 * v0 * duration) / (2.0 * duration**4)
    a5 = (12.0 * delta - 6.0 * v0 * duration) / (2.0 * duration**5)
    position = a0 + a1 * t + a2 * t**2 + a3 * t**3 + a4 * t**4 + a5 * t**5
    velocity = a1 + 2.0 * a2 * t + 3.0 * a3 * t**2 + 4.0 * a4 * t**3 + 5.0 * a5 * t**4
    return position, velocity


def _hermite(
    q0: float, v0: float, q1: float, v1: float, duration_s: float, u: float
) -> tuple[float, float]:
    u = min(max(u, 0.0), 1.0)
    h = max(duration_s, 1e-12)
    h00 = 2.0 * u**3 - 3.0 * u**2 + 1.0
    h10 = u**3 - 2.0 * u**2 + u
    h01 = -2.0 * u**3 + 3.0 * u**2
    h11 = u**3 - u**2
    q = h00 * q0 + h10 * h * v0 + h01 * q1 + h11 * h * v1
    dq_du = (
        (6.0 * u**2 - 6.0 * u) * q0
        + (3.0 * u**2 - 4.0 * u + 1.0) * h * v0
        + (-6.0 * u**2 + 6.0 * u) * q1
        + (3.0 * u**2 - 2.0 * u) * h * v1
    )
    return q, dq_du / h


def _hermite_acceleration(
    q0: float, v0: float, q1: float, v1: float, duration_s: float, u: float
) -> float:
    h = max(duration_s, 1e-12)
    d2q_du2 = (
        (12.0 * u - 6.0) * q0
        + (6.0 * u - 4.0) * h * v0
        + (-12.0 * u + 6.0) * q1
        + (6.0 * u - 2.0) * h * v1
    )
    return d2q_du2 / (h * h)


def _hermite_acceleration_root(
    q0: float, v0: float, q1: float, v1: float, duration_s: float
) -> float | None:
    h = max(duration_s, 1e-12)
    slope = 12.0 * q0 + 6.0 * h * v0 - 12.0 * q1 + 6.0 * h * v1
    intercept = -6.0 * q0 - 4.0 * h * v0 + 6.0 * q1 - 2.0 * h * v1
    if abs(slope) < 1e-15:
        return None
    return -intercept / slope
