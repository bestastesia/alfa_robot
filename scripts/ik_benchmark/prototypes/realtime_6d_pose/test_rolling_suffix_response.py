#!/usr/bin/python3
"""Regression checks for the Motion rolling-response prototype."""

import math

from rolling_suffix import (
    PROVISIONAL_LIMITS,
    RollingTargetPlanner,
    build_hold_suffix,
)


def test_ten_degree_step_keeps_rolling_instead_of_stopping_each_horizon() -> None:
    rate_hz = 30.0
    period_ns = round(1e9 / rate_hz)
    guard_ns = 24_000_000
    initial = [0.0] * 14
    initial[13] = 0.3
    target = list(initial)
    target[6] = math.radians(10.0)
    suffix = build_hold_suffix(
        sequence=1,
        replace_from_ns=guard_ns,
        hold_positions=initial,
        knot_ns=100_000_000,
        horizon_ns=500_000_000,
    )
    planner = RollingTargetPlanner(
        active_indices=range(6, 12),
        limits=PROVISIONAL_LIMITS,
        knot_ns=100_000_000,
        horizon_ns=500_000_000,
        limit_fraction=1.0,
    )

    reached_s = None
    saw_nonzero_horizon_velocity = False
    final_splice_q = initial
    final_splice_v = [0.0] * 14
    for tick in range(1, 91):
        replace_from_ns = tick * period_ns + guard_ns
        splice_q, splice_v = suffix.sample(replace_from_ns)
        final_splice_q, final_splice_v = splice_q, splice_v
        suffix, _ = planner.plan(
            accepted=suffix,
            sequence=tick + 1,
            replace_from_ns=replace_from_ns,
            target_positions=target,
        )
        assert suffix.points[0].positions == splice_q
        assert suffix.points[0].velocities == splice_v
        current_deg = math.degrees(splice_q[6])
        endpoint_deg = math.degrees(suffix.points[-1].positions[6])
        if endpoint_deg < 9.0 and abs(suffix.points[-1].velocities[6]) > 1e-4:
            saw_nonzero_horizon_velocity = True
        if reached_s is None and current_deg >= 9.5:
            reached_s = tick / rate_hz

    assert saw_nonzero_horizon_velocity
    assert reached_s is not None
    assert reached_s <= 1.5
    assert abs(math.degrees(final_splice_q[6]) - 10.0) <= 0.05
    assert abs(math.degrees(final_splice_v[6])) <= 0.05


def test_continuously_dragged_target_keeps_moving_and_settles() -> None:
    rate_hz = 30.0
    period_ns = round(1e9 / rate_hz)
    guard_ns = 24_000_000
    initial = [0.0] * 14
    initial[13] = 0.3
    suffix = build_hold_suffix(
        sequence=1,
        replace_from_ns=guard_ns,
        hold_positions=initial,
        knot_ns=100_000_000,
        horizon_ns=500_000_000,
    )
    planner = RollingTargetPlanner(
        active_indices=range(6, 12),
        limits=PROVISIONAL_LIMITS,
        knot_ns=100_000_000,
        horizon_ns=500_000_000,
        limit_fraction=1.0,
    )

    current_deg_by_tick = []
    last_velocity_deg_s = 0.0
    for tick in range(1, 121):
        replace_from_ns = tick * period_ns + guard_ns
        splice_q, _ = suffix.sample(replace_from_ns)
        current_deg_by_tick.append(math.degrees(splice_q[6]))
        dragged_target_deg = min(10.0, tick / rate_hz * 10.0)
        target = list(initial)
        target[6] = math.radians(dragged_target_deg)
        suffix, _ = planner.plan(
            accepted=suffix,
            sequence=tick + 1,
            replace_from_ns=replace_from_ns,
            target_positions=target,
        )
        _, final_velocity = suffix.sample(replace_from_ns)
        last_velocity_deg_s = math.degrees(final_velocity[6])

    # The execution front must advance while the marker is still being dragged,
    # then converge after the drag stops instead of asymptotically crawling.
    assert current_deg_by_tick[29] >= 1.0
    assert current_deg_by_tick[59] >= 9.5
    assert abs(current_deg_by_tick[-1] - 10.0) <= 0.05
    assert abs(last_velocity_deg_s) <= 0.05
