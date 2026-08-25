#!/usr/bin/python3
"""Regression checks for the Motion rolling-response prototype."""

import math

from rolling_suffix import (
    PROVISIONAL_LIMITS,
    RollingTargetPlanner,
    RollingPoint,
    RollingSuffix,
    SuffixPlanningError,
    build_hold_suffix,
    choose_replace_from_ns,
    target_tracking_allowed,
    validate_suffix,
)

import pytest


def test_target_tracking_requires_external_pose_arm() -> None:
    assert not target_tracking_allowed(enabled=True, armed=False, target_is_fresh=True)
    assert not target_tracking_allowed(enabled=True, armed=True, target_is_fresh=False)
    assert not target_tracking_allowed(enabled=False, armed=True, target_is_fresh=True)
    assert target_tracking_allowed(enabled=True, armed=True, target_is_fresh=True)


def test_replace_frontier_compensates_for_public_state_age() -> None:
    assert choose_replace_from_ns(
        public_replaceable_from_ns=1_000_000_000,
        accepted_replace_from_ns=0,
        public_state_age_ns=30_000_000,
        replace_guard_ns=24_000_000,
        horizon_ns=500_000_000,
        max_horizon_ns=600_000_000,
        replace_lead_ns=16_000_000,
        controller_period_ns=4_000_000,
    ) == 1_054_000_000


def test_replace_frontier_rejects_state_age_beyond_horizon_budget() -> None:
    with pytest.raises(SuffixPlanningError, match="rt_public_state_too_stale"):
        choose_replace_from_ns(
            public_replaceable_from_ns=1_000_000_000,
            accepted_replace_from_ns=0,
            public_state_age_ns=57_000_000,
            replace_guard_ns=24_000_000,
            horizon_ns=500_000_000,
            max_horizon_ns=600_000_000,
            replace_lead_ns=16_000_000,
            controller_period_ns=4_000_000,
        )


def test_local_validation_matches_rt_stopping_viability_reject() -> None:
    positions = [0.0] * 14
    positions[13] = 0.3
    start = list(positions)
    end = list(positions)
    start[8] = 2.420
    end[8] = 2.430
    start_velocity = [0.0] * 14
    end_velocity = [0.0] * 14
    start_velocity[8] = 0.10
    end_velocity[8] = 0.10
    suffix = RollingSuffix(
        sequence=1,
        replace_from_ns=0,
        points=(
            RollingPoint(0, tuple(start), tuple(start_velocity)),
            RollingPoint(100_000_000, tuple(end), tuple(end_velocity)),
        ),
    )

    valid, reason, _, _ = validate_suffix(
        suffix,
        active_indices=range(6, 12),
        limits=PROVISIONAL_LIMITS,
    )

    assert not valid
    assert reason == "stopping_viability:left_joint3"


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
