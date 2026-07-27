import math

from alfa_robot_execution_bridge.trajectory_interpolation import (
    TrajectorySample,
    downsample_controller_trace,
    interpolate_between,
    sample_fixed_rate,
)


def test_positions_only_use_linear_interpolation():
    start = TrajectorySample(0.0, [0.0])
    goal = TrajectorySample(2.0, [1.0])

    state = interpolate_between(start, goal, 0.5)

    assert math.isclose(state.positions[0], 0.25)
    assert math.isclose(state.velocities[0], 0.5)
    assert math.isclose(state.accelerations[0], 0.0)


def test_positions_and_velocities_use_cubic_hermite_interpolation():
    start = TrajectorySample(0.0, [0.0], [0.0])
    goal = TrajectorySample(1.0, [1.0], [0.0])

    state = interpolate_between(start, goal, 0.5)

    assert math.isclose(state.positions[0], 0.5)
    assert math.isclose(state.velocities[0], 1.5)
    assert math.isclose(state.accelerations[0], 0.0, abs_tol=1e-12)


def test_positions_velocities_and_accelerations_use_quintic_interpolation():
    start = TrajectorySample(0.0, [0.0], [0.0], [0.0])
    goal = TrajectorySample(1.0, [1.0], [0.0], [0.0])

    middle = interpolate_between(start, goal, 0.5)
    end = interpolate_between(start, goal, 1.0)

    assert math.isclose(middle.positions[0], 0.5)
    assert math.isclose(middle.velocities[0], 1.875)
    assert math.isclose(end.positions[0], 1.0)
    assert math.isclose(end.velocities[0], 0.0, abs_tol=1e-12)
    assert math.isclose(end.accelerations[0], 0.0, abs_tol=1e-12)


def test_250hz_trace_downsamples_to_nearest_controller_ticks_at_90hz():
    samples = [
        TrajectorySample(0.0, [0.0], [0.0]),
        TrajectorySample(1.0, [1.0], [0.0]),
    ]

    controller_trace = sample_fixed_rate(samples, 250.0)
    display_trace = downsample_controller_trace(controller_trace, 90.0)

    assert len(controller_trace) == 251
    assert len(display_trace) == 91
    assert display_trace[0] is controller_trace[0]
    assert display_trace[-1] is controller_trace[-1]
    assert all(
        state in controller_trace
        for state in display_trace
    )
