"""Path-preserving time-optimal parameterization for curved joint paths (MOTION-324).

The pipeline's `time_parameterize_path` gives the whole path one smoothstep clock, so a single
high-curvature region slows everything. Here TOPP-RA bounds the path speed locally under joint
velocity and acceleration limits; the bound is smoothed, integrated into knot times and fitted
with a C2 time scaling s(t). The trajectory q(t) = path(s(t)) therefore stays on the same
geometric spline as the pipeline, with analytic velocity, acceleration and jerk, and a uniform
time stretch guarantees all three limits on a fine probe.
"""
import numpy as np
from scipy.interpolate import CubicSpline


def _smooth(speed, window):
    if window <= 1:
        return speed
    kernel = np.hanning(window + 2)[1:-1]
    kernel /= kernel.sum()
    padded = np.pad(speed, window // 2, mode="edge")
    smoothed = np.convolve(padded, kernel, mode="same")[window // 2:window // 2 + len(speed)]
    return np.minimum(speed, smoothed)


def time_parameterize_toppra(path, joint_names, velocity_limits, acceleration_limits, jerk_limits,
                             sample_dt=0.025, safety_factor=1.05, acceleration_scale=.4,
                             settings=((400, 15), (200, 31), (100, 31))):
    """Same return contract as `curobo_core.trajectory.time_parameterize_path`.

    `acceleration_scale` leaves headroom for the smoothed speed profile (0.4 minimised the summed
    duration on the MOTION-324 dev tasks); `settings` are (gridpoints, smoothing window) pairs tried
    in order until the time scaling is monotonic.
    """
    errors = []
    for gridpoints, smoothing in settings:
        try:
            return _toppra(path, joint_names, velocity_limits, acceleration_limits, jerk_limits, sample_dt,
                           safety_factor, acceleration_scale, gridpoints, smoothing)
        except RuntimeError as error:
            errors.append(f"{gridpoints}/{smoothing}: {error}")
    raise RuntimeError("; ".join(errors))


def _toppra(path, joint_names, velocity_limits, acceleration_limits, jerk_limits, sample_dt, safety_factor,
            acceleration_scale, gridpoints, smoothing):
    import toppra
    import toppra.algorithm as algorithm
    import toppra.constraint as constraint

    values = np.asarray(path, dtype=np.float64)
    limits = [np.asarray(item, dtype=np.float64) for item in (velocity_limits, acceleration_limits, jerk_limits)]
    if values.ndim != 2 or len(values) < 2 or not np.isfinite(values).all():
        raise ValueError("fixed path must contain at least two finite joint rows")
    if len(joint_names) != values.shape[1] or any(
            item.shape != (values.shape[1],) or np.any(item <= 0) or not np.isfinite(item).all() for item in limits):
        raise ValueError("positive finite velocity, acceleration and jerk limits are required")
    velocity_limits, acceleration_limits, jerk_limits = limits
    keep = np.concatenate(([True], np.linalg.norm(np.diff(values, axis=0) / velocity_limits, axis=1) > 1e-12))
    values = values[keep]
    if len(values) < 2:
        raise ValueError("fixed path has no motion")
    distance = np.linalg.norm(np.diff(values, axis=0) / velocity_limits, axis=1)
    parameter = np.concatenate(([0.0], np.cumsum(distance)))
    parameter /= parameter[-1]
    zero = np.zeros(values.shape[1])
    spline = CubicSpline(parameter, values, axis=0, bc_type=((1, zero), (1, zero)))

    grid = np.linspace(0.0, 1.0, gridpoints)
    margin = 1.0 / safety_factor
    instance = algorithm.TOPPRA(
        [constraint.JointVelocityConstraint(np.column_stack([-velocity_limits, velocity_limits]) * margin),
         constraint.JointAccelerationConstraint(np.column_stack([-acceleration_limits, acceleration_limits]) * margin * acceleration_scale)],
        toppra.SplineInterpolator(parameter, values, bc_type="clamped"), gridpoints=grid)
    _, speed, _ = instance.compute_parameterization(0.0, 0.0)
    if speed is None or not np.isfinite(speed).all():
        raise RuntimeError("TOPP-RA found no feasible path parameterization")
    speed = _smooth(np.maximum(speed, 0.0), smoothing)
    knot_time = np.concatenate(([0.0], np.cumsum(2 * np.diff(grid) / np.maximum(speed[:-1] + speed[1:], 1e-9))))
    timing = CubicSpline(knot_time, grid, bc_type=((1, 0.0), (1, 0.0)))
    base = float(knot_time[-1])

    def evaluate(time, stretch):
        local = time / stretch
        s = np.clip(timing(local), 0.0, 1.0)
        s1, s2, s3 = timing(local, 1) / stretch, timing(local, 2) / stretch**2, timing(local, 3) / stretch**3
        q1, q2, q3 = spline(s, 1), spline(s, 2), spline(s, 3)
        return (s, s1, spline(s), q1 * s1[:, None], q2 * s1[:, None]**2 + q1 * s2[:, None],
                q3 * s1[:, None]**3 + 3 * q2 * (s1 * s2)[:, None] + q1 * s3[:, None])

    stretch = 1.0
    for _ in range(8):
        probe = np.linspace(0.0, base * stretch, max(4000, int(np.ceil(base * stretch / sample_dt)) * 4))
        _, s1, _, velocity, acceleration, jerk = evaluate(probe, stretch)
        if np.min(s1) < -1e-6:
            raise RuntimeError("TOPP-RA time scaling is not monotonic")
        ratio = max(float(np.max(np.abs(velocity) / velocity_limits)),
                    float(np.sqrt(np.max(np.abs(acceleration) / acceleration_limits))),
                    float(np.cbrt(np.max(np.abs(jerk) / jerk_limits))))
        if ratio <= 1.0:
            break
        stretch *= ratio * 1.01
    else:
        raise RuntimeError("TOPP-RA time scaling did not meet the limits")

    duration = base * stretch
    validation_time = np.linspace(0.0, duration, max(2, int(np.ceil(duration / sample_dt)) + 1))
    s, _, positions, velocity, acceleration, jerk = evaluate(validation_time, stretch)
    knot_time_out = np.interp(parameter, s, validation_time)
    _, _, _, knot_velocity, knot_acceleration, knot_jerk = evaluate(knot_time_out, stretch)
    linear = np.column_stack([np.interp(s, parameter, values[:, joint]) for joint in range(values.shape[1])])
    trajectory = {
        "joint_names": list(joint_names), "dt_s": None, "duration_s": duration,
        "maximum_linear_deviation": float(np.max(np.abs(positions - linear))),
        "time_from_start_s": knot_time_out.tolist(),
        "positions": values.astype(np.float32).tolist(),
        "velocities": knot_velocity.astype(np.float32).tolist(),
        "accelerations": knot_acceleration.astype(np.float32).tolist(),
        "jerks": knot_jerk.astype(np.float32).tolist(),
        "toppra": {"bound_duration_s": base, "stretch": stretch, "gridpoints": gridpoints, "smoothing": smoothing,
                   "acceleration_scale": acceleration_scale},
        "sampled_trajectory": {
            "joint_names": list(joint_names), "dt_s": None,
            "time_from_start_s": validation_time.tolist(),
            "positions": positions.astype(np.float32).tolist(),
            "velocities": velocity.astype(np.float32).tolist(),
            "accelerations": acceleration.astype(np.float32).tolist(),
            "jerks": jerk.astype(np.float32).tolist(),
        },
    }
    return trajectory, positions.astype(np.float32)
