import numpy as np


def resample_path(path, horizon, weights=None):
    """Resample a joint path by weighted arc length for a fixed TrajOpt horizon."""
    values = np.asarray(path, dtype=np.float32)
    if values.ndim != 2 or len(values) < 2 or horizon < 2 or not np.isfinite(values).all():
        raise ValueError("path must contain at least two finite joint rows and horizon must be >= 2")
    scale = np.ones(values.shape[1], dtype=np.float32) if weights is None else np.asarray(weights, dtype=np.float32)
    if scale.shape != (values.shape[1],) or not np.isfinite(scale).all() or np.any(scale <= 0):
        raise ValueError("weights must be one positive finite value per joint")
    distance = np.linalg.norm(np.diff(values, axis=0) * scale, axis=1)
    parameter = np.concatenate(([0.0], np.cumsum(distance)))
    if parameter[-1] == 0.0:
        return np.repeat(values[:1], horizon, axis=0)
    keep = np.concatenate(([True], np.diff(parameter) > 0.0))
    target = np.linspace(0.0, parameter[-1], horizon)
    return np.column_stack([
        np.interp(target, parameter[keep], values[keep, joint])
        for joint in range(values.shape[1])
    ]).astype(np.float32)


def joint_state_trajectory(state, joint_names):
    """Serialize a single interpolated cuRobo JointState in the requested joint order."""
    source_names = list(state.joint_names)
    indices = [source_names.index(name) for name in joint_names]

    def rows(field):
        value = getattr(state, field)
        if value is None:
            raise ValueError(f"trajectory has no {field}")
        array = value.detach().cpu().numpy().reshape(-1, len(source_names))[:, indices]
        if not np.isfinite(array).all():
            raise ValueError(f"trajectory {field} contains non-finite values")
        return array

    position = rows("position")
    dt_values = state.dt.detach().cpu().numpy().reshape(-1)
    if not len(dt_values) or not np.isfinite(dt_values).all() or float(dt_values[0]) <= 0:
        raise ValueError("trajectory dt must be positive and finite")
    dt = float(dt_values[0])
    return {
        "joint_names": list(joint_names),
        "dt_s": dt,
        "time_from_start_s": (np.arange(len(position), dtype=np.float64) * dt).tolist(),
        "positions": position.tolist(),
        "velocities": rows("velocity").tolist(),
        "accelerations": rows("acceleration").tolist(),
        "jerks": rows("jerk").tolist(),
    }


def expand_timed_trajectory(trajectory, target_joint_names):
    """Expand a timed trajectory to a larger named joint vector, zero-filling fixed joints."""
    source_names = trajectory["joint_names"]
    target_index = {name: index for index, name in enumerate(target_joint_names)}
    if not set(source_names) <= set(target_index):
        raise ValueError("timed trajectory contains joints absent from the target order")
    output = {key: trajectory[key] for key in ("dt_s", "time_from_start_s")}
    output["joint_names"] = list(target_joint_names)
    for field in ("positions", "velocities", "accelerations", "jerks"):
        source = np.asarray(trajectory[field], dtype=np.float32)
        expanded = np.zeros((len(source), len(target_joint_names)), dtype=np.float32)
        for source_index, name in enumerate(source_names):
            expanded[:, target_index[name]] = source[:, source_index]
        output[field] = expanded.tolist()
    return output


def time_parameterize_path(path, joint_names, velocity_limits, acceleration_limits, jerk_limits,
                           sample_dt=0.025, safety_factor=1.05):
    """Assign a smooth rest-to-rest clock to a fixed geometric path.

    The returned positions are the original path knots. ``validation_positions`` samples
    the C2 spline at ``sample_dt`` for collision validation between knots.
    """
    from scipy.interpolate import CubicSpline

    values = np.asarray(path, dtype=np.float64)
    limits = [np.asarray(item, dtype=np.float64) for item in
              (velocity_limits, acceleration_limits, jerk_limits)]
    if values.ndim != 2 or len(values) < 2 or not np.isfinite(values).all():
        raise ValueError("fixed path must contain at least two finite joint rows")
    if len(joint_names) != values.shape[1] or any(
            item.shape != (values.shape[1],) or np.any(item <= 0) or not np.isfinite(item).all()
            for item in limits):
        raise ValueError("positive finite velocity, acceleration and jerk limits are required")
    if sample_dt <= 0 or safety_factor < 1:
        raise ValueError("sample_dt must be positive and safety_factor must be >= 1")

    velocity_limits, acceleration_limits, jerk_limits = limits
    distance = np.linalg.norm(np.diff(values, axis=0) / velocity_limits, axis=1)
    keep = np.concatenate(([True], distance > 1e-12))
    values = values[keep]
    if len(values) < 2:
        zeros = np.zeros_like(values)
        return ({"joint_names": list(joint_names), "dt_s": None,
                 "time_from_start_s": [0.0], "positions": values.tolist(),
                 "velocities": zeros.tolist(), "accelerations": zeros.tolist(),
                 "jerks": zeros.tolist()}, values)
    distance = np.linalg.norm(np.diff(values, axis=0) / velocity_limits, axis=1)
    parameter = np.concatenate(([0.0], np.cumsum(distance)))
    parameter /= parameter[-1]
    spline = CubicSpline(parameter, values, axis=0, bc_type=((1, np.zeros(values.shape[1])),
                                                             (1, np.zeros(values.shape[1]))))

    probe_u = np.linspace(0.0, 1.0, max(2000, len(values) * 8))
    probe_s = 10*probe_u**3 - 15*probe_u**4 + 6*probe_u**5
    ds = 30*probe_u**2 - 60*probe_u**3 + 30*probe_u**4
    dds = 60*probe_u - 180*probe_u**2 + 120*probe_u**3
    ddds = 60 - 360*probe_u + 360*probe_u**2
    q1, q2, q3 = spline(probe_s, 1), spline(probe_s, 2), spline(probe_s, 3)
    velocity = q1 * ds[:, None]
    acceleration = q2 * ds[:, None]**2 + q1 * dds[:, None]
    jerk = (q3 * ds[:, None]**3 + 3*q2*ds[:, None]*dds[:, None]
            + q1*ddds[:, None])
    duration = safety_factor * max(
        float(np.max(np.abs(velocity) / velocity_limits)),
        float(np.sqrt(np.max(np.abs(acceleration) / acceleration_limits))),
        float(np.cbrt(np.max(np.abs(jerk) / jerk_limits))),
        sample_dt,
    )

    knot_u = np.interp(parameter, probe_s, probe_u)
    # The inverse smoothstep interpolation is approximate; verify and enlarge duration
    # until the actual knot derivatives respect the configured limits.
    for _ in range(4):
        knot_ds = (30*knot_u**2 - 60*knot_u**3 + 30*knot_u**4) / duration
        knot_dds = (60*knot_u - 180*knot_u**2 + 120*knot_u**3) / duration**2
        knot_ddds = (60 - 360*knot_u + 360*knot_u**2) / duration**3
        q1, q2, q3 = spline(parameter, 1), spline(parameter, 2), spline(parameter, 3)
        knot_velocity = q1 * knot_ds[:, None]
        knot_acceleration = q2*knot_ds[:, None]**2 + q1*knot_dds[:, None]
        knot_jerk = (q3*knot_ds[:, None]**3 + 3*q2*knot_ds[:, None]*knot_dds[:, None]
                     + q1*knot_ddds[:, None])
        ratio = max(float(np.max(np.abs(knot_velocity) / velocity_limits)),
                    float(np.sqrt(np.max(np.abs(knot_acceleration) / acceleration_limits))),
                    float(np.cbrt(np.max(np.abs(knot_jerk) / jerk_limits))))
        if ratio <= 1.0:
            break
        duration *= ratio * safety_factor

    validation_time = np.linspace(0.0, duration, max(2, int(np.ceil(duration / sample_dt)) + 1))
    u = validation_time / duration
    validation_s = 10*u**3 - 15*u**4 + 6*u**5
    linear_validation = np.column_stack([
        np.interp(validation_s, parameter, values[:, joint]) for joint in range(values.shape[1])
    ])
    trajectory = {
        "joint_names": list(joint_names), "dt_s": None,
        "duration_s": duration,
        "maximum_linear_deviation": float(np.max(np.abs(spline(validation_s) - linear_validation))),
        "time_from_start_s": (knot_u * duration).tolist(),
        "positions": values.astype(np.float32).tolist(),
        "velocities": knot_velocity.astype(np.float32).tolist(),
        "accelerations": knot_acceleration.astype(np.float32).tolist(),
        "jerks": knot_jerk.astype(np.float32).tolist(),
    }
    return trajectory, spline(validation_s).astype(np.float32)


def assemble_timed_cycle(frame_count, timed_segments):
    """Assemble aligned full-cycle time and derivative arrays from timed segments."""
    if frame_count < 1 or not timed_segments:
        raise ValueError("a nonempty cycle and at least one timed segment are required")
    dof = len(timed_segments[0]["joint_names"])
    time_values = np.zeros(frame_count, dtype=np.float64)
    derivatives = {field: np.zeros((frame_count, dof), dtype=np.float32)
                   for field in ("velocities", "accelerations", "jerks")}
    cursor_index = 0
    cursor_time = 0.0
    for segment in sorted(timed_segments, key=lambda item: item["frame_start"]):
        start, end = segment["frame_start"], segment["frame_end"]
        local_time = np.asarray(segment["time_from_start_s"], dtype=np.float64)
        if not 0 <= start <= end < frame_count or len(local_time) != end - start + 1:
            raise ValueError("timed segment does not match its frame range")
        if start > cursor_index:
            time_values[cursor_index:start] = cursor_time
        offset = time_values[start] - local_time[0] if start < cursor_index else cursor_time - local_time[0]
        time_values[start:end + 1] = offset + local_time
        for field, output in derivatives.items():
            values = np.asarray(segment[field], dtype=np.float32)
            if values.shape != (len(local_time), dof):
                raise ValueError(f"timed segment {field} shape does not match its frame range")
            output[start:end + 1] = values
        cursor_index = max(cursor_index, end + 1)
        cursor_time = max(cursor_time, float(time_values[end]))
    time_values[cursor_index:] = cursor_time
    if np.any(np.diff(time_values) < -1e-12):
        raise ValueError("assembled trajectory time moved backwards")
    return {"time_from_start_s": time_values.tolist(),
            **{field: values.tolist() for field, values in derivatives.items()},
            "motion_duration_s": float(time_values[-1])}


def validate_timed_cycle(phases, time_values, velocities, accelerations, jerks,
                         velocity_limits, acceleration_limits, jerk_limits, timed_segments):
    """Validate full-cycle timing coverage, finiteness and derivative limits."""
    time_values = np.asarray(time_values, dtype=np.float64)
    arrays = [np.asarray(item, dtype=np.float64) for item in
              (velocities, accelerations, jerks)]
    limits = [np.asarray(item, dtype=np.float64) for item in
              (velocity_limits, acceleration_limits, jerk_limits)]
    frame_count = len(phases)
    if len(time_values) != frame_count or any(len(item) != frame_count for item in arrays):
        raise ValueError("timed cycle arrays must match the phase count")
    if not np.isfinite(time_values).all() or any(not np.isfinite(item).all() for item in arrays):
        raise ValueError("timed cycle contains non-finite values")
    if np.any(np.diff(time_values) < -1e-12):
        raise ValueError("timed cycle moves backwards in time")
    ratios = [float(np.max(np.abs(value) / limit)) for value, limit in zip(arrays, limits)]
    covered = np.zeros(frame_count, dtype=bool)
    for segment in timed_segments:
        covered[segment["frame_start"]:segment["frame_end"]+1] = True
    event_phases = {"home", "attach", "release"}
    uncovered_motion = [index for index, phase in enumerate(phases)
                        if phase not in event_phases and not covered[index]]
    return {
        "success": not uncovered_motion and max(ratios) <= 1.0001,
        "velocity_limit_ratio": ratios[0],
        "acceleration_limit_ratio": ratios[1],
        "jerk_limit_ratio": ratios[2],
        "uncovered_motion_frames": uncovered_motion[:20],
        "time_monotonic": True,
    }


def time_parameterize_stops(path, joint_names, velocity_limits, acceleration_limits, jerk_limits,
                            sample_dt=0.025, safety_factor=1.05):
    """Time a collision-checked polyline with a full stop at each existing waypoint."""
    values = np.asarray(path, dtype=np.float64)
    limits = [np.asarray(item, dtype=np.float64) for item in
              (velocity_limits, acceleration_limits, jerk_limits)]
    if values.ndim != 2 or len(values) < 2 or not np.isfinite(values).all():
        raise ValueError("polyline must contain at least two finite joint rows")
    if len(joint_names) != values.shape[1] or any(
            item.shape != (values.shape[1],) or np.any(item <= 0) or not np.isfinite(item).all()
            for item in limits):
        raise ValueError("positive finite velocity, acceleration and jerk limits are required")
    velocity_limits, acceleration_limits, jerk_limits = limits
    deltas = np.diff(values, axis=0)
    durations = safety_factor * np.maximum.reduce((
        np.max(1.875 * np.abs(deltas) / velocity_limits, axis=1),
        np.sqrt(np.max((10.0 / np.sqrt(3.0)) * np.abs(deltas) / acceleration_limits, axis=1)),
        np.cbrt(np.max(60.0 * np.abs(deltas) / jerk_limits, axis=1)),
        np.full(len(deltas), sample_dt),
    ))
    times = np.concatenate(([0.0], np.cumsum(durations)))
    zeros = np.zeros_like(values, dtype=np.float32)
    jerk = np.zeros_like(values, dtype=np.float32)
    jerk[:-1] = (60.0 * deltas / durations[:, None]**3).astype(np.float32)
    jerk[-1] = jerk[-2]
    validation = [values[0]]
    for start, delta, duration in zip(values[:-1], deltas, durations):
        count = max(1, int(np.ceil(duration / sample_dt)))
        u = np.linspace(1.0/count, 1.0, count)
        s = 10*u**3 - 15*u**4 + 6*u**5
        validation.extend(start + s[:, None] * delta)
    return ({
        "joint_names": list(joint_names), "dt_s": None,
        "duration_s": float(times[-1]), "maximum_linear_deviation": 0.0,
        "time_from_start_s": times.tolist(), "positions": values.astype(np.float32).tolist(),
        "velocities": zeros.tolist(), "accelerations": zeros.tolist(), "jerks": jerk.tolist(),
    }, np.asarray(validation, dtype=np.float32))
