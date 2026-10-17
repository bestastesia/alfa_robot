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



def deduplicate_timed_trajectory(trajectory, atol=1e-10):
    """Remove position samples the cycle assembler would drop, keeping derivatives aligned."""
    result = dict(trajectory)
    position = np.asarray(result["positions"])
    keep = [0]
    for index in range(1, len(position)):
        if not np.allclose(position[keep[-1]], position[index], atol=atol, rtol=0):
            keep.append(index)
    for field in ("positions", "velocities", "accelerations", "jerks"):
        result[field] = np.asarray(result[field])[keep].tolist()
    dt = result.get("dt_s")
    result["time_from_start_s"] = ((np.arange(len(keep)) * dt).tolist() if dt is not None else
                                     np.asarray(result["time_from_start_s"])[keep].tolist())
    return result

def joint_state_trajectory(state, joint_names, *, preserve_timing=False):
    """Serialize a single interpolated cuRobo JointState in the requested joint order."""
    source_names = list(state.joint_names)
    joint_names = list(joint_names)
    if (not source_names or len(set(source_names)) != len(source_names) or
            not joint_names or len(set(joint_names)) != len(joint_names)):
        raise ValueError("trajectory joint names must be nonempty and unique")
    indices = [source_names.index(name) for name in joint_names]

    def rows(field):
        value = getattr(state, field)
        if value is None:
            raise ValueError(f"trajectory has no {field}")
        array = value.detach().cpu().numpy()
        if (array.ndim < 2 or array.shape[-1] != len(source_names) or
                any(size != 1 for size in array.shape[:-2])):
            raise ValueError(f"trajectory {field} must describe a single joint trajectory")
        array = array.reshape(-1, len(source_names))[:, indices]
        if not len(array) or (field != "position" and array.shape != position.shape):
            raise ValueError(f"trajectory {field} shape must match nonempty positions")
        if not np.isfinite(array).all():
            raise ValueError(f"trajectory {field} contains non-finite values")
        return array

    position = rows("position")
    derivatives = [rows(field) for field in ("velocity", "acceleration", "jerk")]
    if state.dt is None:
        raise ValueError("trajectory has no dt")
    dt_values = state.dt.detach().cpu().numpy().reshape(-1)
    if (not len(dt_values) or not np.isfinite(dt_values).all() or np.any(dt_values <= 0) or
            not np.allclose(dt_values, dt_values[0], rtol=1e-6, atol=0)):
        raise ValueError("trajectory dt must be positive, finite and constant")
    dt = float(dt_values[0])
    document = {
        "joint_names": list(joint_names),
        "dt_s": dt,
        "time_from_start_s": (np.arange(len(position), dtype=np.float64) * dt).tolist(),
        "positions": position.tolist(),
        "velocities": derivatives[0].tolist(),
        "accelerations": derivatives[1].tolist(),
        "jerks": derivatives[2].tolist(),
    }
    return document if preserve_timing else deduplicate_timed_trajectory(document)


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
    validation_ds = (30*u**2 - 60*u**3 + 30*u**4) / duration
    validation_dds = (60*u - 180*u**2 + 120*u**3) / duration**2
    validation_ddds = (60 - 360*u + 360*u**2) / duration**3
    sampled_positions = spline(validation_s)
    q1, q2, q3 = spline(validation_s, 1), spline(validation_s, 2), spline(validation_s, 3)
    sampled_velocity = q1 * validation_ds[:, None]
    sampled_acceleration = q2*validation_ds[:, None]**2 + q1*validation_dds[:, None]
    sampled_jerk = (q3*validation_ds[:, None]**3 + 3*q2*validation_ds[:, None]*validation_dds[:, None]
                      + q1*validation_ddds[:, None])
    linear_validation = np.column_stack([
        np.interp(validation_s, parameter, values[:, joint]) for joint in range(values.shape[1])
    ])
    trajectory = {
        "joint_names": list(joint_names), "dt_s": None,
        "duration_s": duration,
        "maximum_linear_deviation": float(np.max(np.abs(sampled_positions - linear_validation))),
        "time_from_start_s": (knot_u * duration).tolist(),
        "positions": values.astype(np.float32).tolist(),
        "velocities": knot_velocity.astype(np.float32).tolist(),
        "accelerations": knot_acceleration.astype(np.float32).tolist(),
        "jerks": knot_jerk.astype(np.float32).tolist(),
        "sampled_trajectory": {
            "joint_names": list(joint_names), "dt_s": None,
            "time_from_start_s": validation_time.tolist(),
            "positions": sampled_positions.astype(np.float32).tolist(),
            "velocities": sampled_velocity.astype(np.float32).tolist(),
            "accelerations": sampled_acceleration.astype(np.float32).tolist(),
            "jerks": sampled_jerk.astype(np.float32).tolist(),
        },
    }
    return trajectory, sampled_positions.astype(np.float32)


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
    event_phases = {"home", "attach", "release", "base_advance", "base_reposition",
                    "base_return", "base_pick_reposition"}
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


def trajectory_fallback_path(rows, stats):
    """Use sparse planner waypoints only when they describe the same path endpoints."""
    values = np.asarray(rows)
    sparse = np.asarray(stats.get("trajectory_fallback_waypoints", ()))
    if (sparse.ndim == 2 and len(sparse) >= 2 and sparse.shape[1:] == values.shape[1:] and
            np.allclose(sparse[[0, -1]], values[[0, -1]], atol=1e-6, rtol=0)):
        return sparse
    return values


def time_parameterize_polyline(path, joint_names, velocity_limits, acceleration_limits, jerk_limits,
                               sample_dt=0.025, safety_factor=1.05):
    """Time a dense collision-checked polyline without cutting its geometric edges."""
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
    distance = np.linalg.norm(np.diff(values, axis=0) / velocity_limits, axis=1)
    keep = np.concatenate(([True], distance > 1e-12))
    values = values[keep]
    if len(values) < 2:
        raise ValueError("polyline has no motion")
    distance = np.linalg.norm(np.diff(values, axis=0) / velocity_limits, axis=1)
    parameter = np.concatenate(([0.0], np.cumsum(distance)))
    parameter /= parameter[-1]
    duration = max(sample_dt, safety_factor * 1.875 * float(np.sum(distance)))
    for _ in range(12):
        sample_time = np.linspace(0.0, duration, max(3, int(np.ceil(duration / sample_dt)) + 1))
        u = sample_time / duration
        progress = 10*u**3 - 15*u**4 + 6*u**5
        positions = np.column_stack([
            np.interp(progress, parameter, values[:, joint]) for joint in range(values.shape[1])
        ])
        derivatives = [positions]
        for _order in range(3):
            derivatives.append(np.gradient(derivatives[-1], sample_time, axis=0, edge_order=2))
        velocity, acceleration, jerk = derivatives[1:]
        ratio = max(float(np.max(np.abs(velocity) / velocity_limits)),
                    float(np.sqrt(np.max(np.abs(acceleration) / acceleration_limits))),
                    float(np.cbrt(np.max(np.abs(jerk) / jerk_limits))))
        if ratio <= 1.0:
            break
        duration *= ratio * safety_factor
    else:
        raise RuntimeError("polyline timing did not converge")
    samples = {
        "joint_names": list(joint_names), "dt_s": None,
        "time_from_start_s": sample_time.tolist(),
        "positions": positions.astype(np.float32).tolist(),
        "velocities": velocity.astype(np.float32).tolist(),
        "accelerations": acceleration.astype(np.float32).tolist(),
        "jerks": jerk.astype(np.float32).tolist(),
    }
    return ({"joint_names": list(joint_names), "dt_s": None,
             "duration_s": float(duration), "maximum_linear_deviation": 0.0,
             "time_from_start_s": sample_time.tolist(),
             "positions": positions.astype(np.float32).tolist(),
             "velocities": velocity.astype(np.float32).tolist(),
             "accelerations": acceleration.astype(np.float32).tolist(),
             "jerks": jerk.astype(np.float32).tolist(),
             "sampled_trajectory": samples}, positions.astype(np.float32))


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
    sample_times = [0.0]
    sample_velocity = [np.zeros(values.shape[1])]
    sample_acceleration = [np.zeros(values.shape[1])]
    sample_jerk = [60.0 * deltas[0] / durations[0]**3]
    for index, (start, delta, duration) in enumerate(zip(values[:-1], deltas, durations)):
        count = max(1, int(np.ceil(duration / sample_dt)))
        u = np.linspace(1.0/count, 1.0, count)
        s = 10*u**3 - 15*u**4 + 6*u**5
        validation.extend(start + s[:, None] * delta)
        sample_times.extend(times[index] + u * duration)
        sample_velocity.extend((30*u**2 - 60*u**3 + 30*u**4)[:, None] * delta / duration)
        sample_acceleration.extend((60*u - 180*u**2 + 120*u**3)[:, None] * delta / duration**2)
        sample_jerk.extend((60 - 360*u + 360*u**2)[:, None] * delta / duration**3)
    validation = np.asarray(validation, dtype=np.float32)
    samples = {
        "joint_names": list(joint_names), "dt_s": None,
        "time_from_start_s": np.asarray(sample_times).tolist(), "positions": validation.tolist(),
        "velocities": np.asarray(sample_velocity, dtype=np.float32).tolist(),
        "accelerations": np.asarray(sample_acceleration, dtype=np.float32).tolist(),
        "jerks": np.asarray(sample_jerk, dtype=np.float32).tolist(),
    }
    return ({
        "joint_names": list(joint_names), "dt_s": None,
        "duration_s": float(times[-1]), "maximum_linear_deviation": 0.0,
        "time_from_start_s": times.tolist(), "positions": values.astype(np.float32).tolist(),
        "velocities": zeros.tolist(), "accelerations": zeros.tolist(), "jerks": jerk.tolist(),
        "sampled_trajectory": samples,
    }, validation)


class TimedTrajectory:
    """Executable quintic Hermite interpolation of the baseline q/v/a/time contract.

    Unlike playback's position lerp, this preserves velocity at an arbitrary splice.
    Bernstein coefficient bounds conservatively check the entire interpolant, not
    just its knots. No retiming or zeroing derivatives is performed here.
    """

    def __init__(self, document):
        from scipy.interpolate import BPoly

        self.joint_names = tuple(document["joint_names"])
        self.times = np.array(document["time_from_start_s"], dtype=np.float64)
        self.values = np.array([document[key] for key in
                                ("positions", "velocities", "accelerations", "jerks")], dtype=np.float64)
        if (not self.joint_names or len(set(self.joint_names)) != len(self.joint_names) or
                not all(isinstance(name, str) and name for name in self.joint_names)):
            raise ValueError("trajectory joint names must be nonempty and unique")
        if (self.times.ndim != 1 or len(self.times) < 2 or self.times[0] != 0 or
                not np.isfinite(self.times).all() or np.any(np.diff(self.times) <= 0)):
            raise ValueError("segment times must start at zero and strictly increase")
        if (self.values.shape != (4, len(self.times), len(self.joint_names)) or
                not np.isfinite(self.values).all()):
            raise ValueError("finite q/v/a/jerk must match segment times and joint names")
        coefficients = document.get("bpoly_coefficients")
        if coefficients is None:
            self.spline = BPoly.from_derivatives(self.times, self.values[:3].transpose(1, 0, 2))
        else:
            coefficients = np.asarray(coefficients, dtype=np.float64)
            if (coefficients.ndim != 3 or coefficients.shape[1:] !=
                    (len(self.times)-1, len(self.joint_names)) or coefficients.shape[0] < 2 or
                    not np.isfinite(coefficients).all()):
                raise ValueError("invalid exact polynomial coefficients")
            self.spline = BPoly.construct_fast(coefficients, self.times, extrapolate=False, axis=0)
            for order in range(3):
                if not np.allclose(self.spline(self.times, nu=order), self.values[order], atol=2e-6, rtol=0):
                    raise ValueError("exact polynomial coefficients disagree with q/v/a samples")
        self.times.setflags(write=False)
        self.values.setflags(write=False)

    @property
    def duration(self):
        return float(self.times[-1])

    def sample(self, time_s):
        times = np.asarray(time_s, dtype=np.float64)
        if not np.isfinite(times).all() or np.any(times < 0) or np.any(times > self.duration):
            raise ValueError("cannot extrapolate an executable trajectory")
        scalar = times.ndim == 0
        flat = times.reshape(-1)
        output = {}
        for order, key in enumerate(("positions", "velocities", "accelerations", "jerks")):
            values = np.asarray(self.spline(times, nu=order))
            rows = values.reshape(-1, len(self.joint_names))
            # The serialized contract owns exact endpoints. B-spline evaluation
            # can turn a limit value 0 into +5e-23, which PR65 correctly treats
            # as outside a hard upper bound. Interior values remain polynomial.
            rows[np.isclose(flat, 0., atol=1e-12, rtol=0)] = self.values[order, 0]
            rows[np.isclose(flat, self.duration, atol=1e-12, rtol=0)] = self.values[order, -1]
            output[key] = rows[0] if scalar else rows.reshape(values.shape)
        return output

    def window(self, start_s, end_s):
        if not 0 <= start_s < end_s <= self.duration:
            raise ValueError("window must be contained in the executable trajectory")
        # Wall-clock arithmetic can land picoseconds beside an existing knot.
        # Snap only at representation scale; real sub-sample cuts (e.g. 14.7us)
        # are preserved by de Casteljau below.
        for name, value in (("start", start_s), ("end", end_s)):
            nearest = self.times[np.argmin(np.abs(self.times-value))]
            if abs(nearest-value) <= 1e-9:
                if name == "start": start_s = float(nearest)
                else: end_s = float(nearest)

        from scipy.interpolate import BPoly

        def split(coefficients, fraction):
            """de Casteljau split of one Bernstein interval, stable near knots."""
            fraction = float(np.clip(fraction, 0., 1.))
            table = [np.asarray(coefficients, dtype=np.float64)]
            while len(table[-1]) > 1:
                previous = table[-1]
                table.append((1-fraction)*previous[:-1] + fraction*previous[1:])
            left = np.stack([row[0] for row in table])
            right = np.stack([row[-1] for row in table[::-1]])
            return left, right

        interior = self.times[(self.times > start_s + 1e-12) & (self.times < end_s - 1e-12)]
        absolute = np.concatenate(([start_s], interior, [end_s]))
        pieces = []
        for left_x, right_x in zip(absolute[:-1], absolute[1:]):
            index = min(len(self.times)-2, max(0, int(np.searchsorted(self.times, left_x, side="right")-1)))
            interval_left, interval_right = self.times[index:index+2]
            if right_x > interval_right + 1e-12:
                raise ValueError("window partition crossed an original polynomial knot")
            start_fraction = (left_x-interval_left)/(interval_right-interval_left)
            end_fraction = (right_x-interval_left)/(interval_right-interval_left)
            _, right_piece = split(self.spline.c[:, index], start_fraction)
            relative_end = (end_fraction-start_fraction)/max(1e-300, 1-start_fraction)
            selected, _ = split(right_piece, relative_end)
            pieces.append(selected)
        relative = absolute-start_s
        coefficients = np.stack(pieces, axis=1)
        exact = BPoly.construct_fast(coefficients, relative, extrapolate=False, axis=0)
        values = {key: exact(relative, nu=order) for order, key in enumerate(
            ("positions", "velocities", "accelerations", "jerks"))}
        source_start, source_end = self.sample(start_s), self.sample(end_s)
        for key in values:
            values[key][0] = source_start[key]
            values[key][-1] = source_end[key]
        return {"joint_names": list(self.joint_names), "dt_s": None,
                "time_from_start_s": relative.tolist(), "bpoly_coefficients": coefficients.tolist(),
                **{key: value.tolist() for key, value in values.items()}}

    def suffix(self, time_s):
        return self.window(time_s, self.duration)

    def validate_limits(self, lower, upper, velocity, acceleration, jerk):
        limits = np.asarray((lower, upper, velocity, acceleration, jerk), dtype=np.float64)
        if (limits.shape != (5, len(self.joint_names)) or not np.isfinite(limits).all() or
                np.any(limits[0] >= limits[1]) or np.any(limits[2:] <= 0)):
            raise ValueError("finite ordered position bounds and positive derivative limits required")
        coefficients = self.spline.c
        # The GPU positions/limits are float32. Allow only representation-scale
        # roundoff (two relative ULPs), not a physical joint-limit relaxation.
        tolerance = 2 * np.finfo(np.float32).eps * np.maximum(1., np.abs(limits[:2]))
        if np.any(coefficients < limits[0] - tolerance[0]) or np.any(coefficients > limits[1] + tolerance[1]):
            raise ValueError("interpolated position exceeds limits")
        ratios = {}
        for order, key in enumerate(("velocity", "acceleration", "jerk"), 1):
            peak = np.max(np.abs(self.spline.derivative(order).c), axis=(0, 1))
            ratios[key] = float(np.max(peak / limits[order + 1]))
            if ratios[key] > 1.0001:
                raise ValueError(f"interpolated {key} exceeds limits: {ratios[key]:.6f}")
        return ratios


def braking_trajectory(state, joint_names, duration_s):
    """C2 deceleration candidate, NOT a safety certificate or changed task goal.

    Match the actual initial q/v/a; end at rest at the minimum-jerk free-position
    endpoint. The caller must validate position/derivative limits AND collision
    over the entire polynomial. No clamping or instantaneous velocity reset.
    """
    from scipy.interpolate import BPoly

    names = list(joint_names)
    initial = np.asarray([state[key] for key in ("positions", "velocities", "accelerations")], dtype=float)
    if (not names or len(set(names)) != len(names) or initial.shape != (3, len(names)) or
            not np.isfinite(initial).all() or not np.isfinite(duration_s) or duration_s <= 0):
        raise ValueError("braking needs finite matching q/v/a and positive duration")
    q, v, a = initial
    # Use the same representable endpoint as the float32 SDK before constructing
    # the polynomial. Never patch a checked trajectory's endpoint afterwards.
    stop = np.asarray(q + v * duration_s/2 + a * duration_s**2/12, dtype=np.float32).astype(float)
    zeros = np.zeros_like(q)
    times = np.array([0., duration_s])
    polynomial = BPoly.from_derivatives(times, [initial, [stop, zeros, zeros]])
    return {"joint_names": names, "dt_s": None, "time_from_start_s": times.tolist(),
            **{key: polynomial(times, nu=order).tolist() for order, key in enumerate(
                ("positions", "velocities", "accelerations", "jerks"))}}


def join_timed_trajectories(first, second, tolerance=1e-5):
    """Join checked q/v/a trajectories without a zero-time duplicate or retiming."""
    left, right = TimedTrajectory(first), TimedTrajectory(second)
    if left.joint_names != right.joint_names or not np.isfinite(tolerance) or tolerance <= 0:
        raise ValueError("trajectory join needs identical joints and a positive tolerance")
    end, start = left.sample(left.duration), right.sample(0.)
    if any(not np.allclose(end[key], start[key], atol=tolerance, rtol=0)
           for key in ("positions", "velocities", "accelerations")):
        raise ValueError("trajectory join must preserve position, velocity and acceleration")
    document = {"joint_names": list(left.joint_names), "dt_s": None,
                "time_from_start_s": [*left.times[:-1], *(left.duration+right.times)],
                **{key: [*first[key][:-1], *second[key]] for key in
                   ("positions", "velocities", "accelerations", "jerks")}}
    # A sub-tolerance floating-point difference at the join changes the final
    # interval slightly; recompute derivatives and require the caller to recheck.
    return TimedTrajectory(document).suffix(0.)
