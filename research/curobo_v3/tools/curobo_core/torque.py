"""Torque auditing and uniform segment retiming with cuRobo CUDA RNEA."""
from __future__ import annotations

import copy
import hashlib
import json
from pathlib import Path
import time

import numpy as np


def write_dynamics_robot_config(source, urdf, asset_root, destination):
    """Point an existing cuRobo robot config at a dynamics-overlay URDF."""
    import yaml

    document = yaml.safe_load(Path(source).read_text())
    kinematics = document.get("robot_cfg", document)["kinematics"]
    kinematics["urdf_path"] = str(Path(urdf).resolve())
    kinematics["asset_root_path"] = str(Path(asset_root).resolve())
    Path(destination).write_text(yaml.safe_dump(document, sort_keys=False))


def load_joint_torque_profiles(path):
    """Expand one catalog profile per J number to left/right joint names."""
    document = json.loads(Path(path).read_text())
    groups = document["joint_groups"]
    profiles = {}
    for number in range(1, 8):
        profile = groups[str(number)]
        required = ("model", "ratio", "rated_torque_nm", "average_allowable_torque_nm",
                    "start_stop_peak_torque_nm", "instantaneous_max_torque_nm")
        if any(key not in profile for key in required):
            raise ValueError(f"incomplete torque profile for J{number}")
        limits = [float(profile[key]) for key in required[2:]]
        if not all(np.isfinite(limits)) or not 0 < limits[0] <= limits[1] <= limits[2] <= limits[3]:
            raise ValueError(f"invalid torque ordering for J{number}: {limits}")
        for side in ("left", "right"):
            profiles[f"{side}_joint{number}"] = dict(profile)
    return document, profiles


def combine_inertials(mass_a, com_a, inertia_a, mass_b, com_b, inertia_b):
    """Combine two COM inertias expressed in one link frame."""
    mass = float(mass_a + mass_b)
    center = (mass_a * np.asarray(com_a) + mass_b * np.asarray(com_b)) / mass
    tensor = np.zeros((3, 3), dtype=float)
    for item_mass, item_center, item_tensor in (
            (mass_a, np.asarray(com_a), np.asarray(inertia_a)),
            (mass_b, np.asarray(com_b), np.asarray(inertia_b))):
        delta = item_center - center
        tensor += item_tensor + item_mass * ((delta @ delta) * np.eye(3) - np.outer(delta, delta))
    return mass, center, tensor


def cuboid_inertia(mass, dimensions):
    x, y, z = np.asarray(dimensions, dtype=float)
    return np.diag(mass / 12.0 * np.array([y*y + z*z, x*x + z*z, x*x + y*y]))


def scale_derivatives(trajectory, factor):
    """Uniformly retime a trajectory without changing positions."""
    if factor < 1:
        raise ValueError("retiming factor must be >= 1")
    result = copy.deepcopy(trajectory)
    duration = result.get("duration_s", result["time_from_start_s"][-1])
    result["time_from_start_s"] = (np.asarray(result["time_from_start_s"]) * factor).tolist()
    result["duration_s"] = float(duration * factor)
    for field, power in (("velocities", 1), ("accelerations", 2), ("jerks", 3)):
        result[field] = (np.asarray(result[field]) / factor**power).tolist()
    return result


def _time_warp(segment, slowdown):
    """Retain q samples while applying a smooth, non-uniform time dilation."""
    result = copy.deepcopy(segment)
    time_values = np.asarray(segment["time_from_start_s"], dtype=np.float64)
    slowdown = np.asarray(slowdown, dtype=np.float64)
    if (time_values.ndim != 1 or len(time_values) < 3 or slowdown.shape != time_values.shape or
            np.any(np.diff(time_values) <= 0) or np.any(slowdown < 1) or
            not np.isfinite(slowdown).all()):
        raise ValueError("local retiming requires finite increasing samples and slowdown >= 1")
    dt = np.diff(time_values)
    warped_time = np.concatenate(([0.0], np.cumsum(dt * (slowdown[:-1] + slowdown[1:]) * .5)))
    inverse = 1.0 / slowdown
    inverse_t = np.gradient(inverse, time_values, edge_order=2)
    inverse_tt = np.gradient(inverse_t, time_values, edge_order=2)
    velocity = np.asarray(segment["velocities"], dtype=np.float64)
    acceleration = np.asarray(segment["accelerations"], dtype=np.float64)
    jerk = np.asarray(segment["jerks"], dtype=np.float64)
    a = inverse[:, None]
    at = inverse_t[:, None]
    att = inverse_tt[:, None]
    result["dt_s"] = None
    result["time_from_start_s"] = warped_time.tolist()
    result["duration_s"] = float(warped_time[-1])
    result["velocities"] = (velocity * a).tolist()
    result["accelerations"] = (acceleration * a**2 + velocity * a * at).tolist()
    result["jerks"] = (jerk * a**3 + 3 * acceleration * a**2 * at +
                       velocity * (a * at**2 + a**2 * att)).tolist()
    return result


def _violation_windows(times, mask, padding_s):
    indices = np.flatnonzero(mask)
    if not len(indices):
        return []
    groups = np.split(indices, np.flatnonzero(np.diff(indices) > 1) + 1)
    windows = []
    for group in groups:
        core_start, core_end = float(times[group[0]]), float(times[group[-1]])
        item = [max(float(times[0]), core_start - padding_s), core_start, core_end,
                min(float(times[-1]), core_end + padding_s)]
        if windows and item[0] <= windows[-1][3]:
            windows[-1][1] = min(windows[-1][1], item[1])
            windows[-1][2] = max(windows[-1][2], item[2])
            windows[-1][3] = max(windows[-1][3], item[3])
        else:
            windows.append(item)
    return windows


def _slowdown_profile(times, windows, factor):
    profile = np.ones(len(times), dtype=np.float64)
    for left, core_start, core_end, right in windows:
        bump = np.zeros(len(times), dtype=np.float64)
        bump[(times >= core_start) & (times <= core_end)] = 1.0
        if core_start > left:
            mask = (times >= left) & (times < core_start)
            x = (times[mask] - left) / (core_start - left)
            bump[mask] = 6*x**5 - 15*x**4 + 10*x**3
        if right > core_end:
            mask = (times > core_end) & (times <= right)
            x = (right - times[mask]) / (right - core_end)
            bump[mask] = 6*x**5 - 15*x**4 + 10*x**3
        profile = np.maximum(profile, 1.0 + (factor - 1.0) * bump)
    return profile


class TorqueModel:
    """Compute and constrain arm torque using a dynamics-overlay cuRobo model."""
    def __init__(self, robot_yaml, payload_mass_kg=0.0, payload_dimensions=(.3, .4, .4),
                 torque_limit_scale=.9, joint_torque_profile=None):
        dimensions = np.asarray(payload_dimensions, dtype=float)
        if (payload_mass_kg < 0 or not 0 < torque_limit_scale <= 1 or
                dimensions.shape != (3,) or np.any(dimensions <= 0) or not np.isfinite(dimensions).all()):
            raise ValueError("payload mass must be nonnegative; size and torque scale must be positive finite values")
        import torch
        from curobo._src.robot.kinematics.kinematics_cfg import KinematicsCfg
        from curobo._src.types.device_cfg import DeviceCfg
        self.torch = torch
        self.device = DeviceCfg()
        self.base_kp = KinematicsCfg.from_robot_yaml_file(str(robot_yaml), device_cfg=self.device).kinematics_config
        self.names = list(self.base_kp.joint_names)
        limits = self.base_kp.joint_limits
        self.urdf_effort = limits.effort[1].detach().cpu().numpy()
        self.effort = self.urdf_effort.copy()
        self.rated_effort = self.urdf_effort.copy()
        self.start_stop_effort = self.urdf_effort.copy()
        self.instantaneous_effort = self.urdf_effort.copy()
        self.joint_models = [None] * len(self.names)
        self.torque_profile_source = None
        if joint_torque_profile is not None:
            document, profiles = load_joint_torque_profiles(joint_torque_profile)
            self.torque_profile_source = document["source"]
            for index, name in enumerate(self.names):
                if name not in profiles:
                    continue
                profile = profiles[name]
                self.joint_models[index] = profile["model"]
                self.rated_effort[index] = profile["rated_torque_nm"]
                self.effort[index] = profile["average_allowable_torque_nm"]
                self.start_stop_effort[index] = profile["start_stop_peak_torque_nm"]
                self.instantaneous_effort[index] = profile["instantaneous_max_torque_nm"]
        self.enforced = np.asarray([name.startswith(("left_joint", "right_joint")) for name in self.names])
        self.payload_mass_kg = float(payload_mass_kg)
        self.payload_dimensions = dimensions
        self.torque_limit_scale = float(torque_limit_scale)
        self._models = {}

    @staticmethod
    def _tensor6(tensor):
        return np.asarray([tensor[0,0], tensor[1,1], tensor[2,2], tensor[0,1], tensor[0,2], tensor[1,2]])

    @staticmethod
    def _matrix6(values):
        values = np.asarray(values)
        return np.asarray([[values[0], values[3], values[4]],
                           [values[3], values[1], values[5]],
                           [values[4], values[5], values[2]]])

    def _create(self, active_sides, tool_to_box):
        from curobo._src.robot.dynamics.dynamics import Dynamics
        from curobo._src.robot.dynamics.dynamics_cfg import DynamicsCfg
        kp = self.base_kp.clone()
        if self.payload_mass_kg:
            payload_tensor = cuboid_inertia(self.payload_mass_kg, self.payload_dimensions)
            for side in active_sides:
                name = side + "_tool0"
                values = kp.get_link_masses_com(name).detach().cpu().numpy()
                tensor = self._matrix6(kp.get_link_inertia(name).detach().cpu().numpy())
                transform = np.asarray(tool_to_box[side], dtype=float)
                added_tensor = transform[:3,:3] @ payload_tensor @ transform[:3,:3].T
                mass, center, combined = combine_inertials(
                    values[3], values[:3], tensor, self.payload_mass_kg, transform[:3,3], added_tensor)
                kp.update_link_mass(name, mass)
                kp.update_link_com(name, self.torch.tensor(center, device=self.device.device, dtype=self.torch.float32))
                kp.update_link_inertia(name, self.torch.tensor(
                    self._tensor6(combined), device=self.device.device, dtype=self.torch.float32))
        dynamics = Dynamics(DynamicsCfg(kp, self.device))
        return dynamics

    def _model(self, active_sides, tool_to_box):
        sides = tuple(sorted(active_sides))
        transforms = tuple((side, np.ascontiguousarray(tool_to_box[side], dtype=np.float64).tobytes())
                           for side in sides) if self.payload_mass_kg else ()
        key = sides, transforms
        if key not in self._models:
            self._models[key] = self._create(sides, tool_to_box)
        return self._models[key]

    def torque(self, positions, velocities, accelerations, active_sides=(), tool_to_box=None):
        from curobo._src.state.state_joint import JointState
        arrays = [np.ascontiguousarray(value, dtype=np.float32) for value in
                  (positions, velocities, accelerations)]
        dynamics = self._model(active_sides, tool_to_box or {})
        dynamics.setup_batch_size(len(arrays[0]))
        tensors = [self.torch.tensor(value, device=self.device.device) for value in arrays]
        started = time.perf_counter()
        output = dynamics.compute_inverse_dynamics(JointState(
            position=tensors[0], velocity=tensors[1], acceleration=tensors[2]))
        self.torch.cuda.synchronize()
        return output.detach().cpu().numpy(), (time.perf_counter() - started) * 1000

    def audit(self, segment, active_sides=(), tool_to_box=None):
        source_names = list(segment.get("joint_names", self.names))
        indices = [source_names.index(name) for name in self.names]
        q = np.ascontiguousarray(np.asarray(segment["positions"])[:, indices])
        qd = np.ascontiguousarray(np.asarray(segment["velocities"])[:, indices])
        qdd = np.ascontiguousarray(np.asarray(segment["accelerations"])[:, indices])
        torque, wall_ms = self.torque(q, qd, qdd, active_sides, tool_to_box)
        gravity, gravity_ms = self.torque(q, np.zeros_like(qd), np.zeros_like(qdd),
                                          active_sides, tool_to_box)
        absolute = np.abs(torque)
        gravity_absolute = np.abs(gravity)
        dynamic_absolute = np.abs(torque - gravity)
        ratio = absolute / self.effort
        gravity_ratio = gravity_absolute / self.effort
        rated_ratio = absolute / self.rated_effort
        start_stop_ratio = absolute / self.start_stop_effort
        instantaneous_ratio = absolute / self.instantaneous_effort
        enforced_ratio = ratio[:, self.enforced]
        index = np.unravel_index(np.argmax(enforced_ratio), enforced_ratio.shape)
        joint_index = np.flatnonzero(self.enforced)[index[1]]
        return {
            "maximum_utilization": float(enforced_ratio[index]),
            "maximum_gravity_utilization": float(np.max(gravity_ratio[:, self.enforced])),
            "peak_joint": self.names[joint_index], "peak_frame": int(index[0]),
            "peak_torque_nm": float(abs(torque[index[0], joint_index])),
            "effort_limit_nm": float(self.effort[joint_index]),
            "effort_limit_semantics": ("manufacturer average allowable output torque"
                                       if self.torque_profile_source else "URDF effort"),
            "per_joint_peak_nm": np.max(absolute, axis=0).tolist(),
            "per_joint_rated_limit_nm": self.rated_effort.tolist(),
            "per_joint_average_limit_nm": self.effort.tolist(),
            "per_joint_start_stop_limit_nm": self.start_stop_effort.tolist(),
            "per_joint_instantaneous_limit_nm": self.instantaneous_effort.tolist(),
            "per_joint_gravity_peak_nm": np.max(gravity_absolute, axis=0).tolist(),
            "per_joint_dynamic_peak_nm": np.max(dynamic_absolute, axis=0).tolist(),
            "per_joint_utilization": np.max(ratio, axis=0).tolist(),
            "per_joint_rated_utilization": np.max(rated_ratio, axis=0).tolist(),
            "per_joint_start_stop_utilization": np.max(start_stop_ratio, axis=0).tolist(),
            "per_joint_instantaneous_utilization": np.max(instantaneous_ratio, axis=0).tolist(),
            "per_joint_gravity_utilization": np.max(gravity_ratio, axis=0).tolist(),
            "frame_maximum_utilization": np.max(ratio[:, self.enforced], axis=1).tolist(),
            "frame_maximum_gravity_utilization": np.max(gravity_ratio[:, self.enforced], axis=1).tolist(),
            "compute_ms": wall_ms + gravity_ms,
            "updown_peak_force_n": float(np.max(np.abs(torque[:, self.names.index("updown")]))),
            "updown_effort_not_enforced": True,
        }

    def _derivative_ratio(self, segment):
        source_names = list(segment.get("joint_names", self.names))
        indices = [source_names.index(name) for name in self.names]
        limits = self.base_kp.joint_limits
        ratios = []
        for field, values in (("velocities", limits.velocity[1]),
                              ("accelerations", limits.acceleration[1]),
                              ("jerks", limits.jerk[1])):
            array = np.asarray(segment[field], dtype=np.float64)[:, indices]
            ratios.append(float(np.max(np.abs(array) / values.detach().cpu().numpy())))
        return max(ratios)

    def _uniform_constraint(self, segment, before, active_sides, tool_to_box, deadline):
        lower, upper = 1.0, 2.0
        while upper < 128:
            if deadline is not None and time.perf_counter() > deadline:
                return None, {"success": False, "reason": "dynamics retiming budget exceeded"}
            candidate = scale_derivatives(segment, upper)
            if self.audit(candidate, active_sides, tool_to_box)["maximum_utilization"] <= self.torque_limit_scale:
                break
            upper *= 2
        else:
            return None, {"success": False, "reason": "torque retiming did not converge"}
        for _ in range(12):
            if deadline is not None and time.perf_counter() > deadline:
                return None, {"success": False, "reason": "dynamics retiming budget exceeded"}
            middle = (lower + upper) / 2
            candidate = scale_derivatives(segment, middle)
            if self.audit(candidate, active_sides, tool_to_box)["maximum_utilization"] <= self.torque_limit_scale:
                upper = middle
            else:
                lower = middle
        result = scale_derivatives(segment, upper)
        after = self.audit(result, active_sides, tool_to_box)
        return result, {"success": True, "retiming_mode": "uniform_fallback",
                        "time_scale": upper, "time_scale_peak": upper,
                        "retimed_windows": [], "after": after,
                        "duration_before_s": float(segment["time_from_start_s"][-1]),
                        "duration_after_s": float(result["time_from_start_s"][-1])}

    def constrain(self, segment, active_sides=(), tool_to_box=None, deadline=None):
        before = self.audit(segment, active_sides, tool_to_box)
        if before["maximum_gravity_utilization"] > self.torque_limit_scale:
            return None, before, {"success": False, "reason": "static gravity torque exceeds limit"}
        if before["maximum_utilization"] <= self.torque_limit_scale:
            return segment, before, {"success": True, "retiming_mode": "none",
                                     "time_scale": 1.0, "time_scale_peak": 1.0,
                                     "retimed_windows": [], "after": before,
                                     "duration_before_s": float(segment["time_from_start_s"][-1]),
                                     "duration_after_s": float(segment["time_from_start_s"][-1])}
        times = np.asarray(segment["time_from_start_s"], dtype=np.float64)
        if len(times) < 3:
            result, constraint = self._uniform_constraint(
                segment, before, active_sides, tool_to_box, deadline)
            return result, before, constraint
        violation = np.asarray(before["frame_maximum_utilization"]) > self.torque_limit_scale
        for expansion in range(3):
            windows = _violation_windows(times, violation, .25 * (expansion + 1))
            lower, upper = 1.0, 2.0
            accepted = None
            while upper <= 128:
                if deadline is not None and time.perf_counter() > deadline:
                    return None, before, {"success": False, "reason": "dynamics retiming budget exceeded"}
                candidate = _time_warp(segment, _slowdown_profile(times, windows, upper))
                audit = self.audit(candidate, active_sides, tool_to_box)
                if (audit["maximum_utilization"] <= self.torque_limit_scale and
                        self._derivative_ratio(candidate) <= 1.0001):
                    accepted = (candidate, audit)
                    break
                upper *= 2
            if accepted is None:
                continue
            for _ in range(12):
                if deadline is not None and time.perf_counter() > deadline:
                    return None, before, {"success": False, "reason": "dynamics retiming budget exceeded"}
                middle = (lower + upper) / 2
                candidate = _time_warp(segment, _slowdown_profile(times, windows, middle))
                audit = self.audit(candidate, active_sides, tool_to_box)
                if (audit["maximum_utilization"] <= self.torque_limit_scale and
                        self._derivative_ratio(candidate) <= 1.0001):
                    upper, accepted = middle, (candidate, audit)
                else:
                    lower = middle
            result, after = accepted
            return result, before, {"success": True, "retiming_mode": "local_window",
                                    "time_scale": upper, "time_scale_peak": upper,
                                    "retimed_windows": [{"start_s": item[0], "core_start_s": item[1],
                                                         "core_end_s": item[2], "end_s": item[3]}
                                                        for item in windows],
                                    "after": after,
                                    "duration_before_s": float(times[-1]),
                                    "duration_after_s": float(result["time_from_start_s"][-1])}
        result, constraint = self._uniform_constraint(
            segment, before, active_sides, tool_to_box, deadline)
        return result, before, constraint



def geometry_digest(positions):
    values = np.ascontiguousarray(positions, dtype=np.float32)
    return hashlib.sha256(values.tobytes()).hexdigest()


def trajectory_identity_digest(result):
    identity = {
        "model_id": result.get("model_id"),
        "scene_id": result.get("scene_id"),
        "scene_revision": result.get("scene_revision"),
        "task": result.get("task"),
        "tool_to_box": result.get("tool_to_box"),
        "joint_names": result.get("joint_names"),
        "payload_sha256": hashlib.sha256(
            np.ascontiguousarray(result.get("payload", ()), dtype=np.uint8).tobytes()).hexdigest(),
    }
    encoded = json.dumps(identity, sort_keys=True,
                         default=lambda value: np.asarray(value).tolist()).encode()
    return hashlib.sha256(encoded).hexdigest()


def constrain_timed_result(result, model, copy_result=True, retime_budget_s=.5,
                           require_contract=True):
    """Apply the RNEA torque gate to every timed motion segment in one cycle."""
    from curobo_core.trajectory import assemble_timed_cycle, validate_timed_cycle

    source_positions = np.asarray(result["frames"], dtype=np.float32).copy()
    source_digest = geometry_digest(source_positions)
    contract = result.get("baseline_trajectory_contract")
    if require_contract and (not contract or not contract.get("collision_validation_success") or
                             contract.get("timed_geometry_sha256") != source_digest or
                             contract.get("identity_sha256") != trajectory_identity_digest(result) or
                             contract.get("joint_names") != list(result.get("joint_names", ()))):
        raise ValueError("missing or stale validated baseline trajectory contract")
    updated = copy.deepcopy(result) if copy_result else result
    segments, reports, failed = [], [], False
    started = time.perf_counter()
    deadline = started + retime_budget_s if retime_budget_s is not None else None
    active_sides = tuple(result["task"])
    for segment in result["timed_segments"]:
        loaded = bool(result["payload"][segment["frame_start"]])
        output, before, constraint = model.constrain(
            segment, active_sides if loaded else (), result["tool_to_box"], deadline=deadline)
        failed |= not constraint["success"]
        segments.append(segment if output is None else output)
        reports.append({"phase": segment["name"], "payload": loaded,
                        "before": before, "constraint": constraint})
    for segment in segments:
        loaded = bool(updated["payload"][segment["frame_start"]])
        sides = active_sides if loaded else ()
        source_names = list(segment.get("joint_names", model.names))
        indices = [source_names.index(name) for name in model.names]
        q = np.asarray(segment["positions"], dtype=np.float32)[:, indices]
        qd = np.asarray(segment["velocities"], dtype=np.float32)[:, indices]
        qdd = np.asarray(segment["accelerations"], dtype=np.float32)[:, indices]
        torque, _ = model.torque(q, qd, qdd, sides, updated["tool_to_box"])
        gravity, _ = model.torque(q, np.zeros_like(qd), np.zeros_like(qdd),
                                  sides, updated["tool_to_box"])
        segment["torque_joint_names"] = list(model.names)
        segment["torques_nm"] = torque.tolist()
        segment["gravity_torques_nm"] = gravity.tolist()
    updated["timed_segments"] = segments
    updated.update(assemble_timed_cycle(len(updated["frames"]), segments))
    limits = model.base_kp.joint_limits
    model_index = {name: index for index, name in enumerate(model.names)}
    mapped_limits = []
    for values in (limits.velocity[1], limits.acceleration[1], limits.jerk[1]):
        source = values.detach().cpu().numpy()
        mapped_limits.append(np.asarray([source[model_index[name]] if name in model_index else np.inf
                                         for name in updated["joint_names"]]))
    updated["timed_validation"] = validate_timed_cycle(
        updated["phases"], updated["time_from_start_s"], updated["velocities"],
        updated["accelerations"], updated["jerks"], *mapped_limits, segments)
    updated["dynamics_enabled"] = True
    updated["torque_constraints_enabled"] = True
    final_positions = np.asarray(updated["frames"], dtype=np.float32)
    position_delta = (float(np.max(np.abs(final_positions - source_positions)))
                      if final_positions.shape == source_positions.shape else float("inf"))
    final_digest = geometry_digest(final_positions)
    geometry_frozen = source_digest == final_digest and position_delta == 0.0
    updated["torque_validation"] = {
        "success": not failed and updated["timed_validation"]["success"] and geometry_frozen,
        "geometry_frozen": geometry_frozen,
        "maximum_position_delta": position_delta,
        "geometry_hash_before": source_digest,
        "geometry_hash_after": final_digest,
        "payload_mass_kg": model.payload_mass_kg,
        "torque_limit_scale": model.torque_limit_scale,
        "retimed_segments": sum(item["constraint"].get("time_scale", 1.0) > 1.000001
                                  for item in reports),
        "local_retimed_segments": sum(item["constraint"].get("retiming_mode") == "local_window"
                                        for item in reports),
        "uniform_fallback_segments": sum(item["constraint"].get("retiming_mode") == "uniform_fallback"
                                           for item in reports),
        "failed_segments": sum(not item["constraint"]["success"] for item in reports),
        "retiming_wall_ms": (time.perf_counter() - started) * 1000.0,
        "retiming_budget_ms": None if retime_budget_s is None else retime_budget_s * 1000.0,
        "segments": reports,
    }
    return updated
