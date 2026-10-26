"""Add q/qdot/qddot/jerk timing to the mentor planner without changing its planning interfaces."""
import time
import math
import xml.etree.ElementTree as ET
from dataclasses import replace

import numpy as np
import torch

from curobo_core.scene import Pose
from curobo_core.trajectory import (assemble_timed_cycle, time_parameterize_path,
                                    time_parameterize_polyline, time_parameterize_stops,
                                    validate_timed_cycle)
from v3_batched_loaded_search import GpuValidity
from v3_single_arm_planner import SingleArmPlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS


EVENT_PHASES = {"home", "attach", "release", "base_advance", "base_reposition",
                "base_return", "base_pick_reposition"}


class TimedSingleArmPlanner(SingleArmPlanner):
    def __init__(self, args, sample_dt=.025):
        super().__init__(args)
        self.trajectory_sample_dt = float(sample_dt)
        if self.trajectory_sample_dt <= 0:
            raise ValueError("sample_dt must be positive")
        config = self.mobile_robot["kinematics"]
        self._timed_joint_names = list(config["cspace"]["joint_names"])
        joints = {joint.attrib["name"]: joint for joint in
                  ET.parse(config["urdf_path"]).getroot().findall("joint")}
        self._timed_limits = (
            np.asarray([float(joints[name].find("limit").attrib["velocity"])
                        for name in self._timed_joint_names]),
            np.asarray(config["cspace"]["max_acceleration"], dtype=float),
            np.asarray(config["cspace"]["max_jerk"], dtype=float),
        )

    def full_cycle(self, boxes, progress=lambda text: None):
        result = super().full_cycle(boxes, progress)
        if result.get("success"):
            self.time_parameterize_result(result)
        return result

    def _timed_phase_validity(self, phase, boxes, original_snapshot, first_position):
        self.snapshot = original_snapshot
        if phase in ("conveyor_place", "return_home"):
            mapping = dict(zip(self._timed_joint_names, first_position))
            yaw = mapping["base_yaw"]
            base = Pose((mapping["base_x"] - self.base_advance, mapping["base_y"], 0.),
                        (np.cos(yaw / 2), 0., 0., np.sin(yaw / 2)))
            self.snapshot = replace(original_snapshot,
                                    state=original_snapshot.state.with_positions(mapping, base))
        checker = self.checker(boxes, payload=phase in ("transport", "conveyor_place"))
        return (self.target_validity(checker) if phase == "approach" else
                GpuValidity(checker, check_ground=True))

    def _timed_phase_valid(self, phase, positions, boxes, original_snapshot):
        validity = self._timed_phase_validity(phase, boxes, original_snapshot, positions[0])
        indices = [self._timed_joint_names.index(name) for name in ACTIVE_JOINTS]
        active = np.ascontiguousarray(np.asarray(positions)[:, indices])
        mask = validity.mask(torch.tensor(active, device="cuda", dtype=torch.float32))
        self._last_timed_invalid = torch.nonzero(~mask, as_tuple=False).reshape(-1).cpu().tolist()
        self.snapshot = original_snapshot
        return bool(mask.all().item())

    def _shortcut_phase(self, phase, rows, boxes, original_snapshot):
        if phase not in ("approach", "transport", "conveyor_place", "return_home"):
            return rows, {"raw_waypoints": len(rows), "shortcut_waypoints": len(rows)}
        validity = self._timed_phase_validity(phase, boxes, original_snapshot, rows[0])
        indices = [self._timed_joint_names.index(name) for name in ACTIVE_JOINTS]
        active = torch.tensor(np.ascontiguousarray(rows[:, indices]), device="cuda", dtype=torch.float32)
        selected = [0]
        source = 0
        while source < len(rows) - 1:
            candidates = active[source + 1:]
            starts = active[source].reshape(1, -1).repeat(len(candidates), 1)
            connectable = validity.edges(starts, candidates, resolution=math.radians(.05))
            valid = torch.nonzero(connectable, as_tuple=False).reshape(-1)
            source += int(valid[-1].item()) + 1 if len(valid) else 1
            selected.append(source)
        self.snapshot = original_snapshot
        return rows[selected], {"raw_waypoints": len(rows), "shortcut_waypoints": len(selected)}

    def time_parameterize_result(self, result):
        started = time.perf_counter()
        names = list(result["joint_names"])
        if names != self._timed_joint_names:
            raise ValueError("mentor result joint order differs from the timed mobile model")
        source_frames = np.asarray(result["frames"], dtype=float)
        source_phases = list(result["phases"])
        source_payload = list(result["payload"])
        boxes = dict(result["task"])
        self.configure_task(boxes)
        validation_snapshot = self.snapshot
        frames, phases, payload, segments = [], [], [], []
        index = 0
        sample_dt = self.trajectory_sample_dt
        while index < len(source_frames):
            end = index + 1
            while end < len(source_frames) and source_phases[end] == source_phases[index]:
                end += 1
            phase = source_phases[index]
            rows = source_frames[index:end]
            if phase in EVENT_PHASES or len(rows) < 2 or np.allclose(rows[0], rows[-1], atol=1e-12, rtol=0):
                frames.extend(rows.tolist())
                phases.extend(source_phases[index:end])
                payload.extend(source_payload[index:end])
            else:
                timing_rows, shortcut = self._shortcut_phase(
                    phase, rows, boxes, validation_snapshot)
                candidates = []
                cubic, cubic_validation = time_parameterize_path(
                    timing_rows, names, *self._timed_limits, sample_dt=sample_dt)
                cubic_valid = not (phase in ("contact_approach", "extract") and
                                   cubic["maximum_linear_deviation"] > .002)
                if cubic_valid and self._timed_phase_valid(
                        phase, cubic_validation, boxes, validation_snapshot):
                    candidates.append((cubic["duration_s"], "validated_shortcut_cubic",
                                       cubic, cubic_validation))
                if len(timing_rows) < len(rows):
                    stopped, stopped_validation = time_parameterize_stops(
                        timing_rows, names, *self._timed_limits, sample_dt=sample_dt)
                    if self._timed_phase_valid(
                            phase, stopped_validation, boxes, validation_snapshot):
                        candidates.append((stopped["duration_s"], "validated_shortcut_stops",
                                           stopped, stopped_validation))
                if candidates:
                    _, parameterizer, trajectory, validation = min(candidates, key=lambda item: item[0])
                else:
                    trajectory, validation = time_parameterize_path(
                        rows, names, *self._timed_limits, sample_dt=sample_dt)
                    parameterizer = "original_path_cubic_fallback"
                    shortcut = {"raw_waypoints": len(rows), "shortcut_waypoints": len(rows)}
                    if not self._timed_phase_valid(phase, validation, boxes, validation_snapshot):
                        trajectory, validation = time_parameterize_polyline(
                            rows, names, *self._timed_limits, sample_dt=sample_dt)
                        parameterizer = "original_path_polyline_fallback"
                        if not self._timed_phase_valid(phase, validation, boxes, validation_snapshot):
                            raise RuntimeError(f"{phase} timed samples failed the mentor collision gate")
                sampled = trajectory["sampled_trajectory"]
                constant = np.ptp(rows, axis=0) <= 1e-12
                if np.any(constant):
                    sampled_positions = np.asarray(sampled["positions"], dtype=float)
                    sampled_positions[:, constant] = rows[0, constant]
                    sampled["positions"] = sampled_positions.tolist()
                    for field in ("velocities", "accelerations", "jerks"):
                        values = np.asarray(sampled[field], dtype=float)
                        values[:, constant] = 0.0
                        sampled[field] = values.tolist()
                start = len(frames)
                frames.extend(sampled["positions"])
                phases.extend([phase] * len(sampled["positions"]))
                payload.extend([source_payload[index]] * len(sampled["positions"]))
                segments.append({
                    "name": phase, "frame_start": start,
                    "frame_end": len(frames) - 1,
                    "duration_s": trajectory["duration_s"],
                    "maximum_linear_deviation": trajectory["maximum_linear_deviation"],
                    "parameterizer": parameterizer,
                    **shortcut,
                    **sampled,
                })
            index = end
        result["frames"], result["phases"], result["payload"] = frames, phases, payload
        result["timed_segments"] = segments
        result.update(assemble_timed_cycle(len(frames), segments))
        result["timed_validation"] = validate_timed_cycle(
            phases, result["time_from_start_s"], result["velocities"],
            result["accelerations"], result["jerks"], *self._timed_limits, segments)
        if not result["timed_validation"]["success"]:
            raise RuntimeError(f"timed trajectory validation failed: {result['timed_validation']}")
        for event in result.get("predicted_scene_events", ()):
            event["frame_index"] = phases.index(event["phase"])
        result["trajectory_timing_ms"] = (time.perf_counter() - started) * 1000
        result["dynamics_enabled"] = False
        result["torque_constraints_enabled"] = False
        return result
