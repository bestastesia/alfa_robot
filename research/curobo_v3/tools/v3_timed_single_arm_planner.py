"""Add q/qdot/qddot/jerk timing to the mentor planner without changing its planning interfaces."""
import copy
import gc
import time
import math
import xml.etree.ElementTree as ET
from dataclasses import replace

import numpy as np
import torch

from curobo_core.scene import Pose
from curobo_core.torque import constrain_timed_result
from curobo_core.trajectory import (assemble_timed_cycle, joint_state_trajectory,
                                    resample_path, time_parameterize_path,
                                    time_parameterize_polyline, time_parameterize_stops,
                                    validate_timed_cycle)
from v3_batched_loaded_search import GpuValidity, loaded_robot
from v3_single_arm_planner import SingleArmPlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS


EVENT_PHASES = {"home", "attach", "release", "base_advance", "base_reposition",
                "base_return", "base_pick_reposition"}
TRAJOPT_PHASES = {"approach", "transport", "conveyor_place", "return_home"}


class TimedSingleArmPlanner(SingleArmPlanner):
    def __init__(self, args, sample_dt=.025, torque_model=None, use_trajopt=True):
        super().__init__(args)
        self.trajectory_sample_dt = float(sample_dt)
        self.torque_model = torque_model
        self.use_trajopt = bool(use_trajopt)
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


    def _set_timed_phase_snapshot(self, phase, original_snapshot, first_position):
        self.snapshot = original_snapshot
        if phase in ("conveyor_place", "return_home"):
            mapping = dict(zip(self._timed_joint_names, first_position))
            yaw = mapping["base_yaw"]
            base = Pose((mapping["base_x"] - self.base_advance, mapping["base_y"], 0.),
                        (np.cos(yaw / 2), 0., 0., np.sin(yaw / 2)))
            self.snapshot = replace(original_snapshot,
                                    state=original_snapshot.state.with_positions(mapping, base))

    def _timed_phase_validity(self, phase, boxes, original_snapshot, first_position):
        self._set_timed_phase_snapshot(phase, original_snapshot, first_position)
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

    def _trajectory_optimizer(self, phase, rows, boxes, original_snapshot):
        """Run cuRobo TrajectoryOptimizer on a shortcut RRT seed, with safe fallback metadata."""
        if not self.use_trajopt or phase not in TRAJOPT_PHASES:
            return None, {"trajopt_attempted": False}
        indices = [self._timed_joint_names.index(name) for name in ACTIVE_JOINTS]
        fixed = [index for index in range(len(self._timed_joint_names)) if index not in indices]
        if fixed and not np.allclose(rows[:, fixed], rows[0, fixed], atol=1e-7, rtol=0):
            return None, {"trajopt_attempted": True,
                          "trajopt_failure": "non-arm joints move inside the phase"}
        started = time.perf_counter()
        planner = result = plan = None
        try:
            from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
            from curobo.types import JointState

            self._set_timed_phase_snapshot(phase, original_snapshot, rows[0])
            loaded = phase in ("transport", "conveyor_place")
            robot = (loaded_robot(self.robot, self.box_fit, attachments=self.task_attachments(boxes))
                     if loaded else copy.deepcopy(self.robot))
            planner = MotionPlanner(MotionPlannerCfg.create(
                robot=robot, scene_model=self.scene(boxes),
                collision_cache={"cuboid": max(40, len(self.snapshot.objects)),
                                 "mesh": len(self.snapshot.meshes)},
                num_ik_seeds=4, num_trajopt_seeds=1, self_collision_check=True,
                use_cuda_graph=False, optimizer_collision_activation_distance=0.0,
                interpolation_dt=self.trajectory_sample_dt,
            ))
            active = np.ascontiguousarray(rows[:, indices], dtype=np.float32)
            horizon = planner.trajopt_solver.action_horizon
            seed = resample_path(active, horizon)

            def state(values):
                return JointState.from_position(
                    torch.tensor(values, device="cuda", dtype=torch.float32).reshape(1, -1),
                    joint_names=ACTIVE_JOINTS)

            result = planner.trajopt_solver.solve_cspace(
                goal_state=state(active[-1]), current_state=state(active[0]),
                seed_traj=torch.tensor(seed, device="cuda").view(1, 1, horizon, -1),
                num_seeds=1, return_seeds=1, finetune_attempts=2)
            torch.cuda.synchronize()
            if not bool(result.success.reshape(-1)[0].item()):
                raise RuntimeError("cuRobo TrajectoryOptimizer returned failure")
            plan = result.get_interpolated_plan()
            if plan is None:
                raise RuntimeError("cuRobo TrajectoryOptimizer returned no interpolated plan")
            active_trajectory = joint_state_trajectory(plan, ACTIVE_JOINTS)
            active_positions = np.asarray(active_trajectory["positions"], dtype=np.float32)
            if (np.max(np.abs(active_positions[0] - active[0])) > 1e-5 or
                    np.max(np.abs(active_positions[-1] - active[-1])) > 1e-5):
                raise RuntimeError("optimized endpoints differ from the accepted seed")
            sampled = {"joint_names": list(self._timed_joint_names), "dt_s": active_trajectory["dt_s"],
                       "time_from_start_s": active_trajectory["time_from_start_s"]}
            for field in ("positions", "velocities", "accelerations", "jerks"):
                source = np.asarray(active_trajectory[field], dtype=np.float32)
                expanded = np.zeros((len(source), len(self._timed_joint_names)), dtype=np.float32)
                expanded[:, indices] = source
                if field == "positions":
                    expanded[:, fixed] = rows[0, fixed]
                sampled[field] = expanded.tolist()
            ratios = [float(np.max(np.abs(np.asarray(sampled[field])) / limit))
                      for field, limit in zip(("velocities", "accelerations", "jerks"),
                                              self._timed_limits)]
            if max(ratios) > 1.0001:
                raise RuntimeError(f"optimized derivatives exceed limits: {ratios}")
            positions = np.asarray(sampled["positions"], dtype=np.float32)
            if not self._timed_phase_valid(phase, positions, boxes, original_snapshot):
                raise RuntimeError(f"optimized samples failed collision gate: {self._last_timed_invalid[:5]}")
            duration = float(sampled["time_from_start_s"][-1])
            parameter = np.linspace(0.0, 1.0, len(rows))
            sample_parameter = np.linspace(0.0, 1.0, len(positions))
            linear = np.column_stack([np.interp(sample_parameter, parameter, rows[:, joint])
                                      for joint in range(rows.shape[1])])
            return ({"joint_names": list(self._timed_joint_names), "dt_s": sampled["dt_s"],
                     "duration_s": duration,
                     "maximum_linear_deviation": float(np.max(np.abs(positions - linear))),
                     "sampled_trajectory": sampled},
                    {"trajopt_attempted": True, "trajopt_accepted": True,
                     "trajopt_action_horizon": int(horizon),
                     "trajopt_solve_time_ms": float(result.solve_time) * 1000.0,
                     "trajopt_wall_ms": (time.perf_counter() - started) * 1000.0,
                     "trajopt_limit_ratios": ratios})
        except Exception as error:
            return None, {"trajopt_attempted": True, "trajopt_accepted": False,
                          "trajopt_failure": str(error),
                          "trajopt_wall_ms": (time.perf_counter() - started) * 1000.0}
        finally:
            self.snapshot = original_snapshot
            del plan, result, planner
            gc.collect()
            torch.cuda.empty_cache()

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
                trajectory, trajopt = self._trajectory_optimizer(
                    phase, timing_rows, boxes, validation_snapshot)
                if trajectory is not None:
                    parameterizer = "curobo_v2_trajectory_optimizer"
                else:
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
                        _, parameterizer, trajectory, _ = min(candidates, key=lambda item: item[0])
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
                    **trajopt,
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
        if self.torque_model is None:
            result["dynamics_enabled"] = False
            result["torque_constraints_enabled"] = False
        else:
            torque_started = time.perf_counter()
            constrained = constrain_timed_result(result, self.torque_model, copy_result=False)
            constrained["torque_constraint_ms"] = (time.perf_counter() - torque_started) * 1000
            if not constrained["torque_validation"]["success"]:
                raise RuntimeError(f"torque validation failed: {constrained['torque_validation']}")
            result = constrained
        return result
