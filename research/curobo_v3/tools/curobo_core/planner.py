import ctypes
import json
import math
from pathlib import Path
import time
import threading

import numpy as np
from scipy.spatial.transform import Rotation
import torch
import yourdfpy

from curobo.types import JointState

from v3_batched_loaded_search import GpuValidity, densify
from .backend import CuroboBackend, WORKSPACE_ROOT, goal
from v3_wall_ik_benchmark import ACTIVE_JOINTS, canonical_side_suction_quaternion_wxyz
from .adapter import (task_contact_positions, task_attachments, suction_quaternion,
                      carry_targets, top_lift_blockers, side_face_variants, matrix_pose)
from .contracts import PlanRequest
from .scene import Pose, SceneStore
from .sequence import plan_sequence
from .distance_metric import joint_distance_weights
from .trajectory import (assemble_timed_cycle, expand_timed_trajectory, time_parameterize_path,
                         time_parameterize_stops, trajectory_fallback_path, validate_timed_cycle)


class CycleBlocked(RuntimeError):
    def __init__(self, stage, details, partial=None):
        super().__init__(f"{stage}: {details}")
        self.stage = stage
        self.details = details
        self.partial = partial


class AnalyticBridge:
    def __init__(self):
        path = WORKSPACE_ROOT / "generated/v3_analytic_071cb95/libv3_analytic_bridge.so"
        self.library = ctypes.CDLL(str(path))
        array = np.ctypeslib.ndpointer(dtype=np.float64, flags="C_CONTIGUOUS")
        self.library.v3_swivel.argtypes = [ctypes.c_int, array]
        self.library.v3_swivel.restype = ctypes.c_double
        self.library.v3_solve.argtypes = [ctypes.c_int, array, ctypes.c_double, array, array, ctypes.c_int]
        self.library.v3_solve.restype = ctypes.c_int

    def swivel(self, side, values):
        return self.library.v3_swivel(side, np.ascontiguousarray(values, dtype=np.float64))

    def solve(self, side, target, swivel, seed):
        output = np.zeros((16, 7), dtype=np.float64)
        count = self.library.v3_solve(
            side, np.ascontiguousarray(target, dtype=np.float64), swivel,
            np.ascontiguousarray(seed, dtype=np.float64), output, len(output),
        )
        return output[:count]


class FullCyclePlanner(CuroboBackend):
    def __init__(self, args):
        super().__init__(args)
        self.analytic = AnalyticBridge()
        self.fk_robot = yourdfpy.URDF.load(args.urdf, load_meshes=False, build_scene_graph=True)
        self.last_partial = None
        self._planning_lock = threading.Lock()

    def checker(self, boxes, payload=False, mobile=False):
        return self.collision_checker(boxes, payload=payload, mobile=mobile)

    def contact_poses_for_task(self, boxes):
        return {f"{side}_tool0": {
                    "position": position,
                    "quaternion": suction_quaternion(side, self.suction_mode)}
                for side, position in task_contact_positions(
                    self.snapshot, boxes, suction_mode=self.suction_mode,
                    offsets=self.contact_offsets).items()}

    def plan_contact_approach(self, contact, checker, validity, seed):
        return self.joint_plan(
            self.home_values, contact, checker, validity, seed, self.selected_task)

    def demo_request(self, boxes):
        return PlanRequest(self.snapshot, tuple(boxes.items()), tuple(
            (name, Pose(tuple(pose["position"]), tuple(pose["quaternion"])))
            for name, pose in self.default_target_poses.items()),
            search_seed=getattr(self.args, "search_seed", None))

    def plan_request(self, request, progress=lambda text: None):
        if not self._planning_lock.acquire(blocking=False):
            raise RuntimeError("planning core is busy")
        try:
            return self._plan_request(request, progress)
        finally:
            self._planning_lock.release()

    def _plan_request(self, request, progress):
        if request.snapshot.state.base_pose != Pose():
            raise ValueError("current full cycle requires fixed base at map origin")
        state = dict(zip(request.snapshot.state.joint_names, request.snapshot.state.positions))
        if any(abs(state.get(name, 0.0)) > 1e-12 for name in ("head_joint", "head_pitch_joint")):
            raise ValueError("current model locks both head joints at zero")
        self.set_snapshot(request.snapshot)
        self.search_seed = request.search_seed
        self.home_values = np.asarray(request.snapshot.state.ordered(ACTIVE_JOINTS), dtype=np.float32)
        requested_targets = {name: {"position": pose.position, "quaternion": pose.quaternion_wxyz}
                             for name, pose in request.targets}
        boxes = dict(request.tasks)
        self.selected_task = boxes
        for box_id in boxes.values():
            item = request.snapshot.object(f"wall_box_{box_id:02d}")
            if item.pose.quaternion_wxyz != Pose().quaternion_wxyz:
                raise ValueError("current extraction contract requires axis-aligned task boxes")
            if not np.allclose(item.dimensions_m, self.box_fit["dimensions_m"], atol=1e-12):
                raise ValueError("task box dimensions differ from payload sphere model")
        started = time.perf_counter()
        attempts = []
        variants = ([('side', {}), *[('side', offset) for offset in side_face_variants(request.snapshot, boxes)],
                     ('top', {})] if request.suction_mode == "auto" else [(request.suction_mode, {})])
        for mode, offsets in variants:
            self.suction_mode = mode
            self.contact_offsets = offsets
            self.target_poses = carry_targets(request.snapshot, boxes, requested_targets, mode, offsets)
            attempt_started = time.perf_counter()
            self.last_partial = None
            try:
                blockers = top_lift_blockers(request.snapshot, boxes) if mode == "top" else []
                if blockers:
                    raise CycleBlocked("顶吸净空", f"垂直抬升通道被占用: {blockers}")
                progress(f"尝试{'顶吸' if mode == 'top' else '侧吸'}，面内偏移={offsets or '默认'}")
                result = self._full_cycle(boxes, progress)
            except (CycleBlocked, RuntimeError) as error:
                result = getattr(error, "partial", None) or self.last_partial or {
                    "success": False, "task": boxes, "frames": [], "phases": [], "payload": []}
                result["blocked_stage"] = getattr(error, "stage", "planning_failure")
                result["blocked_details"] = str(getattr(error, "details", error))
            attempts.append({"mode": mode, "contact_offsets_m": offsets, "success": result["success"],
                             "wall_ms": (time.perf_counter()-attempt_started)*1000,
                             "blocked_stage": result.get("blocked_stage"),
                             "blocked_details": result.get("blocked_details"),
                             "contact_ik_diagnostics": result.get("contact_ik_diagnostics")})
            if result["success"] or result.get("blocked_stage") not in ("接触IK", "候选筛选"):
                break
        result["suction_mode"] = self.suction_mode
        result["contact_offsets_m"] = self.contact_offsets
        result["suction_attempts"] = attempts
        result["selected_mode_total_ms"] = result.get("total_ms")
        result["total_ms"] = (time.perf_counter()-started)*1000
        result["carry_tool_targets"] = self.target_poses
        result["search_seed"] = request.search_seed
        result["seed_policy"] = {"scope": "RRT_sampling_only", "effective_search_seed_base": self.search_seed_for(),
                                 "ik_random_seed_unchanged": True, "per_search_budget_s": 2.0,
                                 "bitwise_reproducibility_guaranteed": False}
        result["request_id"] = request.identity
        result["model_id"] = request.snapshot.model_id
        result["scene_id"] = request.snapshot.identity
        result["scene_revision"] = request.snapshot.revision
        result["state_id"] = request.snapshot.state.identity
        result["policy"] = request.snapshot.to_dict()["policy"]
        result["error"] = None if result["success"] else {
            "code": "PLANNING_FAILED", "stage": result.get("blocked_stage"),
            "message": result.get("blocked_details"),
        }
        result["predicted_scene_events"] = self._predicted_events(request, result)
        return result

    def plan_from_store(self, store, tasks, targets, progress=lambda text: None, search_seed=None):
        request = PlanRequest(store.snapshot(), tuple(tasks.items()), tuple(targets.items()), search_seed=search_seed)
        result = self.plan_request(request, progress)
        if not store.is_current(request.snapshot):
            result["success"] = False
            result["error"] = {"code": "SCENE_CHANGED", "stage": "acceptance",
                               "message": "scene or robot state changed during planning"}
        return result

    def full_cycle(self, boxes, progress=lambda text: None):
        return self.plan_request(self.demo_request(boxes), progress)

    def plan_sequence(self, snapshot, progress=lambda text: None, search_seed=None):
        if not self._planning_lock.acquire(blocking=False):
            raise RuntimeError("planning core is busy")
        try:
            return plan_sequence(self, snapshot, progress, search_seed=search_seed)
        finally:
            self._planning_lock.release()

    def _predicted_events(self, request, result):
        predicted = SceneStore(request.snapshot)
        events = []
        attachments = self.attachments(dict(request.tasks))
        for index, phase in enumerate(result.get("phases", ())):
            if phase == "attach" and not events:
                values = dict(zip(result["joint_names"], result["frames"][index]))
                predicted.update_state(predicted.snapshot().state.with_positions(values),
                                       predicted.snapshot().revision)
                predicted.attach_many(attachments, predicted.snapshot().revision)
                events.append({"phase": phase, "frame_index": index,
                               "snapshot": predicted.snapshot().to_dict()})
            elif phase == "release" and len(events) == 1:
                values = dict(zip(result["joint_names"], result["frames"][index]))
                base_pose = Pose((values["base_x"], values["base_y"], 0.0),
                                 (math.cos(values["base_yaw"] / 2), 0.0, 0.0,
                                  math.sin(values["base_yaw"] / 2)))
                predicted.update_state(predicted.snapshot().state.with_positions(values, base_pose),
                                       predicted.snapshot().revision)
                for attachment in attachments:
                    predicted.release(attachment.object_id, predicted.snapshot().revision)
                events.append({"phase": phase, "frame_index": index,
                               "snapshot": predicted.snapshot().to_dict()})
        return events

    def fk(self, values):
        mapping = dict(zip(ACTIVE_JOINTS, values))
        self.fk_robot.update_cfg({name: mapping.get(name, 0.0)
                                  for name in self.fk_robot.actuated_joint_names})

    def stabilize_idle_contacts(self, candidates, boxes, validity):
        if len(boxes) != 1:
            return candidates, np.zeros(len(candidates)), ["paired_ik"] * len(candidates), None
        idle = next(side for side in ("left", "right") if side not in boxes)
        side_index = 0 if idle == "left" else 1
        offset = 1 + side_index * 7
        self.fk(self.home_values)
        initial_target = self.fk_robot.get_transform(idle + "_tool0", self.fk_robot.base_link).copy()
        initial_carriage_z = self.fk_robot.get_transform("arm_carriage", self.fk_robot.base_link)[2, 3]
        initial_swivel = self.analytic.swivel(side_index, self.home_values[offset:offset+7])
        shoulder_indices = [ACTIVE_JOINTS.index(side + "_joint1") for side in ("left", "right")]
        output, height_offsets, sources = [], [], []
        for contact in candidates:
            self.fk(contact)
            actual_idle = self.fk_robot.get_transform(idle + "_tool0", self.fk_robot.base_link).copy()
            world_to_carriage = np.linalg.inv(
                self.fk_robot.get_transform("arm_carriage", self.fk_robot.base_link))
            carriage_delta = (self.fk_robot.get_transform(
                "arm_carriage", self.fk_robot.base_link)[2, 3] - initial_carriage_z)
            trials = [np.asarray(contact).copy()]
            offsets = [actual_idle[2, 3] - initial_target[2, 3]]
            trial_sources = ["ik_nullspace"]
            # Add deterministic idle-arm alternatives; original collision-valid IK remains eligible.
            for fraction in np.linspace(0., 1.25, 11):
                target = initial_target.copy()
                target[2, 3] += carriage_delta * fraction
                for swivel_offset in np.deg2rad(
                        [0., 30., -30., 60., -60., 90., -90., 120., -120., 150., -150., 180.]):
                    solutions = self.analytic.solve(
                        side_index, world_to_carriage @ target,
                        initial_swivel + swivel_offset, self.home_values[offset:offset+7])
                    for solution in solutions:
                        row = np.asarray(contact).copy()
                        row[offset:offset+7] = solution
                        trials.append(row)
                        offsets.append(carriage_delta * fraction)
                        trial_sources.append("analytic_vertical_compensation")
            trials, offsets = np.asarray(trials), np.asarray(offsets)
            within_motion = np.all(np.abs(
                trials[:, shoulder_indices] - self.home_values[shoulder_indices]) <= math.pi, axis=1)
            trials, offsets = trials[within_motion], offsets[within_motion]
            trial_sources = np.asarray(trial_sources, dtype=object)[within_motion]
            if not len(trials):
                continue
            valid = validity.mask(torch.tensor(
                trials, device="cuda", dtype=torch.float32)).cpu().numpy()
            trials, offsets, trial_sources = trials[valid], offsets[valid], trial_sources[valid]
            if not len(trials):
                continue
            cost = np.sum((trials[:, offset:offset+7] - self.home_values[offset:offset+7]) ** 2, axis=1)
            selected = int(np.argmin(cost))
            output.append(trials[selected])
            height_offsets.append(offsets[selected])
            sources.append(str(trial_sources[selected]))
        return (np.asarray(output).reshape(-1, len(ACTIVE_JOINTS)),
                np.asarray(height_offsets), sources, initial_target)

    def analytic_extract(self, contact, empty_validity, loaded_validity, active_sides=("left", "right")):
        self.last_extract_frames = np.asarray(contact, dtype=np.float64).reshape(1, -1)
        self.fk(contact)
        world_to_carriage = np.linalg.inv(
            self.fk_robot.get_transform("arm_carriage", self.fk_robot.base_link))
        targets = {side: self.fk_robot.get_transform(f"{side}_tool0", self.fk_robot.base_link).copy()
                   for side in active_sides}
        swivels = [self.analytic.swivel(side, contact[1 + side * 7:8 + side * 7])
                   for side in range(2)]
        rows = [np.asarray(contact, dtype=np.float64)]
        top = self.suction_mode == "top"
        if top:
            lower, upper = loaded_validity.checker.kinematics.get_joint_limits().position
            if contact[0] < float(lower[0]) or contact[0]+0.35 > float(upper[0]):
                return None, "顶吸抬升35cm超出升降限位"
        for distance in np.linspace(0.01, 0.35, 35):
            next_row = rows[-1].copy()
            if top:
                next_row[0] = contact[0] + distance
                rows.append(next_row)
                continue
            for side_index, side in enumerate(("left", "right")):
                if side not in active_sides:
                    continue
                target = targets[side].copy()
                target[0, 3] -= distance
                offset = 1 + side_index * 7
                seed = rows[-1][offset:offset + 7]
                solutions = self.analytic.solve(side_index, world_to_carriage @ target,
                                                swivels[side_index], seed)
                if len(solutions) == 0:
                    self.last_extract_frames = np.asarray(rows)
                    return None, f"{side}固定ψ解析无解@{distance*100:.1f}cm"
                nearest = np.argmin(np.sum((solutions - seed) ** 2, axis=1))
                next_row[offset:offset + 7] = solutions[nearest]
            rows.append(next_row)
        dense = np.asarray(densify(np.asarray(rows)))
        self.last_extract_frames = dense
        extraction_validity = empty_validity if self.snapshot.policy.defer_payload_until_extract_end else loaded_validity
        valid = extraction_validity.mask(torch.tensor(dense, device="cuda", dtype=torch.float32))
        if not bool(valid.all().item()):
            return None, f"抽离机器人碰撞/稳定性失败@frame{int((~valid).nonzero()[0])+1}"
        if not bool(loaded_validity.mask(torch.tensor(dense[-1:], device="cuda", dtype=torch.float32)).item()):
            return None, "抽离终点携箱碰撞/稳定性失败"
        maximum_line_error = 0.0
        maximum_orientation_error = 0.0
        for values in dense:
            self.fk(values)
            for side in active_sides:
                actual = self.fk_robot.get_transform(f"{side}_tool0", self.fk_robot.base_link)
                maximum_line_error = max(maximum_line_error,
                                         float(np.linalg.norm(actual[[0, 1] if top else [1, 2], 3] -
                                                              targets[side][[0, 1] if top else [1, 2], 3])))
                maximum_orientation_error = max(maximum_orientation_error, float(Rotation.from_matrix(
                    actual[:3, :3].T @ targets[side][:3, :3]).magnitude()))
        if maximum_line_error > 0.002 or maximum_orientation_error > math.radians(1.0):
            return None, f"抽离插值偏离直线/朝向: {maximum_line_error*1000:.3f}mm"
        if top and any(abs(self.fk_robot.get_transform(f"{side}_tool0", self.fk_robot.base_link)[2, 3]
                           - targets[side][2, 3] - .35) > .002 for side in active_sides):
            return None, "升降轴未实现世界Z方向35cm顶升"
        return dense, {"method": "updown_lift" if top else "fixed_updown_analytic",
                       "direction_world": [0, 0, 1] if top else [-1, 0, 0],
                       "start_updown_m": float(contact[0]), "fixed_psi_rad": swivels,
                       "max_line_error_mm": maximum_line_error * 1000,
                       "max_orientation_error_deg": math.degrees(maximum_orientation_error)}

    def joint_plan(self, start, target, checker, validity, seed, boxes=()):
        limits = checker.kinematics.get_joint_limits().position
        lower, upper = self.motion_bounds(limits[0] + 1e-5, limits[1] - 1e-5, boxes)
        path, stats = self.search_path(
            torch.tensor(start, device="cuda", dtype=torch.float32),
            torch.tensor(np.asarray(target).reshape(1, -1), device="cuda", dtype=torch.float32),
            lower, upper, validity, 2.0, seed, apply_shortcut=True,
        )
        stats["search_seed"] = seed
        if path is None:
            return None, stats
        dense = np.asarray(densify(path))
        if not bool(validity.mask(torch.tensor(dense, device="cuda", dtype=torch.float32)).all().item()):
            return None, {**stats, "failure": "密化轨迹验收失败"}
        if getattr(self.args, "trajopt_rrt", False):
            try:
                optimized, telemetry = self.optimize_rrt_path(path, validity, boxes)
            except (RuntimeError, ValueError) as error:
                optimized, telemetry = None, {
                    "attempted": True, "accepted": False,
                    "dynamics_enabled": False, "torque_constraints_enabled": False,
                    "fallback_reason": f"{type(error).__name__}: {error}",
                }
            stats["trajectory_optimization"] = telemetry
            if optimized is not None:
                return optimized, stats
            stats["trajectory_fallback_waypoints"] = np.asarray(path).tolist()
        return dense, stats

    def _full_cycle(self, boxes, progress=lambda text: None):
        started = time.perf_counter()
        report = {"success": False, "task": boxes, "joint_names": list(
            self.mobile_robot["kinematics"]["cspace"]["joint_names"]),
            "frames": [], "phases": [], "payload": [], "attempts": [], "timed_segments": [],
            "fixed_timing": {}}
        report["planner"] = self.planner_kind
        timing = {}
        report["timing_ms"] = timing
        self.last_partial = report
        mobile_names = report["joint_names"]

        def append(rows, phase, payload):
            for row in rows:
                mapping = dict(zip(ACTIVE_JOINTS, row)) if len(row) == 15 else dict(zip(mobile_names, row))
                values = [float(mapping.get(name, 0.0)) for name in mobile_names]
                if report["frames"] and phase not in ("release", "attach") and np.allclose(
                        report["frames"][-1], values, atol=1e-10, rtol=0.0):
                    continue
                report["frames"].append(values)
                report["phases"].append(phase)
                report["payload"].append(payload)

        def appended_active_rows(start):
            indices = [mobile_names.index(name) for name in ACTIVE_JOINTS]
            return np.asarray(report["frames"][start:], dtype=np.float32)[:, indices]

        def record_timed_trajectory(name, frame_count_before_append, trajectory):
            expanded = expand_timed_trajectory(trajectory, mobile_names)
            positions = np.asarray(expanded["positions"])
            for start in (max(0, frame_count_before_append - 1), frame_count_before_append):
                end = start + len(positions)
                if end <= len(report["frames"]) and np.allclose(
                        report["frames"][start:end], positions, atol=1e-5, rtol=0):
                    if "sampled_trajectory" in trajectory:
                        if end != len(report["frames"]):
                            raise ValueError("resampling must target the just-appended segment")
                        expanded = expand_timed_trajectory(trajectory["sampled_trajectory"], mobile_names)
                        count = len(expanded["positions"])
                        for field in ("phases", "payload"):
                            labels = report[field][start:end]
                            report[field][start:end] = [labels[0]] + [labels[-1]] * (count - 1)
                        report["frames"][start:end] = expanded["positions"]
                        end = start + count
                    report["timed_segments"].append({
                        "name": name, "frame_start": start, "frame_end": end - 1, **expanded,
                    })
                    return True
            return False

        def record_timed_segment(name, frame_count_before_append, stats):
            optimization = stats.get("trajectory_optimization", {})
            trajectory = optimization.pop("trajectory", None)
            if not optimization.get("accepted") or trajectory is None:
                return
            if not record_timed_trajectory(name, frame_count_before_append, trajectory):
                optimization["accepted"] = False
                optimization["fallback_reason"] = "optimized samples could not be aligned to cycle frames"

        def fixed_timing(name, rows, validity, joint_names, terminal_validity=None,
                         preserve_edges=False):
            limits = validity.checker.kinematics.get_joint_limits()
            parameterizer = time_parameterize_stops if preserve_edges else time_parameterize_path
            trajectory, validation = parameterizer(
                rows, joint_names,
                np.abs(limits.velocity[1].detach().cpu().numpy()),
                np.abs(limits.acceleration[1].detach().cpu().numpy()),
                np.abs(limits.jerk[1].detach().cpu().numpy()),
                sample_dt=float(getattr(self.args, "trajopt_interpolation_dt", .025)))
            values = torch.tensor(validation, device="cuda", dtype=torch.float32)
            node_mask = validity.mask(values)
            edge_mask = validity.edges(values[:-1], values[1:], resolution=math.radians(.5))
            accepted = bool(node_mask.all().item() and edge_mask.all().item())
            if accepted and terminal_validity is not None:
                accepted = bool(terminal_validity.mask(values[-1:]).item())
            report["fixed_timing"][name] = {
                "accepted": accepted, "duration_s": trajectory.get("duration_s"),
                "maximum_linear_deviation": trajectory.get("maximum_linear_deviation"),
                "validation_frames": len(validation),
                "invalid_nodes": torch.nonzero(~node_mask, as_tuple=False).reshape(-1).cpu().tolist()[:20],
                "invalid_edges": torch.nonzero(~edge_mask, as_tuple=False).reshape(-1).cpu().tolist()[:20],
                "dynamics_enabled": False, "torque_constraints_enabled": False,
            }
            return trajectory if accepted else None, validation

        candidate_limit = getattr(self, "contact_candidate_limit", 128)
        progress(f"检查初始状态并求接触IK（最多{candidate_limit}候选）")
        segment_started = time.perf_counter()
        empty_checker = self.checker(boxes)
        empty_validity = GpuValidity(empty_checker, check_ground=True,
                                     ground_z=self.snapshot.policy.ground_z_m,
                                     weights=self.motion_weights(boxes).tolist())
        if not bool(empty_validity.mask(torch.tensor(self.home_values[None], device="cuda")).item()):
            raise CycleBlocked("初始姿态", "碰撞或越限", report)
        append([self.home_values], "home", False)
        torch.cuda.synchronize()
        timing["initial_checker_setup_and_validation"] = (time.perf_counter()-segment_started)*1000
        segment_started = time.perf_counter()
        solver = self.ik_solver(boxes)
        torch.cuda.synchronize()
        timing["contact_solver_setup"] = (time.perf_counter()-segment_started)*1000
        segment_started = time.perf_counter()
        contact_poses = self.contact_poses_for_task(boxes)
        report["contact_tool_poses"] = {name: {k: [float(v) for v in values] for k, values in pose.items()}
                                        for name, pose in contact_poses.items()}
        result = solver.solve_pose(goal(contact_poses), JointState.from_position(
            torch.tensor(self.home_values[None], device="cuda"), joint_names=ACTIVE_JOINTS),
            return_seeds=candidate_limit)
        names = list(result.js_solution.joint_names)
        values = result.js_solution.position.reshape(-1, len(names))
        successful = result.success.reshape(-1).bool()
        report["contact_ik_diagnostics"] = {
            "statistics_scope": "returned_candidates_only_not_global_best",
            "returned_candidates": int(len(successful)),
            "successful_candidates": int(successful.sum().item()),
            "minimum_position_error_m": float(result.position_error.min().item()),
            "minimum_rotation_error_rad": float(result.rotation_error.min().item()),
        }
        candidates = values[successful][:, [names.index(name) for name in ACTIVE_JOINTS]].cpu().numpy()
        candidates, idle_offsets, idle_sources, idle_target = self.stabilize_idle_contacts(
            candidates, boxes, empty_validity)
        if len(boxes) == 1:
            shoulders = [ACTIVE_JOINTS.index(side + "_joint1") for side in ("left", "right")]
            within_motion = np.all(np.abs(candidates[:, shoulders] - self.home_values[shoulders]) <= math.pi, axis=1)
            candidates, idle_offsets = candidates[within_motion], idle_offsets[within_motion]
            idle_sources = list(np.asarray(idle_sources, dtype=object)[within_motion])
            report["idle_contact_policy"] = "minimize_idle_joint_motion_with_vertical_tcp_relaxation"
            report["shoulder_excursion_limit_deg"] = 180.
            report["idle_stabilized_candidate_count"] = len(candidates)
        if len(candidates) == 0:
            raise CycleBlocked("接触IK", "cuRobo没有返回成功候选", report)
        weights = np.array(joint_distance_weights(ACTIVE_JOINTS))
        if getattr(self, "preserve_contact_candidate_order", False):
            # 4200247 TaskCycle ties approach seeds to the sorted candidate index.
            order = np.argsort(np.sum(((candidates-self.home_values)*weights)**2, axis=1))
            candidates, idle_offsets = candidates[order], idle_offsets[order]
            idle_sources = list(np.asarray(idle_sources, dtype=object)[order])
        else:
            # Single-arm IK leaves an idle-arm nullspace: rank all solutions by motion,
            # while generic paired tasks keep the original first-32 priority.
            if len(boxes) == 1:
                order = np.argsort(np.sum(((candidates-self.home_values)*weights)**2, axis=1))
            else:
                chunks = (np.arange(min(32, len(candidates))), np.arange(32, len(candidates)))
                order = np.concatenate([chunk[np.argsort(np.sum(((candidates[chunk]-self.home_values)*weights)**2, axis=1))]
                                        for chunk in chunks if len(chunk)])
            candidates, idle_offsets = candidates[order], idle_offsets[order]
            idle_sources = list(np.asarray(idle_sources, dtype=object)[order])
            _, unique = np.unique(np.round(candidates, 6), axis=0, return_index=True)
            unique = np.sort(unique)
            candidates, idle_offsets = candidates[unique], idle_offsets[unique]
            idle_sources = list(np.asarray(idle_sources, dtype=object)[unique])
        report["contact_candidate_limit"] = candidate_limit
        report["contact_ik_count"] = len(candidates)
        report["contact_candidates"] = candidates.tolist()
        fixed_contact = getattr(self, "comparison_contact", None)
        if fixed_contact is not None:
            candidates = np.asarray(fixed_contact).reshape(1, -1)
            idle_offsets = np.zeros(1)
            idle_sources = ["comparison_contact"]
            report["comparison_contact_fixed"] = True
        torch.cuda.synchronize()
        timing["contact_ik_and_ranking"] = (time.perf_counter()-segment_started)*1000
        segment_started = time.perf_counter()
        loaded_checker = self.checker(boxes, payload=True)
        loaded_validity = GpuValidity(loaded_checker, active_sides=tuple(boxes),
                                      attachments=self.attachments(boxes),
                                      max_box_tilt_deg=self.snapshot.policy.max_box_tilt_deg, check_ground=True,
                                      ground_z=self.snapshot.policy.ground_z_m,
                                      weights=self.motion_weights(boxes).tolist())
        extracting_validity = GpuValidity(empty_checker, active_sides=tuple(boxes),
                                      attachments=self.attachments(boxes),
                                          max_box_tilt_deg=self.snapshot.policy.max_box_tilt_deg, check_ground=True,
                                          ground_z=self.snapshot.policy.ground_z_m,
                                          weights=self.motion_weights(boxes).tolist())
        torch.cuda.synchronize()
        timing["loaded_checker_setup"] = (time.perf_counter()-segment_started)*1000
        timing["analytic_extract_and_validation"] = 0.0
        timing["approach_rrt_and_validation"] = 0.0
        selected = None
        for index, contact in enumerate(candidates):
            progress(f"候选{index+1}/{len(candidates)}：{'升降轴顶升' if self.suction_mode == 'top' else '固定Updown解析抽离'}")
            attempt_started = time.perf_counter()
            extracted, details = self.analytic_extract(contact, extracting_validity, loaded_validity, tuple(boxes))
            torch.cuda.synchronize()
            timing["analytic_extract_and_validation"] += (time.perf_counter()-attempt_started)*1000
            attempt = {"candidate": index+1, "extraction": details}
            report["attempts"].append(attempt)
            if extracted is None:
                attempt["wall_ms"] = (time.perf_counter()-attempt_started)*1000
                continue
            progress(f"候选{index+1}抽离成功：规划初始→接触")
            segment_started = time.perf_counter()
            approach_target = extracted[-1] if self.suction_mode == "top" else contact
            if self.suction_mode == "top":
                approach, stats = self.joint_plan(
                    self.home_values, approach_target, empty_checker,
                    empty_validity, self.search_seed_for(index), boxes)
            else:
                approach, stats = self.plan_contact_approach(
                    contact, empty_checker, empty_validity, self.search_seed_for(index))
            if approach is not None and self.suction_mode == "top":
                # Reach above the box, then descend along the already checked vertical path.
                approach = np.vstack((approach, extracted[::-1]))
            torch.cuda.synchronize()
            timing["approach_rrt_and_validation"] += (time.perf_counter()-segment_started)*1000
            attempt["approach"] = stats
            attempt["wall_ms"] = (time.perf_counter()-attempt_started)*1000
            if approach is not None:
                selected = (approach, extracted)
                report["selected_candidate"] = index+1
                if len(boxes) == 1:
                    idle = next(side for side in ("left", "right") if side not in boxes)
                    self.fk(contact)
                    actual = matrix_pose(self.fk_robot.get_transform(
                        idle + "_tool0", self.fk_robot.base_link))
                    report["contact_tool_poses"][idle + "_tool0"] = {
                        "position": list(actual.position), "quaternion": list(actual.quaternion_wxyz)}
                    report["idle_contact_height_offset_m"] = float(idle_offsets[index])
                    report["idle_height_relaxed"] = bool(abs(idle_offsets[index]) > 1e-9)
                    report["idle_contact_source"] = idle_sources[index]
                break
        if selected is None:
            raise CycleBlocked("候选筛选", f"{len(candidates)}个候选均未通过抽离与到位", report)
        approach, extracted = selected
        extract_timed = None
        if getattr(self.args, "trajopt_rrt", False):
            extract_timed, extract_validation = fixed_timing(
                "extract", extracted, extracting_validity, ACTIVE_JOINTS,
                loaded_validity if self.snapshot.policy.defer_payload_until_extract_end else None)
            if extract_timed is not None:
                top = self.suction_mode == "top"
                self.fk(extract_validation[0])
                targets = {side: self.fk_robot.get_transform(
                    side + "_tool0", self.fk_robot.base_link).copy() for side in boxes}
                maximum_line_error = maximum_orientation_error = 0.0
                for values in extract_validation:
                    self.fk(values)
                    for side in boxes:
                        actual = self.fk_robot.get_transform(side + "_tool0", self.fk_robot.base_link)
                        axes = [0, 1] if top else [1, 2]
                        maximum_line_error = max(maximum_line_error, float(np.linalg.norm(
                            actual[axes, 3] - targets[side][axes, 3])))
                        maximum_orientation_error = max(maximum_orientation_error, float(
                            Rotation.from_matrix(actual[:3, :3].T @ targets[side][:3, :3]).magnitude()))
                report["fixed_timing"]["extract"].update(
                    maximum_line_error_mm=maximum_line_error * 1000,
                    maximum_orientation_error_deg=math.degrees(maximum_orientation_error))
                if maximum_line_error > .002 or maximum_orientation_error > math.radians(1):
                    report["fixed_timing"]["extract"]["accepted"] = False
                    report["fixed_timing"]["extract"]["fallback_reason"] = "Cartesian extraction corridor changed"
                    extract_timed = None
        split = getattr(self, "contact_approach_split", len(approach))
        approach_optimization = attempt["approach"].get("trajectory_optimization", {})
        approach_source = trajectory_fallback_path(approach[:split], attempt["approach"])
        approach_fallback_timed = None
        if getattr(self.args, "trajopt_rrt", False) and not approach_optimization.get("accepted"):
            approach_fallback_timed, _ = fixed_timing(
                "approach", approach_source, empty_validity, ACTIVE_JOINTS,
                preserve_edges=True)
        contact_approach_timed = None
        if getattr(self.args, "trajopt_rrt", False) and split < len(approach):
            contact_approach_timed, _ = fixed_timing(
                "contact_approach", approach[split-1:], empty_validity, ACTIVE_JOINTS)
        approach_frame_start = len(report["frames"])
        append(approach_source if approach_fallback_timed is not None else approach[:split],
               "approach", False)
        if approach_fallback_timed is not None:
            aligned = record_timed_trajectory("approach", approach_frame_start, approach_fallback_timed)
            if not aligned:
                approach_fallback_timed, _ = fixed_timing(
                    "approach", appended_active_rows(approach_frame_start), empty_validity,
                    ACTIVE_JOINTS, preserve_edges=True)
                if approach_fallback_timed is not None:
                    record_timed_trajectory("approach", approach_frame_start, approach_fallback_timed)
        else:
            record_timed_segment("approach", approach_frame_start, attempt["approach"])
            if (getattr(self.args, "trajopt_rrt", False) and
                    not attempt["approach"].get("trajectory_optimization", {}).get("accepted")):
                approach_fallback_timed, _ = fixed_timing(
                    "approach", appended_active_rows(approach_frame_start), empty_validity,
                    ACTIVE_JOINTS, preserve_edges=True)
                if approach_fallback_timed is not None:
                    record_timed_trajectory("approach", approach_frame_start, approach_fallback_timed)
        contact_approach_frame_start = len(report["frames"])
        append(approach[split:], "contact_approach", False)
        if contact_approach_timed is not None:
            record_timed_trajectory(
                "contact_approach", contact_approach_frame_start, contact_approach_timed)
        append([approach[-1]], "attach", True)
        extract_frame_start = len(report["frames"])
        append(extracted, "extract", True)
        if extract_timed is not None:
            record_timed_trajectory("extract", extract_frame_start, extract_timed)
        progress("规划抽离终点→放置目标并验证携箱转身")
        segment_started = time.perf_counter()
        transport = self.plan(boxes, extracted[-1])
        torch.cuda.synchronize()
        timing["transport_pipeline_total"] = (time.perf_counter()-segment_started)*1000
        report["transport_timing_breakdown_ms"] = transport.get("timing_ms", {})
        if not transport["success"]:
            report["transport_failure"] = transport["failure"]
            append(transport["frames"], "transport_diagnostic", True)
            failure = transport["failure"]
            raise CycleBlocked("搬运/携箱转身", f"{failure['stage']}@frame{failure['frame_index']+1}", report)
        transport_rows = transport["frames"][:transport["transport_frames"]]
        turn_loaded_rows = transport["frames"][transport["transport_frames"]-1:]
        turn_loaded_timed = None
        if getattr(self.args, "trajopt_rrt", False):
            loaded_mobile_validity = GpuValidity(
                self.checker(boxes, payload=True, mobile=True), active_sides=tuple(boxes),
                attachments=self.attachments(boxes),
                max_box_tilt_deg=self.snapshot.policy.max_box_tilt_deg, check_ground=True,
                ground_z=self.snapshot.policy.ground_z_m, joint_names=mobile_names,
                weights=[2., 2., 1.] + self.motion_weights(boxes).tolist())
            turn_loaded_timed, _ = fixed_timing(
                "turn_loaded", turn_loaded_rows, loaded_mobile_validity, mobile_names)
        report["transport_timing_ms"] = transport["total_ms"]
        report["transport_search_stats"] = transport["rrt_stats"]
        transport_optimization = report["transport_search_stats"].get("trajectory_optimization", {})
        active_indices = [mobile_names.index(name) for name in ACTIVE_JOINTS]
        transport_source = trajectory_fallback_path(
            np.asarray(transport_rows)[:, active_indices], report["transport_search_stats"])
        transport_fallback_timed = None
        if (getattr(self.args, "trajopt_rrt", False) and
                not transport_optimization.get("accepted")):
            transport_fallback_timed, _ = fixed_timing(
                "transport", transport_source, loaded_validity,
                ACTIVE_JOINTS, preserve_edges=True)
        transport_frame_start = len(report["frames"])
        append(transport_source if transport_fallback_timed is not None else transport_rows,
               "transport", True)
        if transport_optimization.get("accepted"):
            record_timed_segment("transport", transport_frame_start, report["transport_search_stats"])
        elif transport_fallback_timed is not None:
            record_timed_trajectory("transport", transport_frame_start, transport_fallback_timed)
        turn_loaded_frame_start = len(report["frames"])
        append(transport["frames"][transport["transport_frames"]:], "turn_loaded", True)
        if turn_loaded_timed is not None:
            record_timed_trajectory("turn_loaded", turn_loaded_frame_start, turn_loaded_timed)
        report["transport_max_box_tilt_deg"] = transport["max_box_tilt_deg"]
        append([report["frames"][-1]], "release", False)
        progress("检查箱体释放后原地转回")
        segment_started = time.perf_counter()
        endpoint = np.array(report["frames"][-1])
        yaw_index = mobile_names.index("base_yaw")
        turn_back = np.repeat(endpoint[None], 361, axis=0)
        turn_back[:, yaw_index] = np.linspace(math.pi, 0, 361)
        mobile_checker = self.checker(boxes, mobile=True)
        mobile_validity = GpuValidity(mobile_checker, check_ground=True, joint_names=mobile_names,
                                      ground_z=self.snapshot.policy.ground_z_m,
                                      weights=[2., 2., 1., 5.] + [1.] * 14)
        torch.cuda.synchronize()
        timing["empty_turn_checker_setup"] = (time.perf_counter()-segment_started)*1000
        segment_started = time.perf_counter()
        mask = mobile_validity.mask(torch.tensor(turn_back, device="cuda", dtype=torch.float32))
        torch.cuda.synchronize()
        timing["empty_turn_validation"] = (time.perf_counter()-segment_started)*1000
        if not bool(mask.all().item()):
            raise CycleBlocked("空载转回", f"碰撞@{int((~mask).nonzero()[0])+1}", report)
        turn_empty_timed = None
        if getattr(self.args, "trajopt_rrt", False):
            turn_empty_timed, _ = fixed_timing(
                "turn_empty", turn_back, mobile_validity, mobile_names)
        turn_empty_frame_start = len(report["frames"])
        append(turn_back, "turn_empty", False)
        if turn_empty_timed is not None:
            record_timed_trajectory("turn_empty", turn_empty_frame_start, turn_empty_timed)
        progress("规划双臂空载回到原始Home")
        segment_started = time.perf_counter()
        return_start = np.array([turn_back[-1, mobile_names.index(name)] for name in ACTIVE_JOINTS])
        home_path, stats = self.joint_plan(return_start, self.home_values, empty_checker,
                                            empty_validity, self.search_seed_for(1), boxes)
        torch.cuda.synchronize()
        timing["home_rrt_and_validation"] = (time.perf_counter()-segment_started)*1000
        if home_path is None:
            # Empty return can retrace this request's validated outbound route, but recheck it.
            reuse_started = time.perf_counter()
            indices = [mobile_names.index(name) for name in ACTIVE_JOINTS]
            carried = np.asarray(transport["frames"][:transport["transport_frames"]])[:, indices]
            reverse = np.vstack((carried[::-1], extracted[::-1], approach[::-1]))
            reusable = (np.allclose(reverse[0], return_start, atol=1e-7, rtol=0) and
                        np.allclose(reverse[-1], self.home_values, atol=1e-7, rtol=0) and
                        bool(empty_validity.mask(torch.tensor(reverse, device="cuda", dtype=torch.float32)).all().item()))
            torch.cuda.synchronize()
            reuse_ms = (time.perf_counter()-reuse_started)*1000
            stats = {"strategy": "reverse_validated_empty_path", "fallback": True,
                     "failed_search": stats, "dense_validated": reusable, "reuse_validation_ms": reuse_ms}
            timing["home_rrt_and_validation"] += reuse_ms
            if not reusable:
                raise CycleBlocked("空载回Home", f"搜索及反向空载复核均失败: {stats}", report)
            progress("空载回程搜索失败：显式复用去程反向路径，空载逐帧复核通过")
            home_path = reverse
        report["home_search_stats"] = stats
        home_optimization = stats.get("trajectory_optimization", {})
        home_trajectory = home_optimization.pop("trajectory", None)
        home_source = trajectory_fallback_path(home_path, stats)
        home_fallback_timed = None
        if getattr(self.args, "trajopt_rrt", False) and not home_optimization.get("accepted"):
            home_fallback_timed, _ = fixed_timing(
                "return_home", home_source, empty_validity,
                ACTIVE_JOINTS, preserve_edges=True)
        home_frame_start = len(report["frames"])
        append(home_source if home_fallback_timed is not None else home_path,
               "return_home", False)
        if home_optimization.get("accepted") and home_trajectory is not None and not record_timed_trajectory(
                "return_home", home_frame_start, home_trajectory):
            home_optimization["accepted"] = False
            home_optimization["fallback_reason"] = "optimized samples could not be aligned to cycle frames"
        if (getattr(self.args, "trajopt_rrt", False) and
                not home_optimization.get("accepted") and home_fallback_timed is None):
            home_fallback_timed, _ = fixed_timing(
                "return_home", appended_active_rows(home_frame_start), empty_validity,
                ACTIVE_JOINTS, preserve_edges=True)
        if home_fallback_timed is not None:
            record_timed_trajectory("return_home", home_frame_start, home_fallback_timed)
        rows = np.asarray(report["frames"])
        report["max_adjacent_joint_step_deg"] = float(np.degrees(
            np.abs(np.diff(rows[:, 4:], axis=0)).max()))
        report["final_home_error"] = float(np.max(np.abs(
            rows[-1, [mobile_names.index(name) for name in ACTIVE_JOINTS]] - self.home_values)))
        if len(boxes) == 1:
            for side in ("left", "right"):
                name = side + "_joint1"
                if np.max(np.abs(rows[:, mobile_names.index(name)] - self.home_values[ACTIVE_JOINTS.index(name)])) > math.pi + 1e-6:
                    raise CycleBlocked("动作约束", f"{name}超出初始姿态半圈范围", report)
        if report["timed_segments"]:
            report.update(assemble_timed_cycle(len(report["frames"]), report["timed_segments"]))
            limits = self.checker(boxes, mobile=True).kinematics.get_joint_limits()
            report["timed_validation"] = validate_timed_cycle(
                report["phases"], report["time_from_start_s"], report["velocities"],
                report["accelerations"], report["jerks"],
                np.abs(limits.velocity[1].detach().cpu().numpy()),
                np.abs(limits.acceleration[1].detach().cpu().numpy()),
                np.abs(limits.jerk[1].detach().cpu().numpy()), report["timed_segments"])
            if not report["timed_validation"]["success"]:
                raise CycleBlocked("轨迹时间化", str(report["timed_validation"]), report)
        report["total_ms"] = (time.perf_counter()-started)*1000
        timing["other_assembly_and_bookkeeping"] = report["total_ms"] - sum(timing.values())
        report["success"] = True
        return report
