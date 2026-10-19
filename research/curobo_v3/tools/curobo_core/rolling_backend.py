"""Online boundary adapter on PR65's planner; upstream planning classes stay unchanged.

Only the existing TrajOpt wrapper is carried forward. Geometry, contacts, persistent
collision resources and search come from the new Mentor baseline.
"""
import copy
from contextlib import contextmanager
from dataclasses import asdict, replace
import math
import time

import numpy as np
import torch
from curobo.trajectory_optimizer import TrajectoryOptimizer, TrajectoryOptimizerCfg
from curobo.types import JointState
from v3_timed_single_arm_planner import TimedSingleArmPlanner
from v3_batched_loaded_search import loaded_robot
from v3_multiroot_search import batched_rrt_connect_multi_goal
from .contracts import ACTIVE_JOINTS
from .scene import Pose, digest
from .trajectory import deduplicate_timed_trajectory, joint_state_trajectory, resample_path

@contextmanager
def preserve_torch_rng():
    """Keep optional TrajOpt calls from perturbing later IK randomness."""
    cpu_state = torch.random.get_rng_state()
    cuda_state = torch.cuda.get_rng_state_all()
    try:
        yield
    finally:
        torch.random.set_rng_state(cpu_state)
        torch.cuda.set_rng_state_all(cuda_state)


class RollingBackend(TimedSingleArmPlanner):
    def __init__(self, args):
        super().__init__(args)
        self._cached_trajectory_optimizers = {}
        self.suction_mode = "side"
        self.contact_offsets = {}
        self.online_phase = "approach"

    def configure_task(self, boxes):
        result = super().configure_task(boxes)
        self.suction_mode = "top" if self.top_grasp else "side"
        return result

    def task_attachments(self, boxes):
        if self.snapshot.attachments:
            expected = {f"wall_box_{box_id:02d}": side + "_tool0" for side, box_id in boxes.items()}
            if {item.object_id: item.parent_link for item in self.snapshot.attachments} != expected:
                raise ValueError("executing payload does not match the segment task")
            return self.snapshot.attachments
        return super().task_attachments(boxes)

    def attachments(self, boxes):
        return self.task_attachments(boxes)

    def snapshot_for_stage(self, result, segment):
        """Freeze the exact phase frame/payload used by PR65's timing gate."""
        boxes = dict(result["task"])
        self.configure_task(boxes)
        original = self.snapshot
        phase = segment["name"]
        self._timed_phase_validity(phase, boxes, original, segment["positions"][0])
        snapshot = self.snapshot
        mapping = dict(zip(segment["joint_names"], segment["positions"][0]))
        snapshot = replace(snapshot, state=snapshot.state.with_positions(mapping))
        if phase in ("extract", "transport", "conveyor_place"):
            attachments = self.task_attachments(boxes)
            ids = {item.object_id for item in attachments}
            snapshot = replace(snapshot, attachments=attachments,
                               objects=tuple(o for o in snapshot.objects if o.object_id not in ids))
        self.snapshot = original
        return snapshot

    def export_cycle(self, boxes, progress=lambda text: None):
        self.configure_task(boxes)
        request = {"snapshot": self.snapshot.to_dict(), "tasks": dict(boxes)}
        result = self.full_cycle(boxes, progress)
        if result.get("success"):
            result["rolling_stage_inputs"] = {
                segment["name"]: {"snapshot": self.snapshot_for_stage(result, segment).to_dict(),
                                  "top_grasp": self.top_grasp, "base_advance_m": self.base_advance}
                for segment in result["timed_segments"]
                if segment["name"] in ("approach", "contact_approach", "extract", "transport", "conveyor_place", "return_home")}
        return {"request": request, "result": result, "baseline": "PR65@541fdbc"}

    def optimization_scene(self, boxes):
        if self.online_phase == "approach":
            from .adapter import to_curobo_scene
            # TargetContactValidity forbids contact until the precontact endpoint.
            # Include task boxes in TrajOpt too; the legacy solver excluded them
            # and repeatedly produced paths rejected by the Mentor target gate.
            return to_curobo_scene(self.snapshot)
        return self.scene(boxes)

    def online_validity(self, boxes, payload, phase):
        from v3_batched_loaded_search import GpuValidity
        self.online_phase = phase
        checker = self.collision_checker(boxes, payload=phase in ("transport", "conveyor_place"))
        if phase == "approach":
            return self.target_validity(checker)
        # contact_approach/extract deliberately match TimedSingleArmPlanner's
        # empty-robot phase gate. Extract adds an independent loaded endpoint gate.
        return GpuValidity(checker, check_ground=True)

    def online_terminal_validity(self, boxes, phase):
        if phase != "extract":
            return None
        from v3_batched_loaded_search import GpuValidity
        return GpuValidity(self.collision_checker(boxes, payload=True), check_ground=True)

    def motion_weights(self, boxes):
        from .distance_metric import joint_distance_weights
        return np.asarray(joint_distance_weights(ACTIVE_JOINTS), dtype=np.float32)

    def motion_bounds(self, lower, upper, boxes):
        return lower, upper

    def online_search_path(self, start, goals, lower, upper, validity, budget, seed):
        # The same multiroot Connect used by SingleArmPlanner, not the retired
        # full-budget legacy RRT; the caller already supplies the remaining budget.
        return batched_rrt_connect_multi_goal(
            start, goals, lower, upper, validity, budget, seed,
            informed_sampling=True, stop_at_first_valid=True, refinement_s=0.)

    def set_snapshot(self, snapshot, *, allow_attached=False, require_locked_state=False):
        if snapshot.model_id != self.model_id:
            raise ValueError("snapshot model does not match configured assets")
        locks = self.robot.get("robot_cfg", self.robot)["kinematics"].get("lock_joints") or {}
        state = dict(zip(snapshot.state.joint_names, snapshot.state.positions))
        if any((require_locked_state and name not in state) or abs(state.get(name, value)-value) > 1e-12
               for name, value in locks.items()):
            raise ValueError("robot state does not confirm the model's locked joints")
        if snapshot.attachments and not allow_attached:
            raise ValueError("full cycle must start empty; scene store supports independent attachment updates")
        if snapshot.policy.ground_support_link != "base_link":
            raise ValueError("current checker only exempts base_link support")
        self.snapshot = snapshot

    def trajectory_optimizer_key(self, boxes, payload=False):
        key = (("loaded", tuple(sorted(boxes)), self.suction_mode,
                digest(self.contact_offsets), digest(self.box_fit)) if payload else ("empty",))
        if payload and self.snapshot.attachments:
            key = (*key, digest([asdict(item) for item in self.snapshot.attachments]))
        return key

    def trajectory_optimizer(self, boxes, payload=False):
        """Return a cached kinematic TrajOpt solver; dynamics stay disabled until calibrated."""
        self.prepare_task(boxes)
        key = self.trajectory_optimizer_key(boxes, payload)
        if key not in self._cached_trajectory_optimizers:
            robot = loaded_robot(
                self.robot, self.box_fit,
                attachments=self.attachments(boxes),
            ) if payload else copy.deepcopy(self.robot)
            robot.get("robot_cfg", robot)["load_dynamics"] = False
            with preserve_torch_rng():
                config = TrajectoryOptimizerCfg.create(
                    robot=robot,
                    scene_model=self.optimization_scene(boxes),
                    collision_cache={"cuboid": max(40, len(self.snapshot.objects)),
                                     "mesh": len(self.snapshot.meshes)},
                    num_seeds=1,
                    self_collision_check=True,
                    use_cuda_graph=True,
                    optimizer_collision_activation_distance=0.0,
                    interpolation_dt=float(getattr(self.args, "trajopt_interpolation_dt", 0.025)),
                    interpolation_buffer_size=1000,
                )
                self._cached_trajectory_optimizers[key] = TrajectoryOptimizer(config)
        else:
            self._cached_trajectory_optimizers[key].scene_collision_checker.load_collision_model(
                self.optimization_scene(boxes))
        return self._cached_trajectory_optimizers[key]

    def optimize_rrt_path(self, path, validity, boxes, payload=False, *, start_state=None, timing_scale=None, uniform_seed=False):
        """Optimize one 15-DoF RRT path and retain the original path on failure."""
        optimizer = self.trajectory_optimizer(boxes, payload=payload)
        seed = resample_path(path, optimizer.action_horizon,
                             None if uniform_seed else validity.weights.detach().cpu().numpy())
        seed = torch.tensor(seed, device="cuda", dtype=torch.float32).reshape(
            1, 1, optimizer.action_horizon, len(ACTIVE_JOINTS))
        state = lambda row: JointState.from_position(
            torch.tensor(np.asarray(row).reshape(1, -1), device="cuda", dtype=torch.float32),
            joint_names=list(ACTIVE_JOINTS),
        )
        current = state(path[0])
        if start_state is not None:
            # Online splicing needs the FUTURE boundary derivatives, not rest-to-rest defaults.
            # Match q/v/a (C2). Jerk is bounded by validation, not constrained to
            # match a noisy derivative of the prior sampled trajectory.
            for field, key in (("velocity", "velocities"), ("acceleration", "accelerations")):
                values = np.asarray(start_state[key], dtype=np.float32)
                if values.shape != (len(ACTIVE_JOINTS),) or not np.isfinite(values).all():
                    raise ValueError(f"invalid splice {key}")
                setattr(current, field, torch.tensor(values.reshape(1, -1), device="cuda"))
            positions = np.asarray(start_state["positions"], dtype=np.float32)
            if (positions.shape != (len(ACTIVE_JOINTS),) or
                    not np.allclose(positions, path[0], atol=1e-5, rtol=0)):
                raise ValueError("splice position differs from the seed start")
        torch.cuda.synchronize()
        started = time.perf_counter()
        solve_options = {}
        if timing_scale is not None:
            # Nonzero-boundary feasibility is non-monotonic in dt. Bounded
            # retries use the SDK's existing dt control, not weaker constraints.
            if not math.isfinite(timing_scale) or not 0 < timing_scale <= 1:
                raise ValueError("timing_scale must be in (0, 1]")
            solve_options = {"dt": torch.tensor([[optimizer.config.maximum_trajectory_dt * timing_scale]],
                                               device="cuda", dtype=torch.float32),
                             "finetune_dt_scale": 1.0}
        with preserve_torch_rng():
            result = optimizer.solve_cspace(
                goal_state=state(path[-1]), current_state=current, seed_traj=seed,
                num_seeds=1, return_seeds=1, finetune_attempts=2, **solve_options,
            )
        torch.cuda.synchronize()
        wall_ms = (time.perf_counter() - started) * 1000.0
        telemetry = {
            "attempted": True,
            "accepted": False,
            "wall_ms": wall_ms,
            "solve_time_ms": float(result.solve_time) * 1000.0,
            "source_waypoints": len(path),
            "seed_waypoints": optimizer.action_horizon,
            "payload_collision_model": bool(payload),
            "timing_scale": timing_scale,
            "uniform_seed": bool(uniform_seed),
            "dynamics_enabled": False,
            "torque_constraints_enabled": False,
        }
        solver_success = bool(result.success.reshape(-1)[0].item())
        constraint_residual = max(
            (float(value.detach().max().cpu())
             for metrics in (result.metrics, result.interpolated_metrics) if metrics is not None
             for collection in (metrics.costs_and_constraints.constraints,
                                metrics.costs_and_constraints.hybrid_costs_constraints)
             for value in collection.values),
            default=0.0,
        )
        telemetry["solver_success"] = solver_success
        telemetry["maximum_constraint_residual"] = constraint_residual
        if not solver_success and constraint_residual > 1e-6:
            telemetry["fallback_reason"] = "cuRobo TrajOpt constraints did not converge"
            return None, telemetry
        trajectory = result.get_interpolated_plan()
        if trajectory is None:
            telemetry["fallback_reason"] = "cuRobo TrajOpt returned no interpolated trajectory"
            return None, telemetry
        timed = joint_state_trajectory(trajectory, ACTIVE_JOINTS, preserve_timing=start_state is not None)
        positions = np.asarray(timed["positions"], dtype=np.float32)
        if (not np.allclose(positions[0], path[0], atol=1e-5, rtol=0) or
                not np.allclose(positions[-1], path[-1], atol=1e-5, rtol=0)):
            telemetry["fallback_reason"] = "optimized trajectory changed an endpoint"
            return None, telemetry
        positions[0] = np.asarray(path[0], dtype=np.float32)
        positions[-1] = np.asarray(path[-1], dtype=np.float32)
        timed["positions"] = positions.tolist()
        if start_state is None:
            timed = deduplicate_timed_trajectory(timed)
        else:
            for key, tolerance in (("velocities", 1e-4), ("accelerations", 1e-3)):
                error = float(np.max(np.abs(np.asarray(timed[key][0]) - start_state[key])))
                telemetry[f"splice_{key}_error"] = error
                if error > tolerance:
                    telemetry["fallback_reason"] = f"optimizer changed splice {key}"
                    return None, telemetry
        positions = np.asarray(timed["positions"], dtype=np.float32)
        values = torch.tensor(positions, device="cuda", dtype=torch.float32)
        node_mask = validity.mask(values)
        edge_mask = (torch.ones(0, dtype=torch.bool, device="cuda") if len(values) < 2 else
                     validity.edges(values[:-1], values[1:], resolution=math.radians(0.5)))
        invalid_nodes = torch.nonzero(~node_mask, as_tuple=False).reshape(-1).cpu().tolist()
        invalid_edges = torch.nonzero(~edge_mask, as_tuple=False).reshape(-1).cpu().tolist()
        telemetry["project_invalid_nodes"] = invalid_nodes[:20]
        telemetry["project_invalid_edges"] = invalid_edges[:20]
        if invalid_nodes or invalid_edges:
            telemetry["fallback_reason"] = "project collision/stability gate rejected optimized trajectory"
            return None, telemetry
        telemetry.update(accepted=True, output_frames=len(positions), trajectory=timed)
        return positions, telemetry
