"""Stage-local adapter; reuse the baseline search, TrajOpt and collision policies."""
import math
import time

import numpy as np
import torch

from .backend import CuroboBackend
from .contracts import ACTIVE_JOINTS
from .scene import Pose
from .trajectory import TimedTrajectory
from v3_batched_loaded_search import GpuValidity


class SegmentReplanner:
    def __init__(self, backend, boxes, *, payload=False, sample_dt=.025):
        if not isinstance(backend, CuroboBackend):
            raise TypeError("reuse an initialized CuroboBackend")
        if not math.isfinite(sample_dt) or sample_dt <= 0:
            raise ValueError("positive validation sample interval required")
        self.backend, self.boxes, self.payload = backend, dict(boxes), payload
        self.sample_dt = sample_dt
        # Existing connect mode returns the first finely verified feasible path;
        # the baseline anytime RRT intentionally consumes its full search budget.
        self.backend.planner_kind = "informed_connect"

    def scene(self, snapshot):
        if snapshot.state.base_pose != Pose():
            raise ValueError("this segment uses the fixed-base 15-joint model")
        if bool(snapshot.attachments) != self.payload:
            raise ValueError("payload mode must match execution-confirmed attachments")
        self.backend.set_snapshot(snapshot, allow_attached=True)
        checker = self.backend.collision_checker(self.boxes, payload=self.payload)
        return GpuValidity(
            checker, active_sides=tuple(self.boxes) if self.payload else (),
            attachments=self.backend.attachments(self.boxes) if self.payload else None,
            max_box_tilt_deg=snapshot.policy.max_box_tilt_deg, check_ground=True,
            ground_z=snapshot.policy.ground_z_m,
            weights=self.backend.motion_weights(self.boxes).tolist())

    def validate(self, document, validity):
        curve = TimedTrajectory(document)
        if curve.joint_names != tuple(ACTIVE_JOINTS):
            raise ValueError("online segment joint order must match ACTIVE_JOINTS")
        limits = validity.checker.kinematics.get_joint_limits()
        cpu = lambda value: value.detach().cpu().numpy()
        ratios = curve.validate_limits(cpu(limits.position[0]), cpu(limits.position[1]),
                                       abs(cpu(limits.velocity[1])), abs(cpu(limits.acceleration[1])),
                                       abs(cpu(limits.jerk[1])))
        samples = np.unique(np.concatenate((curve.times, np.linspace(
            0., curve.duration, max(2, math.ceil(curve.duration / self.sample_dt) + 1)))))
        q = torch.tensor(curve.sample(samples)["positions"], device="cuda", dtype=torch.float32)
        mask = validity.mask(q)
        if not bool(mask.all().item()):
            first = int(torch.nonzero(~mask, as_tuple=False)[0].item())
            raise ValueError(f"interpolated trajectory failed collision/stability/position gate at {samples[first]:.6f}s")
        if not bool(validity.edges(q[:-1], q[1:], resolution=math.radians(.5)).all().item()):
            raise ValueError("interpolated trajectory edge collision rejected")
        return {"limit_ratios": ratios, "collision_samples": len(samples),
                "edge_resolution_rad": math.radians(.5), "sample_dt_s": self.sample_dt,
                "dynamics_enabled": False, "torque_constraints_enabled": False}

    def solve(self, ticket, observation, prefix, *, max_duration_s, search_seed=20261003, progress=lambda event: None):
        """Called in the worker while the executor advances through prefix.

        Every scene refresh, search, interpolation and final check is charged to t2.
        A late result is returned as a failure, NEVER a rest-to-rest fallback.
        """
        started = time.perf_counter()
        spans = {}
        report = {"success": False, "sequence": ticket.sequence,
                  "observation_id": observation.identity, "spans": spans}
        try:
            if max_duration_s <= 0 or not math.isfinite(max_duration_s):
                raise ValueError("positive bounded prediction duration required")
            scene_end = ticket.splice_at + max_duration_s
            # The committed prefix only occupies [observation, splice]. Using
            # the whole future envelope here rejects obstacles arriving later.
            prefix_snapshot = observation.swept_snapshot(observation.observed_at, ticket.splice_at)
            validity = self.scene(prefix_snapshot)
            spans["prefix_scene_update"] = [started, time.perf_counter()]
            report["prefix_certificate"] = {"snapshot": prefix_snapshot.to_dict(),
                                            "valid_from": observation.observed_at,
                                            "valid_until": ticket.splice_at, "trajectory": prefix}
            try:
                report["prefix_validation"] = self.validate(prefix, validity)
            except ValueError as error:
                raise ValueError(f"committed prefix unsafe: {error}") from error
            spans["prefix_validation"] = [spans["prefix_scene_update"][1], time.perf_counter()]
            scene_started = time.perf_counter()
            snapshot = observation.swept_snapshot(ticket.splice_at, scene_end)
            validity = self.scene(snapshot)
            report["certificate"] = {"snapshot": snapshot.to_dict(),
                                     "valid_from": ticket.splice_at, "valid_until": scene_end}
            spans["scene_update"] = [scene_started, time.perf_counter()]
            audit_started = time.perf_counter()
            try:
                report["previous_suffix_validation"] = self.validate(ticket.remaining, validity)
                report["previous_suffix_valid"] = True
            except ValueError as error:
                report["previous_suffix_valid"] = False
                report["previous_suffix_rejection"] = str(error)
            spans["previous_suffix_validation"] = [audit_started, time.perf_counter()]
            seed_path = np.asarray(ticket.remaining["positions"], dtype=np.float32)
            candidates = []
            # Loaded-boundary probe: weighted arc-length seeding + max dt failed,
            # while the same geometry resampled uniformly at .75*max dt passed.
            # Keep the legacy first attempt; bounded seed/dt diversity shares t2.
            attempts = ((None, False), (1., False), (.75, True), (.5, True), (.75, True))
            for attempt, (timing_scale, uniform_seed) in enumerate(attempts):
                search_attempt = attempt == len(attempts)-1
                progress({"phase": "search" if search_attempt else "trajopt", "attempt": attempt,
                          "at": time.perf_counter()})
                if time.perf_counter() >= ticket.compute_deadline:
                    raise TimeoutError("t2 expired before solve")
                if search_attempt:
                    search_started = time.perf_counter()
                    limits = validity.checker.kinematics.get_joint_limits().position
                    lower, upper = self.backend.motion_bounds(limits[0] + 1e-5, limits[1] - 1e-5, self.boxes)
                    path, search = self.backend.search_path(
                        torch.tensor(seed_path[0], device="cuda"),
                        torch.tensor(seed_path[-1:], device="cuda"), lower, upper, validity,
                        max(0., (ticket.compute_deadline - search_started) / 2), search_seed,
                        apply_shortcut=True)
                    report["search"] = search
                    spans["search"] = [search_started, time.perf_counter()]
                    if path is None:
                        raise ValueError("no replacement geometric path")
                    seed_path = np.asarray(path, dtype=np.float32)
                solve_started = time.perf_counter()
                _, telemetry = self.backend.optimize_rrt_path(
                    seed_path, validity, self.boxes, payload=self.payload,
                    start_state=ticket.initial_state, timing_scale=timing_scale, uniform_seed=uniform_seed)
                spans[f"trajopt_{attempt}"] = [solve_started, time.perf_counter()]
                document = telemetry.pop("trajectory", None)
                candidates.append(telemetry)
                progress({"phase": "trajopt_result", "attempt": attempt,
                          "at": time.perf_counter(), "telemetry": telemetry})
                report["attempts"] = candidates
                if document is None or not telemetry["accepted"]:
                    continue
                validation_started = time.perf_counter()
                try:
                    curve = TimedTrajectory(document)
                    if curve.duration > max_duration_s:
                        raise ValueError("optimized suffix outlives obstacle prediction envelope")
                    report["validation"] = self.validate(document, validity)
                    if (report["previous_suffix_valid"] and
                            curve.duration > ticket.remaining["time_from_start_s"][-1] + 1e-6):
                        telemetry["online_rejection"] = "would postpone an already safe arrival"
                        break
                except ValueError as error:
                    telemetry["online_rejection"] = str(error)
                    continue
                finally:
                    spans[f"final_validation_{attempt}"] = [validation_started, time.perf_counter()]
                report.update(success=True, optimization_success=True,
                              strategy="optimized_suffix", trajectory=curve.suffix(0))
                break
            if not report["success"]:
                # Retain a trajectory ONLY after validating its entire remaining
                # interpolant against THIS observation/envelope. This is not a
                # successful optimization and is counted separately in reports.
                fallback_started = time.perf_counter()
                if TimedTrajectory(ticket.remaining).duration > max_duration_s:
                    raise ValueError("remaining trajectory outlives prediction")
                report["validation"] = self.validate(ticket.remaining, validity)
                spans["fallback_revalidation"] = [fallback_started, time.perf_counter()]
                report.update(success=True, optimization_success=False,
                              strategy="revalidated_suffix", trajectory=ticket.remaining)
        except (ValueError, RuntimeError, TimeoutError) as error:
            report.update(success=False, error=f"{type(error).__name__}: {error}")
            report.pop("trajectory", None)
        ready_at = time.perf_counter()
        if ready_at > ticket.compute_deadline:
            report.update(success=False, error="compute deadline miss")
            report.pop("trajectory", None)
        report.update(started_at=started, ready_at=ready_at, compute_wall_s=ready_at-started,
                      deadline_miss=ready_at > ticket.compute_deadline)
        return report
