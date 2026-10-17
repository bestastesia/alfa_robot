"""Stage-local adapter; reuse the baseline search, TrajOpt and collision policies."""
import math
import time

import numpy as np
import torch

from .backend import CuroboBackend
from .contracts import ACTIVE_JOINTS
from .trajectory import TimedTrajectory, braking_trajectory, join_timed_trajectories
from v3_batched_loaded_search import GpuValidity


class SegmentReplanner:
    def __init__(self, backend, boxes, *, payload=False, sample_dt=.025, phase="approach", braking_duration_s=.5, allow_braking_fallback=True):
        if not isinstance(backend, CuroboBackend):
            raise TypeError("reuse an initialized CuroboBackend")
        if phase not in ("approach", "contact_approach", "extract", "transport", "conveyor_place", "return_home"):
            raise ValueError("phase requires an explicit online motion contract")
        if type(payload) is not bool or payload != (phase in ("extract", "transport", "conveyor_place")):
            raise ValueError("payload mode must match the phase contract")
        if type(allow_braking_fallback) is not bool:
            raise ValueError("allow_braking_fallback must be an explicit boolean")
        if not math.isfinite(sample_dt) or sample_dt <= 0:
            raise ValueError("positive validation sample interval required")
        self.backend, self.boxes, self.payload = backend, dict(boxes), payload
        self.sample_dt = sample_dt
        if not math.isfinite(braking_duration_s) or braking_duration_s <= 0:
            raise ValueError("positive finite braking duration required")
        self.phase = phase
        self.constrained_phase = phase in ("contact_approach", "extract")
        self.braking_duration_s = braking_duration_s
        self.allow_braking_fallback = allow_braking_fallback
        # Existing connect mode returns the first finely verified feasible path;
        # the baseline anytime RRT intentionally consumes its full search budget.
        self.backend.planner_kind = "informed_connect"

    def scene(self, snapshot):
        if bool(snapshot.attachments) != self.payload:
            raise ValueError("payload mode must match execution-confirmed attachments")
        self.backend.set_snapshot(snapshot, allow_attached=True, require_locked_state=True)
        # PR65 already caches by robot/payload topology, updates world geometry,
        # and invalidates its base-cuboid query cache. Reuse that whole contract.
        validity = self.backend.online_validity(self.boxes, self.payload, self.phase)
        self.terminal_validity = self.backend.online_terminal_validity(self.boxes, self.phase)
        return validity

    def validate(self, document, validity, *, require_phase_terminal=True):
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
        if require_phase_terminal and self.terminal_validity is not None and not bool(self.terminal_validity.mask(q[-1:]).item()):
            raise ValueError("extract terminal state failed loaded-payload gate")
        return {"limit_ratios": ratios, "collision_samples": len(samples),
                "edge_resolution_rad": math.radians(.5), "sample_dt_s": self.sample_dt,
                "dynamics_enabled": False, "torque_constraints_enabled": False}

    def brake_then_plan(self, ticket, validity, search_seed):
        """Conservative fallback: preserve moving boundary, stop inside the new suffix,
        then reuse Mentor search + the existing TrajOpt to the ORIGINAL goal.
        All work shares the original deadline. This is not an emergency stop API.
        """
        began = time.perf_counter()
        telemetry = {"attempted": True, "accepted": False, "strategy": "brake_then_optimize",
                     "braking_duration_s": self.braking_duration_s}
        try:
            brake = braking_trajectory(ticket.initial_state, ACTIVE_JOINTS, self.braking_duration_s)
            generated = time.perf_counter()
            telemetry["braking_generation_s"] = generated-began
            telemetry["braking_validation"] = self.validate(brake, validity)
            telemetry["braking_validation_s"] = time.perf_counter()-generated
            if time.perf_counter() >= ticket.compute_deadline:
                raise TimeoutError("deadline expired during braking validation")
            rest_start = TimedTrajectory(brake).sample(self.braking_duration_s)
            goal = np.asarray(ticket.remaining["positions"][-1], dtype=np.float32)
            limits = validity.checker.kinematics.get_joint_limits().position
            lower, upper = self.backend.motion_bounds(limits[0]+1e-5, limits[1]-1e-5, self.boxes)
            search_started = time.perf_counter()
            path, stats = self.backend.online_search_path(
                torch.tensor(rest_start["positions"], device="cuda", dtype=torch.float32),
                torch.tensor(goal[None], device="cuda", dtype=torch.float32),
                lower, upper, validity, max(0., (ticket.compute_deadline-search_started)/2), search_seed)
            telemetry["search"] = stats
            telemetry["search_wall_s"] = time.perf_counter()-search_started
            if path is None or time.perf_counter() >= ticket.compute_deadline:
                raise ValueError("no timely path from the validated stopping point")
            _, optimization = self.backend.optimize_rrt_path(
                path, validity, self.boxes, payload=self.payload, start_state=rest_start,
                timing_scale=1., uniform_seed=True)
            rest = optimization.pop("trajectory", None)
            telemetry["rest_optimization"] = optimization
            if "solve_time_ms" in optimization:
                telemetry["solve_time_ms"] = optimization["solve_time_ms"]
            if rest is None or not optimization["accepted"]:
                raise ValueError("rest-to-goal optimizer rejected the braking fallback")
            join_started = time.perf_counter()
            joined = join_timed_trajectories(brake, rest)
            telemetry["joined_validation"] = self.validate(joined, validity)
            telemetry["join_and_validation_s"] = time.perf_counter()-join_started
            if time.perf_counter() > ticket.compute_deadline:
                raise TimeoutError("braking fallback missed compute deadline")
            telemetry.update(accepted=True, internal_stops=1)
            return joined, telemetry
        except (ValueError, RuntimeError, TimeoutError) as error:
            telemetry["fallback_reason"] = str(error)
            return None, telemetry
        finally:
            telemetry["wall_s"] = time.perf_counter()-began

    def check_endpoints(self, path, validity):
        values = torch.tensor(np.asarray(path)[[0, -1]], device="cuda", dtype=torch.float32)
        return [bool(item) for item in validity.mask(values).cpu().tolist()]

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
            remaining_duration = TimedTrajectory(ticket.remaining).duration
            if remaining_duration > max_duration_s:
                raise ValueError("remaining trajectory exceeds available prediction horizon")
            # Prediction validity is NOT the duration occupied by the robot.
            # Start with the current suffix duration; final validation below
            # rebuilds the envelope for the candidate's actual arrival time.
            scene_end = ticket.splice_at + remaining_duration
            # The committed prefix only occupies [observation, splice]. Using
            # the whole future envelope here rejects obstacles arriving later.
            prefix_snapshot = observation.swept_snapshot(observation.observed_at, ticket.splice_at)
            validity = self.scene(prefix_snapshot)
            spans["prefix_scene_update"] = [started, time.perf_counter()]
            report["prefix_certificate"] = {"snapshot": prefix_snapshot.to_dict(),
                                            "valid_from": observation.observed_at,
                                            "valid_until": ticket.splice_at, "trajectory": prefix}
            try:
                report["prefix_validation"] = self.validate(prefix, validity, require_phase_terminal=False)
            except ValueError as error:
                report["failure_code"] = "UNSAFE_COMMITTED_PREFIX"
                report["required_action"] = "independent_protective_stop"
                raise ValueError(f"committed prefix unsafe: {error}") from error
            spans["prefix_validation"] = [spans["prefix_scene_update"][1], time.perf_counter()]
            scene_started = time.perf_counter()
            snapshot = observation.swept_snapshot(ticket.splice_at, scene_end)
            validity = self.scene(snapshot)
            report["planning_certificate"] = {"snapshot": snapshot.to_dict(),
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
            report["endpoint_validity"] = self.check_endpoints(seed_path, validity)
            if not all(report["endpoint_validity"]):
                # Fixed endpoints are part of the user contract. No optimizer can
                # repair a goal inside this conservative occupancy envelope.
                report["failure_code"] = ("SPLICE_ENDPOINT_COLLISION" if not report["endpoint_validity"][0]
                                          else "GOAL_ENDPOINT_COLLISION")
                report["required_action"] = "independent_safety_intervention"
                report["endpoint_check_interval"] = [ticket.splice_at, scene_end]
                raise ValueError(f"{report['failure_code']}: fixed endpoint violates the planning envelope")
            if getattr(self, "constrained_phase", False):
                if not report["previous_suffix_valid"]:
                    report["failure_code"] = "CONSTRAINED_PHASE_BLOCKED"
                    report["required_action"] = "independent_protective_stop"
                    report["certificate"] = report["planning_certificate"]
                    raise ValueError(f"{self.phase} corridor blocked; free-space detour is forbidden")
                report.update(success=True, optimization_success=False,
                              strategy="revalidated_constrained_suffix", trajectory=ticket.remaining,
                              certificate={"snapshot": snapshot.to_dict(), "valid_from": ticket.splice_at,
                                           "valid_until": scene_end},
                              internal_stops=0, constrained_phase=True)
                ready_at = time.perf_counter()
                report.update(started_at=started, ready_at=ready_at,
                              compute_wall_s=ready_at-started, deadline_miss=ready_at>ticket.compute_deadline)
                return report
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
                # Final validation temporarily loads the previous candidate's
                # actual-time world. Do not let a rejected candidate silently
                # change the planning world for unrelated subsequent attempts.
                refresh_started = time.perf_counter()
                validity = self.scene(snapshot)
                spans[f"attempt_scene_update_{attempt}"] = [refresh_started, time.perf_counter()]
                if search_attempt:
                    search_started = time.perf_counter()
                    limits = validity.checker.kinematics.get_joint_limits().position
                    lower, upper = self.backend.motion_bounds(limits[0] + 1e-5, limits[1] - 1e-5, self.boxes)
                    path, search = self.backend.online_search_path(
                        torch.tensor(seed_path[0], device="cuda"),
                        torch.tensor(seed_path[-1:], device="cuda"), lower, upper, validity,
                        max(0., (ticket.compute_deadline - search_started) / 2), search_seed)
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
                    if report["previous_suffix_valid"]:
                        break  # one optimization attempt; don't search for an already safe route
                    if attempt == 0 and self.allow_braking_fallback:
                        brake_started = time.perf_counter()
                        document, braking = self.brake_then_plan(ticket, validity, search_seed)
                        spans["brake_then_plan"] = [brake_started, time.perf_counter()]
                        candidates.append(braking)
                        progress({"phase": "braking_fallback", "at": time.perf_counter(), "telemetry": braking})
                        if document is not None:
                            telemetry = braking
                    if document is None or not telemetry["accepted"]:
                        continue
                validation_started = time.perf_counter()
                try:
                    curve = TimedTrajectory(document)
                    if curve.duration > max_duration_s:
                        raise ValueError("optimized suffix outlives obstacle prediction envelope")
                    candidate_end = ticket.splice_at + curve.duration
                    candidate_snapshot = observation.swept_snapshot(ticket.splice_at, candidate_end)
                    validity = self.scene(candidate_snapshot)
                    report["validation"] = self.validate(document, validity)
                    candidate_certificate = {"snapshot": candidate_snapshot.to_dict(),
                                             "valid_from": ticket.splice_at, "valid_until": candidate_end}
                    if (report["previous_suffix_valid"] and
                            curve.duration > ticket.remaining["time_from_start_s"][-1] + 1e-6):
                        telemetry["online_rejection"] = "would postpone an already safe arrival"
                        break
                except ValueError as error:
                    telemetry["online_rejection"] = str(error)
                    if report["previous_suffix_valid"]:
                        break
                    continue
                finally:
                    spans[f"final_validation_{attempt}"] = [validation_started, time.perf_counter()]
                report.update(success=True, optimization_success=True,
                              strategy=telemetry.get("strategy", "optimized_suffix"), trajectory=curve.suffix(0),
                              internal_stops=telemetry.get("internal_stops", 0),
                              certificate=candidate_certificate)
                break
            if not report["success"]:
                # Retain a trajectory ONLY after validating its entire remaining
                # interpolant against THIS observation/envelope. This is not a
                # successful optimization and is counted separately in reports.
                fallback_started = time.perf_counter()
                if TimedTrajectory(ticket.remaining).duration > max_duration_s:
                    raise ValueError("remaining trajectory outlives prediction")
                validity = self.scene(snapshot)
                try:
                    report["validation"] = self.validate(ticket.remaining, validity)
                except ValueError as error:
                    report["failure_code"] = "NO_VALID_REPLACEMENT"
                    report["required_action"] = "independent_safety_intervention"
                    report["certificate"] = report["planning_certificate"]
                    raise ValueError(f"no valid replacement before deadline: {error}") from error
                report["certificate"] = {"snapshot": snapshot.to_dict(),
                                         "valid_from": ticket.splice_at, "valid_until": scene_end}
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
