#!/usr/bin/env python3
"""Wall-clock rolling replanning on the frozen V3 model (simulation, NEVER hardware).

Run prepare_checkout.py in an isolated runtime first. --initial-cycle must be a
fresh {request, result} from that runtime's baseline, not an unrelated old replay.
"""
import argparse
from dataclasses import asdict, replace
import json
import hashlib
import importlib.metadata
import math
import platform
import queue
import threading
import multiprocessing as mp
import os
from pathlib import Path
import time

import numpy as np

from curobo_core.contracts import ACTIVE_JOINTS, PlannerAssets
from curobo_core.rolling import ExecutionFeedback, RollingSegment, SceneObservation, SliceSchedule
from curobo_core.scene import Pose, SceneObject, SceneSnapshot
from curobo_core.trajectory import TimedTrajectory



def runtime_source_hashes(tools):
    """Include inherited Mentor planners, not just the online adapter directory."""
    return {str(path.relative_to(tools)): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in sorted(tools.rglob("*.py"))}


def stage_input(record, name, round_index=0):
    if record.get("sequence"):
        record = record["rounds"][round_index]
    elif round_index:
        raise ValueError("--round requires a sequence document")
    result = record["result"] if "result" in record else record["results"][-1]
    if not result["success"]:
        raise ValueError("initial full-cycle planning failed")
    stage = next(item for item in result["timed_segments"] if item["name"] == name)
    indices = [stage["joint_names"].index(joint) for joint in ACTIVE_JOINTS]
    locked = [i for i, joint in enumerate(stage["joint_names"]) if joint not in ACTIVE_JOINTS]
    positions = np.asarray(stage["positions"])
    if locked and np.any(np.ptp(positions[:, locked], axis=0) > 1e-7):
        raise ValueError("moving-base segments need their own model; cannot discard those joints")
    if name not in result.get("rolling_stage_inputs", {}):
        raise ValueError("regenerate input with v3_rolling_prepare.py on PR65; legacy snapshots are not interchangeable")
    trajectory = {"joint_names": list(ACTIVE_JOINTS), "time_from_start_s": stage["time_from_start_s"],
                  **{key: np.asarray(stage[key])[:, indices].tolist() for key in
                     ("positions", "velocities", "accelerations", "jerks")}}
    snapshot = SceneSnapshot.from_dict(result["rolling_stage_inputs"][name]["snapshot"])
    return trajectory, snapshot, result["task"], name in ("extract", "transport", "conveyor_place")



def exchange_worker(connection, commands, replies, stop):
    """Own blocking pipe I/O outside the execution/deadline supervisor thread."""
    def publish(reply):
        while not stop.is_set():
            try:
                replies.put(reply, timeout=.05)
                return True
            except queue.Full:
                pass
        return False
    try:
        if not publish(connection.recv()):  # complete startup message, not just its header
            return
        while not stop.is_set():
            try:
                command = commands.get(timeout=.05)
            except queue.Empty:
                continue
            connection.send(command)
            if command is None:
                return
            while not stop.is_set():
                reply = connection.recv()
                if not publish(reply):
                    return
                if "progress" not in reply:
                    break
    except Exception as error:
        publish({"success": False, "error": f"worker transport: {type(error).__name__}: {error}"})
    finally:
        connection.close()



def record_reply_timing(result, requested_at, received_at):
    """Monotonic same-host intervals; SDK diagnostics are nested, never summed."""
    result["requested_at"] = requested_at
    result["reply_received_at"] = received_at
    if "started_at" in result and "ready_at" in result:
        began, ready = result["started_at"], result["ready_at"]
        if not requested_at <= began <= ready <= received_at:
            raise ValueError("worker timestamps are not in the supervisor monotonic clock domain")
        result["queue_ipc_s"] = began-requested_at
        result["compute_wall_s"] = ready-began
        result["request_to_ready_s"] = ready-requested_at
        result["result_ipc_s"] = received_at-ready
    result["sdk_solve_times_s"] = [attempt["solve_time_ms"]/1000. for attempt in result.get("attempts", [])
                                   if "solve_time_ms" in attempt]
    result["timing_scope"] = {"queue_ipc": "enqueue through serialization, pipe and scheduling to worker start",
                              "sdk_solve": "SDK diagnostic nested inside TrajOpt wall span; not additive",
                              "real_perception": "unmeasured", "real_network": "unmeasured"}


def worker(connection, snapshot, boxes, payload, initial, random_seed, phase, braking_duration_s, allow_braking_fallback):
    # Own the CUDA context in a separate process so the deadline supervisor does
    # not wait on a blocked solve/GIL. Process startup and warmup precede t1.
    from curobo_core.rolling_backend import RollingBackend
    from curobo_core.rolling_curobo import SegmentReplanner
    from curobo.types import JointState
    import torch
    import curobo

    try:
        started = time.perf_counter()
        torch.manual_seed(random_seed)
        np.random.seed(random_seed)
        assets = PlannerAssets()
        backend = RollingBackend(assets)
        backend.configure_task(boxes)
        adapter = SegmentReplanner(backend, boxes, payload=payload, phase=phase, braking_duration_s=braking_duration_s,
                                   allow_braking_fallback=allow_braking_fallback)
        validity = adapter.scene(snapshot)
        initial_validation = adapter.validate(initial, validity)
        curve = TimedTrajectory(initial)
        cut = curve.duration / 4
        _, warm = backend.optimize_rrt_path(curve.suffix(cut)["positions"], validity, boxes,
                                            payload=payload, start_state=curve.sample(cut))
        warm.pop("trajectory", None)
        q = torch.tensor(curve.sample(curve.duration * .6)["positions"],
                         device="cuda", dtype=torch.float32).reshape(1, 1, -1)
        state = validity.checker.kinematics.compute_kinematics(
            JointState.from_position(q, joint_names=list(ACTIVE_JOINTS)))
        tool = state.tool_poses.to_dict()[next(iter(boxes), "left") + "_tool0"]
        from curobo_core.adapter import pose_matrix
        local_center = tool.position.reshape(-1, 3)[0].cpu().numpy()
        center = (pose_matrix(snapshot.state.base_pose) @ np.r_[local_center, 1.])[:3].tolist()
        connection.send({"ready": True, "initialization_s": time.perf_counter()-started,
                         "validation": initial_validation, "warmup": warm,
                         "obstacle_center": center, "model_id": backend.model_id,
                         "hardware": {"gpu": torch.cuda.get_device_name(), "platform": platform.platform()},
                         "packages": {**{name: importlib.metadata.version(name) for name in ("torch", "numpy", "scipy")},
                                      "curobo": getattr(curobo, "__version__", "unpackaged source")},
                         "curobo_module": str(curobo.__file__),
                         "model_assets_sha256": {name: hashlib.sha256(path.read_bytes()).hexdigest()
                                                 for name, path in vars(assets).items() if isinstance(path, Path)},
                         "base_cuboids_sha256": hashlib.sha256((Path(__file__).resolve().parents[1] /
                             "generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json").read_bytes()).hexdigest()})
        while True:
            command = connection.recv()
            if command is None:
                break
            ticket, observation, prefix, duration, seed, fault = command
            if fault == "compute_delay":
                time.sleep(max(0., ticket.compute_deadline - time.perf_counter()) + .2)
            if fault == "worker_failure":
                os._exit(91)  # controlled test fault in this run's own child only
            if fault == "solver_failure":
                connection.send({"success": False, "error": "injected solver failure", "sequence": ticket.sequence,
                                 "ready_at": time.perf_counter()})
                continue
            if ticket is None:
                began = time.perf_counter()
                snapshot = observation.swept_snapshot(observation.observed_at, observation.observed_at + duration)
                certificate = {"snapshot": snapshot.to_dict(), "valid_from": observation.observed_at,
                               "valid_until": observation.observed_at + duration}
                response = {"success": False, "trajectory": prefix, "certificate": certificate}
                try:
                    check = adapter.scene(snapshot)
                    response.update(success=True, validation=adapter.validate(prefix, check))
                except (ValueError, RuntimeError) as error:
                    response.update(error=str(error), failure_code="UNSAFE_COMMITTED_PREFIX",
                                    required_action="independent_protective_stop")
                ready = time.perf_counter()
                response.update(started_at=began, ready_at=ready, compute_wall_s=ready-began,
                                observation_id=observation.identity, terminal=True)
            else:
                response = adapter.solve(ticket, observation, prefix,
                                         max_duration_s=duration, search_seed=seed,
                                         progress=lambda event: connection.send({"progress": event}))
            connection.send(response)
    except (Exception, EOFError) as error:
        try:
            connection.send({"success": False, "error": f"worker: {type(error).__name__}: {error}"})
        except (BrokenPipeError, OSError):
            pass
    finally:
        connection.close()


def observe(snapshot, center, scenario, sequence, now, elapsed, duration, feedback, velocity_map=None, dimensions=(.08, .08, .08)):
    active = scenario in ("static", "moving") or (scenario == "appear" and sequence >= 1) or (
        scenario == "depart" and sequence < 2)
    objects = snapshot.objects
    velocities = ()
    if active:
        velocity = np.asarray([0., .025, 0.] if velocity_map is None else velocity_map) if scenario == "moving" else np.zeros(3)
        position = np.asarray(center) + velocity * (elapsed - duration * .6)
        objects += (SceneObject("rolling_obstacle", tuple(dimensions), Pose(tuple(position))),)
        velocities = (("rolling_obstacle", tuple(velocity)),)
    state = replace(snapshot.state.with_positions(dict(zip(ACTIVE_JOINTS, feedback["positions"]))),
                    stamp_ns=round(now * 1e9))
    scene = replace(snapshot, objects=objects, state=state, revision=snapshot.revision+sequence+1)
    return SceneObservation(scene, now, now, now + max(2 * duration, 30.), velocities,
                            source=f"synthetic:{scenario}")


def run(args):
    initial, snapshot, boxes, payload = stage_input(json.loads(args.initial_cycle.read_text()), args.stage, args.round)
    curve = TimedTrajectory(initial)
    schedule = SliceSchedule(curve.duration, args.compute_bound_s, args.link_bound_s, args.slices)
    if not 0 <= args.link_delay_s <= args.link_bound_s:
        raise ValueError("simulated link delay must fit its declared bound")
    parent, child = mp.get_context("spawn").Pipe()
    process = mp.get_context("spawn").Process(target=worker, args=(child, snapshot, boxes, payload, initial, args.seed, args.stage, args.braking_duration_s, not args.disable_braking_fallback))
    report = {"schema_version": 3, "success": False, "source": "wall_clock_simulation", "hardware_commands": False,
              "scenario": args.scenario, "stage": args.stage, "round": args.round, "seed": args.seed, "fault": args.fault,
              "obstacle_center_override_map_m": args.obstacle_center,
              "obstacle_velocity_override_map_m_s": args.obstacle_velocity,
              "obstacle_size_m": args.obstacle_size,
              "braking_duration_s": args.braking_duration_s,
              "braking_fallback_enabled": not args.disable_braking_fallback,
              "schedule": asdict(schedule), "initial_snapshot": snapshot.to_dict(),
              "initial_trajectory": initial, "task_boxes": boxes, "payload": payload,
              "initial_cycle_path": str(args.initial_cycle.resolve()), "solves": [], "observations": [], "feedbacks": [], "samples": [],
              "trajectories": [], "worker_events": [], "dynamics_enabled": False, "torque_constraints_enabled": False,
              "runtime_source_sha256": runtime_source_hashes(Path(__file__).parent),
              "initial_cycle_sha256": hashlib.sha256(args.initial_cycle.read_bytes()).hexdigest()}
    process.start(); child.close()
    commands, replies, stop_io = queue.Queue(maxsize=1), queue.Queue(maxsize=8), threading.Event()
    io_thread = threading.Thread(target=exchange_worker, args=(parent, commands, replies, stop_io),
                                 name="motion292-worker-transport", daemon=True)
    io_thread.start()
    segment = None
    try:
        try:
            startup = replies.get(timeout=180)
        except queue.Empty as error:
            raise TimeoutError("worker initialization timed out") from error
        report["startup"] = startup
        if not startup.get("ready"):
            raise RuntimeError(startup.get("error", "worker failed to initialize"))
        started = time.perf_counter()
        segment = RollingSegment(initial, schedule, started,
                                 max_scene_age_s=schedule.period_s + args.compute_bound_s + args.link_bound_s,
                                 max_feedback_age_s=args.max_feedback_age_s)
        report["started_at"] = started
        def complete(received_at):
            if not any(event["event"] in ("terminal_validated", "terminal_certificate_reused")
                       for event in segment.events):
                raise RuntimeError("segment ended without terminal safety verification")
            endpoint_at = segment.active_at + segment.active.duration
            report["final_feedback"] = {
                "source": "analytic_simulator", "sampled_at": endpoint_at, "received_at": received_at,
                "trajectory_id": segment.trajectory_id, "joint_names": list(segment.active.joint_names),
                "state": {key: value.tolist() for key, value in segment.active.sample(segment.active.duration).items()}}
            report["success"] = True

        next_sample = started
        certificate = None
        while True:
            tick = started + segment.sequence * schedule.period_s
            now = time.perf_counter()
            if now - started > args.execution_timeout_s:
                raise TimeoutError("segment execution timeout")
            if now >= segment.active_at + segment.active.duration:
                complete(now)
                break
            expected = segment.state(now)
            obstacle_center = args.obstacle_center if args.obstacle_center is not None else startup["obstacle_center"]
            observation = observe(snapshot, obstacle_center, args.scenario, segment.sequence,
                                  now, now-started, curve.duration, expected, args.obstacle_velocity, args.obstacle_size)
            feedback = ExecutionFeedback(segment.trajectory_id, segment.active.joint_names, now, expected)
            fault = args.fault if segment.sequence == 0 else "none"
            if fault == "stale_scene":
                observation = replace(observation, observed_at=now-segment.max_scene_age_s-1.)
            if fault == "feedback_error":
                feedback.state["positions"] = feedback.state["positions"] + .1
            report["observations"].append({**asdict(observation), "identity": observation.identity})
            report["feedbacks"].append({"trajectory_id": feedback.trajectory_id, "sampled_at": feedback.sampled_at,
                                       "joint_names": feedback.joint_names,
                                       "state": {k: v.tolist() for k, v in feedback.state.items()}})
            ticket = segment.begin(observation, feedback, now)
            terminal = ticket is None
            boundary = segment.active_at + segment.active.duration if terminal else ticket.splice_at
            # The terminal endpoint is already known in relative trajectory time.
            # Adding/subtracting a large wall-clock epoch can round it just past
            # duration; do not manufacture an out-of-range final interval.
            prefix_end = segment.active.duration if terminal else boundary-segment.active_at
            prefix = segment.active.window(now-segment.active_at, prefix_end)
            forecast_duration = boundary-now if terminal else observation.valid_until-boundary
            reuse = (terminal and certificate is not None and
                     certificate["valid_from"] <= now <= boundary <= certificate["valid_until"] and
                     observation.covered_by(SceneSnapshot.from_dict(certificate["snapshot"]), now, boundary))
            if reuse:
                checked_at = time.perf_counter()
                if checked_at > min(now + args.compute_bound_s, boundary):
                    raise TimeoutError("terminal cached validation missed deadline")
                segment.events.append({"event": "terminal_certificate_reused", "at": checked_at,
                                       "scene_id": observation.identity,
                                       "available_latency_s": checked_at-observation.observed_at,
                                       "communication_required": False})
            else:
                requested_at = time.perf_counter()
                commands.put_nowait((ticket, observation, prefix, forecast_duration,
                                     (args.seed + segment.sequence) % 2**32, fault))
            deadline = min(now + args.compute_bound_s, boundary-args.link_bound_s) if terminal else ticket.compute_deadline
            result = {"success": True, "ready_at": now, "terminal": True} if reuse else None
            arrived, installed = (now if reuse else None), reuse
            while True:
                now = time.perf_counter()
                # A reply may legitimately spend t3 in transit AFTER computation
                # finishes at its t2 deadline. The complete-message deadline is
                # t2+t3; the reported ready_at is checked separately when received.
                if result is None and now > deadline + args.link_bound_s:
                    raise TimeoutError("worker reply deadline miss (supervisor)")
                message = None
                if result is None:
                    try:
                        message = replies.get_nowait()
                    except queue.Empty:
                        pass
                if message is not None:
                    if "progress" in message:
                        report["worker_events"].append(message["progress"])
                        continue
                    result = message
                    received_at = time.perf_counter()
                    record_reply_timing(result, requested_at, received_at)
                    arrived = received_at + args.link_delay_s
                    if fault == "link_delay":
                        arrived += schedule.period_s + args.link_bound_s
                    report["solves"].append(result)
                    if not result.get("success"):
                        report["failure_code"] = result.get("failure_code", "PLAN_REJECTED")
                        report["required_action"] = result.get("required_action", "independent_safety_intervention")
                        raise RuntimeError(result.get("error", "planning failed"))
                    if not terminal and (result.get("sequence") != ticket.sequence or
                                         result.get("observation_id") != observation.identity):
                        raise ValueError("worker returned an obsolete task or scene")
                if result is not None and not installed and now > result["ready_at"] + args.link_bound_s:
                    raise TimeoutError("communication deadline miss before splice")
                if result is not None and not installed and now >= arrived:
                    if terminal:
                        if (result["ready_at"] > deadline or now > boundary or
                                now-result["ready_at"] > args.link_bound_s):
                            raise TimeoutError("terminal revalidation missed deadline")
                        segment.events.append({"event": "terminal_validated", "at": now,
                                               "scene_id": observation.identity,
                                               "safe_ready_at": result["ready_at"],
                                               "available_latency_s": result["ready_at"]-observation.observed_at,
                                               "accepted_latency_s": now-observation.observed_at,
                                               "trajectory_id": segment.trajectory_id})
                    else:
                        if fault == "admission_delay":
                            time.sleep(schedule.period_s)  # simulate a blocked contract validator
                        segment.stage(ticket, result["trajectory"], observation, result["ready_at"], now,
                                      previous_suffix_valid=result["previous_suffix_valid"],
                                      admission_clock=time.perf_counter)
                        now = time.perf_counter()
                    installed = True
                if now >= boundary:
                    if not installed:
                        raise TimeoutError("communication deadline miss: replacement not delivered by splice")
                    if terminal:
                        complete(now)
                    else:
                        # Analytic simulator executes a buffered switch at its specified
                        # boundary; the supervisor receives that synthetic feedback now.
                        feedback = ExecutionFeedback(segment.trajectory_id, segment.active.joint_names,
                                                     boundary, segment.state(boundary))
                        if fault == "scene_changed":
                            observation = replace(observation, snapshot=replace(
                                observation.snapshot, revision=observation.snapshot.revision+1))
                        segment.activate(feedback, observation, boundary)
                        certificate = result["certificate"]
                        segment.events[-1]["feedback_received_at"] = now
                        segment.events[-1]["feedback_latency_s"] = now-observation.observed_at
                        report["trajectories"].append({"active_at": boundary,
                                                       "trajectory": result["trajectory"]})
                    break
                if now >= next_sample:
                    report["samples"].append({"at": now, "trajectory_id": segment.trajectory_id,
                                               **{k: v.tolist() for k, v in segment.state(now).items()}})
                    next_sample = now + .02
                time.sleep(min(.002, max(0., boundary-time.perf_counter())))
            if terminal:
                break
        report["completed_at"] = time.perf_counter()
    except (Exception, KeyboardInterrupt) as error:
        report["error"] = f"{type(error).__name__}: {error}"
        if isinstance(error, TimeoutError) and "failure_code" not in report:
            report["failure_code"] = "DEADLINE_MISS"
            report["required_action"] = "independent_safety_intervention"
        if segment is not None:
            segment.fail(report["error"], time.perf_counter())
        report["safety_action"] = "simulation_aborted; physical stopping NOT validated"
    finally:
        stop_io.set()
        if process.is_alive():
            process.terminate()  # only this run's worker, including obsolete in-flight CUDA work
        process.join(10)
        io_thread.join(1)
        parent.close()
        report["events"] = segment.events if segment else []
        fields = {
            "compute_wall_s": [x["compute_wall_s"] for x in report["solves"] if "compute_wall_s" in x],
            "available_latency_s": [x["available_latency_s"] for x in report["events"] if "available_latency_s" in x],
            "accepted_latency_s": [x["accepted_latency_s"] for x in report["events"] if "accepted_latency_s" in x],
            "feedback_latency_s": [x["feedback_latency_s"] for x in report["events"] if "feedback_latency_s" in x],
        }
        for result in report["solves"]:
            for name in ("queue_ipc_s", "request_to_ready_s", "result_ipc_s"):
                if name in result:
                    fields.setdefault(name, []).append(result[name])
            fields.setdefault("sdk_solve_s", []).extend(result.get("sdk_solve_times_s", []))
            for name, (start, end) in result.get("spans", {}).items():
                fields.setdefault(f"stage_{name}_s", []).append(end-start)
        report["statistics"] = {key: {"count": len(values), "median": float(np.median(values)),
                                            "p95": float(np.percentile(values, 95)), "max": max(values)}
                                for key, values in fields.items() if values}
        report["trigger_count"] = sum(e["event"] == "requested" for e in report["events"])
        report["splice_count"] = sum(e["event"] == "activated" for e in report["events"])
        report["optimization_success_count"] = sum(x.get("optimization_success", False) for x in report["solves"])
        report["direct_optimization_success_count"] = sum(x.get("strategy") == "optimized_suffix" and x.get("success", False)
                                                         for x in report["solves"])
        report["revalidated_fallback_count"] = sum(x.get("strategy") in ("revalidated_suffix", "revalidated_constrained_suffix")
                                                  for x in report["solves"])
        report["constrained_revalidation_count"] = sum(x.get("strategy") == "revalidated_constrained_suffix"
                                                         for x in report["solves"])
        installed_sequences = {event["sequence"] for event in report["events"] if event["event"] == "activated"}
        report["braking_candidate_count"] = sum(x.get("strategy") == "brake_then_optimize" and x.get("success", False)
                                                 for x in report["solves"])
        report["braking_replan_count"] = sum(x.get("strategy") == "brake_then_optimize" and x.get("success", False)
                                              and x.get("sequence") in installed_sequences for x in report["solves"])
        report["direct_splice_count"] = sum(x.get("strategy") == "optimized_suffix" and x.get("sequence") in installed_sequences
                                             for x in report["solves"])
        report["revalidated_splice_count"] = sum(x.get("strategy") == "revalidated_suffix" and x.get("sequence") in installed_sequences
                                                  for x in report["solves"])
        report["deadline_miss_count"] = sum(x.get("deadline_miss", False) for x in report["solves"])
        if "deadline" in report.get("error", "") and not report["deadline_miss_count"]:
            report["deadline_miss_count"] = 1
        report["cancelled_count"] = sum(e["event"] == "safety_required" and e.get("cancelled_sequence") is not None
                                        for e in report["events"])
        report["censored_compute_count"] = report["trigger_count"] - sum("compute_wall_s" in x and not x.get("terminal", False)
                                                                     for x in report["solves"])
        report["collision_rejection_count"] = sum("collision" in x.get("previous_suffix_rejection", "")
                                                   for x in report["solves"])
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({k: report[k] for k in ("success", "trigger_count", "splice_count", "direct_optimization_success_count",
                                            "braking_replan_count", "revalidated_fallback_count", "statistics")},
                     ensure_ascii=False), flush=True)
    if not report["success"]:
        raise SystemExit(report["error"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--initial-cycle", type=Path, required=True)
    parser.add_argument("--round", type=int, default=0, help="zero-based round for a full-sequence input")
    parser.add_argument("--stage", choices=("approach", "contact_approach", "extract", "transport", "conveyor_place", "return_home"), default="approach")
    parser.add_argument("--scenario", choices=("clear", "static", "moving", "appear", "depart"), default="clear")
    parser.add_argument("--obstacle-center", type=float, nargs=3,
                        help="explicit map-frame obstacle center at 0.6*t1; overrides the default TCP fixture")
    parser.add_argument("--obstacle-velocity", type=float, nargs=3,
                        help="map-frame m/s for moving scenario; default is 0 0.025 0")
    parser.add_argument("--obstacle-size", type=float, nargs=3, default=(.08, .08, .08),
                        help="map-frame box dimensions in meters")
    parser.add_argument("--fault", choices=("none", "compute_delay", "link_delay", "stale_scene",
                                           "feedback_error", "scene_changed", "worker_failure", "solver_failure", "admission_delay"),
                        default="none", help="simulation-only failure injection, at first tick")
    parser.add_argument("--compute-bound-s", type=float, required=True)
    parser.add_argument("--link-bound-s", type=float, required=True)
    parser.add_argument("--link-delay-s", type=float, default=0.)
    parser.add_argument("--slices", type=int)
    parser.add_argument("--execution-timeout-s", type=float, default=120.)
    parser.add_argument("--disable-braking-fallback", action="store_true", help="controlled comparison: require direct moving-boundary optimization")
    parser.add_argument("--braking-duration-s", type=float, default=.5,
                        help="validated internal deceleration fallback duration, not a hardware stop command")
    parser.add_argument("--max-feedback-age-s", type=float, default=.1)
    parser.add_argument("--seed", type=int, default=20261003)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if (not math.isfinite(args.execution_timeout_s) or args.execution_timeout_s <= 0 or
            not math.isfinite(args.max_feedback_age_s) or args.max_feedback_age_s <= 0 or
            not math.isfinite(args.braking_duration_s) or args.braking_duration_s <= 0 or
            not 0 <= args.seed < 2**32 or args.round < 0):
        parser.error("positive finite execution/freshness limits and unsigned 32-bit seed required")
    if args.obstacle_center is not None and not np.isfinite(args.obstacle_center).all():
        parser.error("obstacle center must contain three finite map coordinates")
    if args.obstacle_velocity is not None and not np.isfinite(args.obstacle_velocity).all():
        parser.error("obstacle velocity must contain three finite map components")
    if not np.isfinite(args.obstacle_size).all() or np.any(np.asarray(args.obstacle_size) <= 0):
        parser.error("obstacle size must contain three positive finite dimensions")
    run(args)


if __name__ == "__main__":
    main()
