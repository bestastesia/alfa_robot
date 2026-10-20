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
from curobo_core.scene import Pose, SceneObject, SceneSnapshot, digest
from curobo_core.trajectory import TimedTrajectory


PROGRESS_PREFIX = "MOTION292_PROGRESS "


def progress_line(event, **fields):
    return PROGRESS_PREFIX + json.dumps({"event": event, **fields}, ensure_ascii=False, separators=(",", ":"))


def publish_progress(args, event, **fields):
    if getattr(args, "stream_progress", False):
        print(progress_line(event, **fields), flush=True)


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


def worker(connection, snapshot=None, boxes=None, payload=None, initial=None, random_seed=0,
           phase=None, braking_duration_s=.5, allow_braking_fallback=True,
           compute_bound_s=1., link_bound_s=.15, admission_bound_s=.05,
           first_compute_bound_s=None):
    """Own one CUDA backend; optional stage arguments preserve the legacy one-shot CLI."""
    from curobo_core.rolling_backend import RollingBackend
    from curobo_core.rolling_curobo import SegmentReplanner
    from curobo.types import JointState
    from curobo_core.adapter import pose_matrix
    import torch
    import curobo

    try:
        service_started = time.perf_counter()
        assets = PlannerAssets()
        backend = RollingBackend(assets)
        warmed = set()
        metadata = {
            "model_id": backend.model_id,
            "hardware": {"gpu": torch.cuda.get_device_name(), "platform": platform.platform()},
            "packages": {**{name: importlib.metadata.version(name) for name in ("torch", "numpy", "scipy")},
                         "curobo": getattr(curobo, "__version__", "unpackaged source")},
            "curobo_module": str(curobo.__file__),
            "model_assets_sha256": {name: hashlib.sha256(path.read_bytes()).hexdigest()
                                    for name, path in vars(assets).items() if isinstance(path, Path)},
            "base_cuboids_sha256": hashlib.sha256((Path(__file__).resolve().parents[1] /
                "generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json").read_bytes()).hexdigest(),
        }
        adapter = validity = None

        def configure(config, allow_warm):
            nonlocal adapter, validity
            began = time.perf_counter()
            stage_snapshot = config["snapshot"]
            stage_boxes = config["boxes"]
            stage_payload = config["payload"]
            stage_initial = config["initial"]
            stage_phase = config["phase"]
            seed = config["seed"]
            torch.manual_seed(seed)
            np.random.seed(seed)
            backend.configure_task(stage_boxes)
            adapter = SegmentReplanner(
                backend, stage_boxes, payload=stage_payload, phase=stage_phase,
                braking_duration_s=config["braking_duration_s"],
                allow_braking_fallback=config["allow_braking_fallback"])
            validity = adapter.scene(stage_snapshot)
            initial_validation = adapter.validate(stage_initial, validity)
            curve = TimedTrajectory(stage_initial)
            optimizer_key = backend.trajectory_optimizer_key(stage_boxes, stage_payload)
            warm = {"reused": optimizer_key in warmed, "optimizer_key": digest(optimizer_key),
                    "first_compute_bound_s": config["first_compute_bound_s"]}
            if allow_warm and optimizer_key not in warmed and stage_phase not in ("contact_approach", "extract"):
                schedule = SliceSchedule(curve.duration, config["compute_bound_s"],
                                         config["link_bound_s"], None,
                                         config["admission_bound_s"])
                cut = schedule.period_s
                first_started = time.perf_counter()
                _, first = backend.optimize_rrt_path(
                    curve.suffix(cut)["positions"], validity, stage_boxes,
                    payload=stage_payload, start_state=curve.sample(cut))
                first_wall_s = time.perf_counter() - first_started
                first.pop("trajectory", None)
                warm.update(first_planning=first, first_wall_s=first_wall_s,
                            first_deadline_miss=first_wall_s > config["first_compute_bound_s"])
                warmed.add(optimizer_key)
            elif stage_phase in ("contact_approach", "extract"):
                warm["not_applicable"] = "constrained phase has no TrajOpt first-call"
            q = torch.tensor(curve.sample(curve.duration * .6)["positions"],
                             device="cuda", dtype=torch.float32).reshape(1, 1, -1)
            state = validity.checker.kinematics.compute_kinematics(
                JointState.from_position(q, joint_names=list(ACTIVE_JOINTS)))
            tool = state.tool_poses.to_dict()[next(iter(stage_boxes), "left") + "_tool0"]
            local_center = tool.position.reshape(-1, 3)[0].cpu().numpy()
            center = (pose_matrix(stage_snapshot.state.base_pose) @ np.r_[local_center, 1.])[:3].tolist()
            return {"ready": True, "initialization_s": time.perf_counter() - began,
                    "validation": initial_validation, "warmup": warm,
                    "obstacle_center": center, **metadata}

        if initial is not None:
            connection.send(configure({
                "snapshot": snapshot, "boxes": boxes, "payload": payload, "initial": initial,
                "seed": random_seed, "phase": phase, "braking_duration_s": braking_duration_s,
                "allow_braking_fallback": allow_braking_fallback,
                "compute_bound_s": compute_bound_s, "link_bound_s": link_bound_s,
                "admission_bound_s": admission_bound_s,
                "first_compute_bound_s": (compute_bound_s + 1.5 if first_compute_bound_s is None
                                            else first_compute_bound_s),
            }, True))
        else:
            connection.send({"service_ready": True,
                             "initialization_s": time.perf_counter() - service_started, **metadata})

        while True:
            command = connection.recv()
            if command is None:
                break
            if isinstance(command, dict):
                if command.get("op") == "prime":
                    began = time.perf_counter()
                    details = [configure(config, True) for config in command["stages"]]
                    adapter = validity = None
                    connection.send({"prime_ready": True, "wall_s": time.perf_counter() - began,
                                     "warmed_topologies": sorted(warmed), "stages": details})
                    continue
                if command.get("op") == "start_stage":
                    connection.send(configure(command["stage"], False))
                    continue
                raise ValueError(f"unknown persistent worker command: {command.get('op')}")
            if adapter is None:
                raise ValueError("solve received without an active stage")
            ticket, observation, prefix, duration, seed, fault = command
            if fault == "compute_delay":
                time.sleep(max(0., ticket.compute_deadline - time.perf_counter()) + .2)
            if fault == "worker_failure":
                os._exit(91)
            if fault == "solver_failure":
                connection.send({"success": False, "error": "injected solver failure", "sequence": ticket.sequence,
                                 "ready_at": time.perf_counter()})
                continue
            if ticket is None:
                began = time.perf_counter()
                terminal_snapshot = observation.swept_snapshot(
                    observation.observed_at, observation.observed_at + duration)
                certificate = {"snapshot": terminal_snapshot.to_dict(), "valid_from": observation.observed_at,
                               "valid_until": observation.observed_at + duration}
                response = {"success": False, "trajectory": prefix, "certificate": certificate}
                try:
                    check = adapter.scene(terminal_snapshot)
                    response.update(success=True, validation=adapter.validate(prefix, check))
                except (ValueError, RuntimeError) as error:
                    response.update(error=str(error), failure_code="UNSAFE_COMMITTED_PREFIX",
                                    required_action="independent_protective_stop")
                ready = time.perf_counter()
                response.update(started_at=began, ready_at=ready, compute_wall_s=ready-began,
                                observation_id=observation.identity, terminal=True)
            else:
                response = adapter.solve(
                    ticket, observation, prefix, max_duration_s=duration, search_seed=seed,
                    progress=lambda event: connection.send({"progress": {
                        "sequence": ticket.sequence, "observation_id": observation.identity, **event}}))
            connection.send(response)
    except (Exception, EOFError) as error:
        try:
            connection.send({"success": False, "error": f"worker: {type(error).__name__}: {error}"})
        except (BrokenPipeError, OSError):
            pass
    finally:
        connection.close()


def start_persistent_worker(timeout_s=180.):
    parent, child = mp.get_context("spawn").Pipe()
    process = mp.get_context("spawn").Process(target=worker, args=(child,))
    process.start(); child.close()
    commands, replies, stop_io = queue.Queue(maxsize=1), queue.Queue(maxsize=8), threading.Event()
    io_thread = threading.Thread(target=exchange_worker, args=(parent, commands, replies, stop_io),
                                 name="motion292-persistent-worker-transport", daemon=True)
    io_thread.start()
    try:
        service = replies.get(timeout=timeout_s)
    except queue.Empty as error:
        process.terminate(); process.join(10); stop_io.set(); parent.close()
        raise TimeoutError("persistent worker initialization timed out") from error
    if not service.get("service_ready"):
        process.terminate(); process.join(10); stop_io.set(); parent.close()
        raise RuntimeError(service.get("error", "persistent worker failed to initialize"))
    return {"parent": parent, "process": process, "commands": commands, "replies": replies,
            "stop_io": stop_io, "io_thread": io_thread, "service": service,
            "poisoned": False, "primed": False}


def persistent_stage_config(initial, snapshot, boxes, payload, args):
    return {"snapshot": snapshot, "boxes": boxes, "payload": payload, "initial": initial,
            "seed": args.seed, "phase": args.stage, "braking_duration_s": args.braking_duration_s,
            "allow_braking_fallback": not args.disable_braking_fallback,
            "compute_bound_s": args.compute_bound_s, "link_bound_s": args.link_bound_s,
            "admission_bound_s": getattr(args, "admission_bound_s", .05),
            "first_compute_bound_s": getattr(args, "first_compute_bound_s", args.compute_bound_s + 1.5)}


def prime_persistent_worker(shared, stages, timeout_s=240.):
    shared["commands"].put({"op": "prime", "stages": stages})
    try:
        result = shared["replies"].get(timeout=timeout_s)
    except queue.Empty as error:
        shared["poisoned"] = True
        raise TimeoutError("persistent worker warmup timed out") from error
    if not result.get("prime_ready"):
        shared["poisoned"] = True
        raise RuntimeError(result.get("error", "persistent worker warmup failed"))
    shared["primed"] = True
    return result


def close_persistent_worker(shared):
    if shared["process"].is_alive() and not shared.get("poisoned"):
        try:
            shared["commands"].put(None, timeout=1.)
        except queue.Full:
            shared["poisoned"] = True
    shared["process"].join(10)
    if shared["process"].is_alive():
        shared["process"].terminate(); shared["process"].join(10)
    shared["stop_io"].set()
    shared["io_thread"].join(1)
    shared["parent"].close()


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


def run(args, shared=None):
    initial, snapshot, boxes, payload = stage_input(json.loads(args.initial_cycle.read_text()), args.stage, args.round)
    curve = TimedTrajectory(initial)
    configured_first_bound = getattr(args, "first_compute_bound_s", args.compute_bound_s + 1.5)
    effective_first_bound = (args.compute_bound_s if args.stage in ("contact_approach", "extract")
                             else configured_first_bound)
    schedule = SliceSchedule(curve.duration, args.compute_bound_s, args.link_bound_s, args.slices,
                             getattr(args, "admission_bound_s", .05), effective_first_bound)
    if not 0 <= args.link_delay_s <= args.link_bound_s:
        raise ValueError("simulated link delay must fit its declared bound")
    owns_worker = shared is None
    if owns_worker:
        parent, child = mp.get_context("spawn").Pipe()
        process = mp.get_context("spawn").Process(target=worker, args=(
            child, snapshot, boxes, payload, initial, args.seed, args.stage,
            args.braking_duration_s, not args.disable_braking_fallback,
            args.compute_bound_s, args.link_bound_s, getattr(args, "admission_bound_s", .05),
            configured_first_bound))
    else:
        if not shared.get("primed"):
            raise ValueError("persistent worker must be primed before stage execution")
        parent, process = shared["parent"], shared["process"]
    report = {"schema_version": 3, "success": False, "source": "wall_clock_simulation", "hardware_commands": False,
              "scenario": args.scenario, "stage": args.stage, "round": args.round, "seed": args.seed, "fault": args.fault,
              "obstacle_center_override_map_m": args.obstacle_center,
              "obstacle_velocity_override_map_m_s": args.obstacle_velocity,
              "obstacle_size_m": args.obstacle_size,
              "braking_duration_s": args.braking_duration_s,
              "braking_fallback_enabled": not args.disable_braking_fallback,
              "schedule": {**asdict(schedule), "configured_first_compute_bound_s": configured_first_bound},
              "initial_snapshot": snapshot.to_dict(),
              "initial_trajectory": initial, "task_boxes": boxes, "payload": payload,
              "initial_cycle_path": str(args.initial_cycle.resolve()), "solves": [], "observations": [], "feedbacks": [], "samples": [],
              "trajectories": [], "worker_events": [], "dynamics_enabled": False, "torque_constraints_enabled": False,
              "runtime_source_sha256": runtime_source_hashes(Path(__file__).parent),
              "initial_cycle_sha256": hashlib.sha256(args.initial_cycle.read_bytes()).hexdigest()}
    if owns_worker:
        process.start(); child.close()
        commands, replies, stop_io = queue.Queue(maxsize=1), queue.Queue(maxsize=8), threading.Event()
        io_thread = threading.Thread(target=exchange_worker, args=(parent, commands, replies, stop_io),
                                     name="motion292-worker-transport", daemon=True)
        io_thread.start()
    else:
        commands, replies = shared["commands"], shared["replies"]
        commands.put({"op": "start_stage", "stage": persistent_stage_config(
            initial, snapshot, boxes, payload, args)})
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
                                 max_scene_age_s=(max(schedule.period_for(0), schedule.period_s) + args.compute_bound_s +
                                                  args.link_bound_s + schedule.admission_bound_s),
                                 max_feedback_age_s=args.max_feedback_age_s)
        report["started_at"] = started
        publish_progress(args, "stage_started", stage=args.stage, round=args.round,
                         started_at=started, initial_snapshot=report["initial_snapshot"],
                         initial_trajectory=initial, schedule=report["schedule"])
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
            observed = {**asdict(observation), "identity": observation.identity}
            report["observations"].append(observed)
            rolling_obstacle = next((item for item in observed["snapshot"]["objects"]
                                     if item["object_id"] == "rolling_obstacle"), None)
            publish_progress(args, "observation", stage=args.stage, elapsed_s=now-started,
                             obstacle=rolling_obstacle,
                             obstacle_velocity=dict(observed["velocities"]).get("rolling_obstacle"))
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
            deadline = min(now + args.compute_bound_s,
                           boundary - args.link_bound_s - schedule.admission_bound_s) if terminal else ticket.compute_deadline
            active_result = {"success": True, "ready_at": now, "terminal": True} if reuse else None
            installed = reuse
            worker_done = reuse
            candidate = None
            candidate_arrived = None
            candidate_staged = reuse
            safe_fallback = None
            while True:
                now = time.perf_counter()
                # A validated current suffix is staged as soon as its compact
                # certificate arrives. Optional TrajOpt may then use the remaining
                # window, but its timeout cannot invalidate that already-safe path.
                if not worker_done and not installed and now > deadline + args.link_bound_s:
                    raise TimeoutError("worker reply deadline miss (supervisor)")
                message = None
                if not worker_done:
                    try:
                        message = replies.get_nowait()
                    except queue.Empty:
                        pass
                if message is not None:
                    if "progress" in message:
                        detail = message["progress"]
                        if (not terminal and (detail.get("sequence") != ticket.sequence or
                                              detail.get("observation_id") != observation.identity)):
                            report.setdefault("late_worker_events", []).append(detail)
                            continue
                        report["worker_events"].append(detail)
                        publish_progress(args, "planner_progress", stage=args.stage, detail=detail)
                        if not terminal and detail.get("phase") == "safe_suffix":
                            received_at = time.perf_counter()
                            safe_arrived = received_at + args.link_delay_s
                            if fault == "link_delay":
                                safe_arrived += schedule.period_s + args.link_bound_s
                            safe_snapshot = observation.swept_snapshot(
                                detail["valid_from"], detail["valid_until"])
                            safe_fallback = {
                                "success": True, "sequence": ticket.sequence,
                                "observation_id": observation.identity,
                                "previous_suffix_valid": True, "optimization_success": False,
                                "strategy": "revalidated_suffix", "trajectory": ticket.remaining,
                                "certificate": {"snapshot": safe_snapshot.to_dict(),
                                                "valid_from": detail["valid_from"],
                                                "valid_until": detail["valid_until"]},
                                "started_at": detail["started_at"], "ready_at": detail["at"],
                                "deadline_miss": False, "received_at": received_at,
                                "arrived_at": safe_arrived,
                            }
                        continue
                    if (not terminal and (message.get("sequence") != ticket.sequence or
                                          message.get("observation_id") != observation.identity)):
                        report.setdefault("late_worker_results", []).append({
                            key: value for key, value in message.items()
                            if key not in ("trajectory", "certificate", "prefix_certificate",
                                           "planning_certificate", "validation")})
                        continue
                    worker_done = True
                    received_at = time.perf_counter()
                    record_reply_timing(message, requested_at, received_at)
                    arrived = received_at + args.link_delay_s
                    if fault == "link_delay":
                        arrived += schedule.period_s + args.link_bound_s
                    report["solves"].append(message)
                    publish_progress(args, "solve_finished", stage=args.stage,
                                     sequence=message.get("sequence"), terminal=bool(message.get("terminal")),
                                     success=bool(message.get("success")), strategy=message.get("strategy"),
                                     compute_wall_s=message.get("compute_wall_s"),
                                     request_to_ready_s=message.get("request_to_ready_s"),
                                     sdk_solve_times_s=message.get("sdk_solve_times_s", []),
                                     failure_code=message.get("failure_code"))
                    if not message.get("success"):
                        report["failure_code"] = message.get("failure_code", "PLAN_REJECTED")
                        report["required_action"] = message.get("required_action", "independent_safety_intervention")
                        raise RuntimeError(message.get("error", "planning failed"))
                    if message.get("strategy") == "deadline_revalidated_suffix" and installed:
                        active_result = message
                    else:
                        candidate, candidate_arrived = message, arrived
                if safe_fallback is not None and not installed and now >= safe_fallback["arrived_at"]:
                    if fault == "admission_delay":
                        time.sleep(schedule.period_s)
                    segment.stage(ticket, ticket.remaining, observation, safe_fallback["ready_at"],
                                  safe_fallback["arrived_at"], previous_suffix_valid=True,
                                  admission_clock=time.perf_counter)
                    installed = True
                    active_result = safe_fallback
                    now = time.perf_counter()
                if candidate is not None and not candidate_staged and now > candidate["ready_at"] + args.link_bound_s:
                    if installed and candidate.get("previous_suffix_valid"):
                        candidate["optional_communication_deadline_miss"] = True
                        candidate = None
                    else:
                        raise TimeoutError("communication deadline miss before splice")
                if candidate is not None and not candidate_staged and now >= candidate_arrived:
                    if terminal:
                        received_at = now
                        if (candidate["ready_at"] > deadline or received_at > boundary or
                                received_at-candidate["ready_at"] > args.link_bound_s):
                            raise TimeoutError("terminal revalidation missed communication deadline")
                        admitted_at = time.perf_counter()
                        if (admitted_at > boundary or
                                admitted_at-received_at > schedule.admission_bound_s):
                            raise TimeoutError("terminal revalidation missed admission deadline")
                        segment.events.append({"event": "terminal_validated", "at": admitted_at,
                                               "scene_id": observation.identity,
                                               "safe_ready_at": candidate["ready_at"],
                                               "received_at": received_at,
                                               "transport_latency_s": received_at-candidate["ready_at"],
                                               "admission_latency_s": admitted_at-received_at,
                                               "available_latency_s": candidate["ready_at"]-observation.observed_at,
                                               "accepted_latency_s": admitted_at-observation.observed_at,
                                               "trajectory_id": segment.trajectory_id})
                    else:
                        if fault == "admission_delay":
                            time.sleep(schedule.period_s)
                        segment.stage(ticket, candidate["trajectory"], observation, candidate["ready_at"], now,
                                      previous_suffix_valid=candidate["previous_suffix_valid"],
                                      admission_clock=time.perf_counter, replace_staged=installed)
                        now = time.perf_counter()
                    installed = candidate_staged = True
                    active_result = candidate
                if now >= boundary:
                    if not installed:
                        raise TimeoutError("communication deadline miss: replacement not delivered by splice")
                    if not terminal and active_result is safe_fallback:
                        safe_result = dict(safe_fallback)
                        safe_result.pop("received_at", None)
                        safe_result.pop("arrived_at", None)
                        safe_result["optional_optimization_pending"] = not worker_done
                        safe_result["optional_candidate_discarded"] = candidate is None and worker_done
                        record_reply_timing(safe_result, requested_at, safe_fallback["received_at"])
                        report["solves"].append(safe_result)
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
                        certificate = active_result["certificate"]
                        segment.events[-1]["feedback_received_at"] = now
                        segment.events[-1]["feedback_latency_s"] = now-observation.observed_at
                        segment.events[-1]["strategy"] = active_result.get("strategy")
                        report["trajectories"].append({"active_at": boundary,
                                                       "trajectory": active_result["trajectory"]})
                        publish_progress(args, "trajectory_activated", stage=args.stage,
                                         sequence=segment.sequence - 1, active_at=boundary,
                                         strategy=active_result.get("strategy"),
                                         trajectory=active_result["trajectory"])
                    break
                if now >= next_sample:
                    sample = {"at": now, "trajectory_id": segment.trajectory_id,
                              **{k: v.tolist() for k, v in segment.state(now).items()}}
                    report["samples"].append(sample)
                    publish_progress(args, "motion_sample", stage=args.stage,
                                     elapsed_s=now-started, joint_names=list(segment.active.joint_names),
                                     positions=sample["positions"])
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
        if owns_worker:
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
            "transport_latency_s": [x["transport_latency_s"] for x in report["events"] if "transport_latency_s" in x],
            "admission_latency_s": [x["admission_latency_s"] for x in report["events"] if "admission_latency_s" in x],
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
        report["revalidated_fallback_count"] = sum(x.get("strategy") in (
            "revalidated_suffix", "revalidated_constrained_suffix", "deadline_revalidated_suffix")
            for x in report["solves"])
        report["optional_timeout_skip_count"] = sum(
            x.get("strategy") == "deadline_revalidated_suffix" for x in report["solves"])
        report["constrained_revalidation_count"] = sum(x.get("strategy") == "revalidated_constrained_suffix"
                                                         for x in report["solves"])
        installed_strategies = {event["sequence"]: event.get("strategy") for event in report["events"]
                                if event["event"] == "activated"}
        report["braking_candidate_count"] = sum(x.get("strategy") == "brake_then_optimize" and x.get("success", False)
                                                 for x in report["solves"])
        report["braking_replan_count"] = sum(strategy == "brake_then_optimize"
                                              for strategy in installed_strategies.values())
        report["direct_splice_count"] = sum(strategy == "optimized_suffix"
                                             for strategy in installed_strategies.values())
        report["revalidated_splice_count"] = sum(strategy in (
            "revalidated_suffix", "deadline_revalidated_suffix")
            for strategy in installed_strategies.values())
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
        motion_wall_s = (report.get("completed_at", 0.) - report.get("started_at", 0.)
                         if "completed_at" in report and "started_at" in report else 0.)
        publish_progress(args, "stage_finished", stage=args.stage, success=report["success"],
                         motion_wall_s=max(0., motion_wall_s), splice_count=report["splice_count"],
                         failure_code=report.get("failure_code"))
    print(json.dumps({k: report[k] for k in ("success", "trigger_count", "splice_count", "direct_optimization_success_count",
                                            "braking_replan_count", "revalidated_fallback_count", "statistics")},
                     ensure_ascii=False), flush=True)
    if not report["success"]:
        raise RuntimeError(report["error"]) if shared is not None else SystemExit(report["error"])
    return report


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
    parser.add_argument("--first-compute-bound-s", type=float,
                        help="test/startup budget for the first exact optimizer key; default t2+1.5s")
    parser.add_argument("--link-bound-s", type=float, required=True)
    parser.add_argument("--admission-bound-s", type=float, default=.05)
    parser.add_argument("--link-delay-s", type=float, default=0.)
    parser.add_argument("--slices", type=int)
    parser.add_argument("--execution-timeout-s", type=float, default=120.)
    parser.add_argument("--disable-braking-fallback", action="store_true", help="controlled comparison: require direct moving-boundary optimization")
    parser.add_argument("--braking-duration-s", type=float, default=.5,
                        help="validated internal deceleration fallback duration, not a hardware stop command")
    parser.add_argument("--max-feedback-age-s", type=float, default=.1)
    parser.add_argument("--seed", type=int, default=20261003)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stream-progress", action="store_true",
                        help="emit line-delimited UI progress on stdout")
    args = parser.parse_args()
    if args.first_compute_bound_s is None:
        args.first_compute_bound_s = args.compute_bound_s + 1.5
    if (not math.isfinite(args.execution_timeout_s) or args.execution_timeout_s <= 0 or
            not math.isfinite(args.max_feedback_age_s) or args.max_feedback_age_s <= 0 or
            not math.isfinite(args.braking_duration_s) or args.braking_duration_s <= 0 or
            not math.isfinite(args.admission_bound_s) or args.admission_bound_s < 0 or
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
