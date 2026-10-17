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
import multiprocessing as mp
import os
from pathlib import Path
import time

import numpy as np

from curobo_core.contracts import ACTIVE_JOINTS, PlannerAssets
from curobo_core.rolling import ExecutionFeedback, RollingSegment, SceneObservation, SliceSchedule
from curobo_core.scene import Pose, SceneObject, SceneSnapshot
from curobo_core.trajectory import TimedTrajectory


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
    if locked and np.max(np.abs(np.asarray(stage["positions"])[:, locked])) > 1e-7:
        raise ValueError("moving-base segments need their own model; cannot discard those joints")
    trajectory = {"joint_names": list(ACTIVE_JOINTS), "time_from_start_s": stage["time_from_start_s"],
                  **{key: np.asarray(stage[key])[:, indices].tolist() for key in
                     ("positions", "velocities", "accelerations", "jerks")}}
    snapshot = SceneSnapshot.from_dict(record["request"]["snapshot"])
    if name in ("transport", "return_home"):
        event = "attach" if name == "transport" else "release"
        snapshot = SceneSnapshot.from_dict(next(e["snapshot"] for e in result["predicted_scene_events"]
                                                if e["phase"] == event))
    snapshot = replace(snapshot, state=snapshot.state.with_positions(
        dict(zip(ACTIVE_JOINTS, trajectory["positions"][0])), Pose()))
    return trajectory, snapshot, ({} if name == "return_home" else result["task"]), name == "transport"


def worker(connection, snapshot, boxes, payload, initial, random_seed):
    # Own the CUDA context in a separate process so the deadline supervisor does
    # not wait on a blocked solve/GIL. Process startup and warmup precede t1.
    from curobo_core.backend import CuroboBackend
    from curobo_core.rolling_curobo import SegmentReplanner
    from curobo.types import JointState
    import torch
    import curobo

    try:
        started = time.perf_counter()
        torch.manual_seed(random_seed)
        np.random.seed(random_seed)
        backend = CuroboBackend(PlannerAssets())
        adapter = SegmentReplanner(backend, boxes, payload=payload)
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
        center = tool.position.reshape(-1, 3)[0].cpu().tolist()
        connection.send({"ready": True, "initialization_s": time.perf_counter()-started,
                         "validation": initial_validation, "warmup": warm,
                         "obstacle_center": center, "model_id": backend.model_id,
                         "hardware": {"gpu": torch.cuda.get_device_name(), "platform": platform.platform()},
                         "packages": {**{name: importlib.metadata.version(name) for name in ("torch", "numpy", "scipy")},
                                      "curobo": getattr(curobo, "__version__", "unpackaged source")},
                         "curobo_module": str(curobo.__file__)})
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
                try:
                    check = adapter.scene(observation.swept_snapshot(
                        observation.observed_at, observation.observed_at + duration))
                    validation = adapter.validate(prefix, check)
                    response = {"success": True, "validation": validation}
                except (ValueError, RuntimeError) as error:
                    response = {"success": False, "error": str(error)}
                response.update(started_at=began, ready_at=time.perf_counter(), terminal=True)
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


def observe(snapshot, center, scenario, sequence, now, elapsed, duration, feedback):
    active = scenario in ("static", "moving") or (scenario == "appear" and sequence >= 1) or (
        scenario == "depart" and sequence < 2)
    objects = snapshot.objects
    velocities = ()
    if active:
        velocity = np.array([0., .025, 0.]) if scenario == "moving" else np.zeros(3)
        position = np.asarray(center) + velocity * (elapsed - duration * .6)
        objects += (SceneObject("rolling_obstacle", (.08, .08, .08), Pose(tuple(position))),)
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
    process = mp.get_context("spawn").Process(target=worker, args=(child, snapshot, boxes, payload, initial, args.seed))
    report = {"success": False, "source": "wall_clock_simulation", "hardware_commands": False,
              "scenario": args.scenario, "stage": args.stage, "round": args.round, "seed": args.seed, "fault": args.fault,
              "schedule": asdict(schedule), "initial_snapshot": snapshot.to_dict(),
              "initial_trajectory": initial, "task_boxes": boxes, "payload": payload,
              "initial_cycle_path": str(args.initial_cycle.resolve()), "solves": [], "observations": [], "feedbacks": [], "samples": [],
              "trajectories": [], "worker_events": [], "dynamics_enabled": False, "torque_constraints_enabled": False,
              "runtime_source_sha256": {str(p.relative_to(Path(__file__).parent)): hashlib.sha256(p.read_bytes()).hexdigest()
                                        for p in [Path(__file__), *sorted((Path(__file__).parent / "curobo_core").glob("*.py"))]},
              "initial_cycle_sha256": hashlib.sha256(args.initial_cycle.read_bytes()).hexdigest()}
    process.start(); child.close()
    segment = None
    try:
        if not parent.poll(180):
            raise TimeoutError("worker initialization timed out")
        startup = parent.recv()
        report["startup"] = startup
        if not startup.get("ready"):
            raise RuntimeError(startup.get("error", "worker failed to initialize"))
        started = time.perf_counter()
        segment = RollingSegment(initial, schedule, started,
                                 max_scene_age_s=schedule.period_s + args.compute_bound_s + args.link_bound_s,
                                 max_feedback_age_s=args.max_feedback_age_s)
        report["started_at"] = started
        next_sample = started
        certificate = None
        while True:
            tick = started + segment.sequence * schedule.period_s
            now = time.perf_counter()
            if now - started > args.execution_timeout_s:
                raise TimeoutError("segment execution timeout")
            if now >= segment.active_at + segment.active.duration:
                report["success"] = True
                break
            expected = segment.state(now)
            observation = observe(snapshot, startup["obstacle_center"], args.scenario, segment.sequence,
                                  now, now-started, curve.duration, expected)
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
            prefix = segment.active.window(now-segment.active_at, boundary-segment.active_at)
            forecast_duration = boundary-now if terminal else observation.valid_until-boundary
            reuse = (terminal and certificate is not None and
                     certificate["valid_from"] <= now <= boundary <= certificate["valid_until"] and
                     observation.covered_by(SceneSnapshot.from_dict(certificate["snapshot"]), now, boundary))
            if reuse:
                segment.events.append({"event": "terminal_certificate_reused", "at": now,
                                       "scene_id": observation.identity})
            else:
                parent.send((ticket, observation, prefix, forecast_duration, (args.seed + segment.sequence) % 2**32, fault))
            deadline = min(now + args.compute_bound_s, boundary-args.link_bound_s) if terminal else ticket.compute_deadline
            result = {"success": True, "ready_at": now, "terminal": True} if reuse else None
            arrived, installed = (now if reuse else None), reuse
            while True:
                now = time.perf_counter()
                if not process.is_alive() and not parent.poll():
                    raise RuntimeError("planning worker exited")
                if result is None and parent.poll():
                    message = parent.recv()
                    if "progress" in message:
                        report["worker_events"].append(message["progress"])
                        continue
                    result = message; arrived = time.perf_counter() + args.link_delay_s
                    if fault == "link_delay":
                        arrived += schedule.period_s + args.link_bound_s
                    report["solves"].append(result)
                    if not result.get("success"):
                        raise RuntimeError(result.get("error", "planning failed"))
                    if not terminal and (result.get("sequence") != ticket.sequence or
                                         result.get("observation_id") != observation.identity):
                        raise ValueError("worker returned an obsolete task or scene")
                if result is None and now > deadline:
                    raise TimeoutError("compute deadline miss (supervisor)")
                if result is not None and not installed and now >= arrived:
                    if terminal:
                        if result["ready_at"] > deadline or now > boundary:
                            raise TimeoutError("terminal revalidation missed deadline")
                    else:
                        segment.stage(ticket, result["trajectory"], observation, result["ready_at"], now,
                                      previous_suffix_valid=result["previous_suffix_valid"])
                    installed = True
                if now >= boundary:
                    if not installed:
                        raise TimeoutError("replacement not delivered by splice")
                    if terminal:
                        report["success"] = True
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
        if segment is not None:
            segment.fail(report["error"], time.perf_counter())
        report["safety_action"] = "simulation_aborted; physical stopping NOT validated"
    finally:
        if process.is_alive():
            process.terminate()  # only this run's worker, including obsolete in-flight CUDA work
        process.join(10)
        parent.close()
        report["events"] = segment.events if segment else []
        fields = {
            "compute_wall_s": [x["compute_wall_s"] for x in report["solves"] if "compute_wall_s" in x],
            "available_latency_s": [x["available_latency_s"] for x in report["events"] if "available_latency_s" in x],
            "accepted_latency_s": [x["accepted_latency_s"] for x in report["events"] if "accepted_latency_s" in x],
            "feedback_latency_s": [x["feedback_latency_s"] for x in report["events"] if "feedback_latency_s" in x],
        }
        for result in report["solves"]:
            for name, (start, end) in result.get("spans", {}).items():
                fields.setdefault(f"stage_{name}_s", []).append(end-start)
        report["statistics"] = {key: {"count": len(values), "median": float(np.median(values)),
                                            "p95": float(np.percentile(values, 95)), "max": max(values)}
                                for key, values in fields.items() if values}
        report["trigger_count"] = sum(e["event"] == "requested" for e in report["events"])
        report["splice_count"] = sum(e["event"] == "activated" for e in report["events"])
        report["optimization_success_count"] = sum(x.get("optimization_success", False) for x in report["solves"])
        report["revalidated_fallback_count"] = sum(x.get("strategy") == "revalidated_suffix" for x in report["solves"])
        report["deadline_miss_count"] = sum(x.get("deadline_miss", False) for x in report["solves"])
        if "deadline" in report.get("error", "") and not report["deadline_miss_count"]:
            report["deadline_miss_count"] = 1
        report["cancelled_count"] = sum(e["event"] == "safety_required" and e.get("cancelled_sequence") is not None
                                        for e in report["events"])
        report["censored_compute_count"] = report["trigger_count"] - sum(not x.get("terminal", False) for x in report["solves"])
        report["collision_rejection_count"] = sum("collision" in x.get("previous_suffix_rejection", "")
                                                   for x in report["solves"])
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps({k: report[k] for k in ("success", "trigger_count", "splice_count", "statistics")},
                     ensure_ascii=False), flush=True)
    if not report["success"]:
        raise SystemExit(report["error"])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--initial-cycle", type=Path, required=True)
    parser.add_argument("--round", type=int, default=0, help="zero-based round for a full-sequence input")
    parser.add_argument("--stage", choices=("approach", "transport", "return_home"), default="approach")
    parser.add_argument("--scenario", choices=("clear", "static", "moving", "appear", "depart"), default="clear")
    parser.add_argument("--fault", choices=("none", "compute_delay", "link_delay", "stale_scene",
                                           "feedback_error", "scene_changed", "worker_failure", "solver_failure"),
                        default="none", help="simulation-only failure injection, at first tick")
    parser.add_argument("--compute-bound-s", type=float, required=True)
    parser.add_argument("--link-bound-s", type=float, required=True)
    parser.add_argument("--link-delay-s", type=float, default=0.)
    parser.add_argument("--slices", type=int)
    parser.add_argument("--execution-timeout-s", type=float, default=120.)
    parser.add_argument("--max-feedback-age-s", type=float, default=.1)
    parser.add_argument("--seed", type=int, default=20261003)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if (not math.isfinite(args.execution_timeout_s) or args.execution_timeout_s <= 0 or
            not math.isfinite(args.max_feedback_age_s) or args.max_feedback_age_s <= 0 or
            not 0 <= args.seed < 2**32 or args.round < 0):
        parser.error("positive finite execution/freshness limits and unsigned 32-bit seed required")
    run(args)


if __name__ == "__main__":
    main()
