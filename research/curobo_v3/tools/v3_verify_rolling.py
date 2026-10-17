#!/usr/bin/env python3
"""Audit saved rolling execution evidence, not just the producer's success flag."""
import argparse
import bisect
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import yaml

from curobo_core.contracts import PlannerAssets
from curobo_core.trajectory import TimedTrajectory
from curobo_core.rolling import SceneObservation
from curobo_core.scene import SceneSnapshot, digest



def terminal_evidence(report):
    """Prove the final committed interval was checked against its fresh observation."""
    entries = report["trajectories"]
    entry = entries[-1] if entries else dict(active_at=report["started_at"], trajectory=report["initial_trajectory"])
    curve = TimedTrajectory(entry["trajectory"])
    end = entry["active_at"] + curve.duration
    period = report["schedule"]["duration_s"] / report["schedule"]["slices"]
    if curve.duration > period + 1e-8:
        raise ValueError("terminal interval skipped required periodic replanning")
    events = [e for e in report["events"] if e["event"] in ("terminal_validated", "terminal_certificate_reused")]
    if len(events) != 1:
        raise ValueError("missing or duplicate terminal safety evidence")
    event = events[0]
    source = next((o for o in report["observations"] if o["identity"] == event["scene_id"]), None)
    if source is None:
        raise ValueError("terminal observation is missing")
    observation = SceneObservation.from_dict(source)
    observation.require_fresh(event["at"], period + report["schedule"]["compute_bound_s"] + report["schedule"]["link_bound_s"])
    if not entry["active_at"] <= observation.observed_at <= event["at"] <= end:
        raise ValueError("terminal observation is outside the active interval")
    document = curve.window(observation.observed_at-entry["active_at"], curve.duration)
    world = observation.swept_snapshot(observation.observed_at, end)
    if event["event"] == "terminal_certificate_reused":
        activated = [e for e in report["events"] if e["event"] == "activated"]
        if not activated:
            raise ValueError("no installed trajectory certificate to reuse")
        result = next((s for s in report["solves"] if s.get("sequence") == activated[-1]["sequence"]), None)
        if not result or not result.get("success") or "certificate" not in result:
            raise ValueError("missing successful solve behind reused terminal certificate")
        certificate = result["certificate"]
        if digest(entry["trajectory"]) != digest(result["trajectory"]):
            raise ValueError("reused certificate refers to a different trajectory")
        if (not certificate["valid_from"] <= observation.observed_at <= end <= certificate["valid_until"] or
                not observation.covered_by(SceneSnapshot.from_dict(certificate["snapshot"]), observation.observed_at, end)):
            raise ValueError("new terminal scene is not covered by the old certificate")
    else:
        result = next((s for s in report["solves"] if s.get("terminal") and
                       s.get("observation_id") == observation.identity), None)
        if not result or not result.get("success") or "certificate" not in result:
            raise ValueError("missing terminal verification result")
        t2, t3 = report["schedule"]["compute_bound_s"], report["schedule"]["link_bound_s"]
        if (result["ready_at"] != event["safe_ready_at"] or
                not observation.observed_at <= result["ready_at"] <= min(observation.observed_at+t2, end-t3) or
                not result["ready_at"] <= event["at"] <= result["ready_at"]+t3):
            raise ValueError("terminal computation/communication deadline violation")
        certificate = result["certificate"]
        if (digest(result["trajectory"]) != digest(document) or
                certificate["valid_from"] != observation.observed_at or abs(certificate["valid_until"]-end) > 1e-8 or
                SceneSnapshot.from_dict(certificate["snapshot"]).geometry_key != world.geometry_key):
            raise ValueError("terminal verification does not describe the executed interval")
    return document, world


def audit(report, config_path, expect_complete=True, require_obstacle_response=False):
    schedule = report["schedule"]
    t1, t2, t3, n = (schedule[key] for key in ("duration_s", "compute_bound_s", "link_bound_s", "slices"))
    if not np.isfinite([t1, t2, t3]).all() or t2 <= 0 or t3 < 0 or type(n) is not int or not 1 <= n <= t1/(t2+t3):
        raise ValueError("invalid t1/t2/t3/n")
    if report["hardware_commands"]:
        raise ValueError("hardware-enabled report is not a simulation acceptance result")
    if report["success"] != expect_complete:
        raise ValueError(f"unexpected outcome: {report.get('error', 'run succeeded but rejection was expected')}")
    if not report.get("runtime_source_sha256") or not report.get("initial_cycle_sha256"):
        raise ValueError("missing reproducibility hashes")
    initial = TimedTrajectory(report["initial_trajectory"])
    if abs(initial.duration-t1) > 1e-8:
        raise ValueError("t1 is not the initial trajectory duration")
    cfg = yaml.safe_load(config_path.read_text())
    kin = cfg.get("robot_cfg", cfg)["kinematics"]
    cspace = kin["cspace"]
    root = ET.parse(kin["urdf_path"]).getroot()
    limits_by_name = {j.attrib["name"]: j.find("limit").attrib for j in root.findall("joint")
                      if j.find("limit") is not None}
    names = initial.joint_names
    indices = [cspace["joint_names"].index(name) for name in names]
    lower, upper, velocity = (np.array([float(limits_by_name[n][k]) for n in names]) for k in ("lower", "upper", "velocity"))
    def configured(name, default=None):
        values = np.asarray(cspace.get(name, default), dtype=float)
        return np.full(len(names), float(values)) if values.ndim == 0 else values[indices]
    velocity *= configured("velocity_scale", 1.)
    acceleration = configured("max_acceleration") * configured("acceleration_scale", 1.)
    jerk = configured("max_jerk") * configured("jerk_scale", 1.)
    entries = [{"active_at": report["started_at"], "trajectory": report["initial_trajectory"]}, *report["trajectories"]]
    curves = [TimedTrajectory(x["trajectory"]) for x in entries]
    epochs = [x["active_at"] for x in entries]
    if np.any(np.diff(epochs) <= 0):
        raise ValueError("non-monotonic execution epochs")
    goal = initial.sample(initial.duration)
    maximum_ratios = dict(velocity=0., acceleration=0., jerk=0.)
    for i, curve in enumerate(curves):
        ratios = curve.validate_limits(lower, upper, velocity, acceleration, jerk)
        maximum_ratios = {k: max(maximum_ratios[k], ratios[k]) for k in ratios}
        if curve.joint_names != names:
            raise ValueError("joint order changed")
        for key, tolerance in (("positions", 1e-4), ("velocities", 1e-3), ("accelerations", 1e-3)):
            if not np.allclose(curve.sample(curve.duration)[key], goal[key], atol=tolerance, rtol=0):
                raise ValueError(f"original segment endpoint {key} changed")
            if i and not np.allclose(curve.sample(0)[key], curves[i-1].sample(epochs[i]-epochs[i-1])[key],
                                     atol=tolerance, rtol=0):
                raise ValueError(f"splice {i} discontinuous {key}")
    request_events = [e for e in report["events"] if e["event"] == "requested"]
    sequences = [e["sequence"] for e in request_events]
    if sequences != list(range(len(sequences))) or any(type(sequence) is not int for sequence in sequences):
        raise ValueError("duplicate, missing or out-of-order replanning request")
    for request in request_events:
        tick = report["started_at"] + request["sequence"] * (t1/n)
        if not tick <= request["at"] < tick+t1/n or abs(request["splice_at"]-(tick+t1/n)) > 1e-8:
            raise ValueError("replanning request does not follow the declared time-slice schedule")
    requests = {e["sequence"]: e for e in request_events}
    staged = {e["sequence"]: e for e in report["events"] if e["event"] == "staged"}
    activated = [e for e in report["events"] if e["event"] == "activated"]
    observations = {o["identity"]: o for o in report["observations"]}
    if len(activated) != len(entries)-1 or len(activated) != report["splice_count"]:
        raise ValueError("execution trace and trajectory count disagree")
    for i, event in enumerate(activated):
        request, stage = requests[event["sequence"]], staged[event["sequence"]]
        observation = observations[request["scene_id"]]
        deadline = min(request["at"]+t2, request["splice_at"]-t3)
        if (abs(event["at"] - request["splice_at"]) > 1e-8 or
                abs(event["at"] - epochs[i+1]) > 1e-8 or
                not request["at"] <= stage["safe_ready_at"] <= deadline or
                not stage["safe_ready_at"] <= stage["at"] <= event["at"] or
                stage["at"]-stage["safe_ready_at"] > t3 + 1e-8):
            raise ValueError("deadline/communication/boundary violation")
        if abs((stage["safe_ready_at"]-observation["observed_at"])-stage["available_latency_s"]) > 1e-8:
            raise ValueError("incorrect observation-to-safe-trajectory latency")
    sample_times = [s["at"] for s in report["samples"]]
    if (expect_complete and not sample_times) or np.any(np.diff(sample_times) <= 0):
        raise ValueError("missing or non-monotonic execution samples")
    for sample in report["samples"]:
        index = bisect.bisect_right(epochs, sample["at"])-1
        expected = curves[index].sample(sample["at"]-epochs[index])
        for key in ("positions", "velocities", "accelerations"):
            if not np.allclose(sample[key], expected[key], atol=1e-7, rtol=0):
                raise ValueError(f"sample does not follow the installed trajectory: {key}")
    if expect_complete:
        if report.get("schema_version", 1) >= 3:
            terminal_evidence(report)
        endpoint_at = epochs[-1] + curves[-1].duration
        if report["completed_at"] < endpoint_at:
            raise ValueError("completion recorded before reaching the endpoint")
        if report.get("schema_version", 1) >= 2:
            final = report["final_feedback"]
            expected_id = activated[-1]["trajectory_id"] if activated else report["samples"][0]["trajectory_id"]
            if (final["trajectory_id"] != expected_id or tuple(final["joint_names"]) != names or
                    abs(final["sampled_at"] - endpoint_at) > 1e-8 or
                    not endpoint_at <= final["received_at"] <= report["completed_at"]):
                raise ValueError("completion feedback refers to the wrong trajectory/time")
            for key, tolerance in (("positions", 1e-4), ("velocities", 1e-3), ("accelerations", 1e-3)):
                if not np.allclose(final["state"][key], goal[key], atol=tolerance, rtol=0):
                    raise ValueError(f"completion feedback did not reach the segment goal: {key}")
    elif not any(e["event"] == "safety_required" for e in report["events"]):
        raise ValueError("failure without an explicit safety-required event")
    obstacle_responses = [s for s in report["solves"] if s.get("previous_suffix_valid") is False
                          and s.get("optimization_success") and s.get("success")]
    if require_obstacle_response and not obstacle_responses:
        raise ValueError("no evidence of replacing a newly blocked suffix with an optimized valid one")
    nonzero_splices = sum(bool(np.max(np.abs(c.sample(0)["velocities"])) > 1e-4) for c in curves[1:])
    if expect_complete and n > 1 and not nonzero_splices:
        raise ValueError("no demonstrated moving-boundary splice")
    return {"verified": True, "complete": report["success"], "splices": len(activated),
            "moving_splices": nonzero_splices, "obstacle_responses": len(obstacle_responses),
            "maximum_limit_ratios": maximum_ratios}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--robot-config", type=Path, default=PlannerAssets().robot_config)
    parser.add_argument("--expect", choices=("complete", "rejected"), default="complete")
    parser.add_argument("--require-obstacle-response", action="store_true")
    parser.add_argument("--require-rejection-code", choices=("GOAL_ENDPOINT_COLLISION", "SPLICE_ENDPOINT_COLLISION"),
                        help="verify a specific fail-closed condition, separately from successful avoidance")
    parser.add_argument("--gpu", action="store_true", help="independently recheck installed trajectories in recorded collision envelopes")
    args = parser.parse_args()
    report = json.loads(args.result.read_text())
    summary = audit(report, args.robot_config, args.expect == "complete", args.require_obstacle_response)
    if args.require_rejection_code:
        if args.expect != "rejected" or report.get("failure_code") != args.require_rejection_code:
            raise ValueError("report did not reject for the requested collision reason")
    if args.gpu:
        from curobo_core.rolling_backend import RollingBackend
        from curobo_core.rolling_curobo import SegmentReplanner
        from curobo_core.scene import SceneSnapshot, digest
        from curobo_core.rolling import SceneObservation
        backend = RollingBackend(PlannerAssets())
        backend.configure_task(report["task_boxes"])
        adapter = SegmentReplanner(backend, report["task_boxes"], payload=report["payload"], phase=report["stage"])
        adapter.validate(report["initial_trajectory"], adapter.scene(SceneSnapshot.from_dict(report["initial_snapshot"])))
        solves = {s["sequence"]: s for s in report["solves"] if "sequence" in s}
        events = [e for e in report["events"] if e["event"] == "activated"]
        observations = {o["identity"]: SceneObservation.from_dict(o) for o in report["observations"]}
        previous = TimedTrajectory(report["initial_trajectory"])
        previous_at = report["started_at"]
        for event, entry in zip(events, report["trajectories"]):
            solve = solves[event["sequence"]]
            if digest(entry["trajectory"]) != digest(solve["trajectory"]):
                raise ValueError("executed trajectory differs from the verified solve result")
            observation = observations[solve["observation_id"]]
            if observation.identity != solve["observation_id"]:
                raise ValueError("observation contents disagree with its recorded identity")
            prefix = solve["prefix_certificate"]
            prefix_scene = observation.swept_snapshot(observation.observed_at, event["at"])
            if prefix_scene.geometry_key != SceneSnapshot.from_dict(prefix["snapshot"]).geometry_key:
                raise ValueError("prefix certificate does not cover the observed obstacle motion")
            expected_prefix = previous.window(observation.observed_at-previous_at, event["at"]-previous_at)
            if digest(expected_prefix) != digest(prefix["trajectory"]):
                raise ValueError("prefix certificate is not for the actually executed prefix")
            adapter.validate(expected_prefix, adapter.scene(prefix_scene))
            certificate = solve["certificate"]
            snapshot = observation.swept_snapshot(event["at"], certificate["valid_until"])
            if snapshot.geometry_key != SceneSnapshot.from_dict(certificate["snapshot"]).geometry_key:
                raise ValueError("suffix certificate does not cover the observed obstacle motion")
            if event["at"] + TimedTrajectory(entry["trajectory"]).duration > certificate["valid_until"]:
                raise ValueError("suffix exceeds collision certificate time horizon")
            adapter.validate(entry["trajectory"], adapter.scene(snapshot))
            previous, previous_at = TimedTrajectory(entry["trajectory"]), event["at"]
        if args.require_rejection_code:
            rejected = next((s for s in reversed(report["solves"])
                             if s.get("failure_code") == args.require_rejection_code), None)
            if not rejected or rejected.get("success") or "trajectory" in rejected:
                raise ValueError("missing fail-closed endpoint-check result")
            request = next(e for e in report["events"] if e["event"] == "requested"
                           and e["sequence"] == rejected["sequence"])
            interval = [request["splice_at"], previous_at + previous.duration]
            if not np.allclose(rejected["endpoint_check_interval"], interval, atol=1e-8, rtol=0):
                raise ValueError("endpoint rejection used the wrong execution interval")
            observation = observations[rejected["observation_id"]]
            world = observation.swept_snapshot(*interval)
            q = np.stack([previous.sample(interval[0]-previous_at)["positions"],
                          previous.sample(previous.duration)["positions"]])
            measured = adapter.check_endpoints(q, adapter.scene(world))
            expected = rejected["endpoint_validity"]
            index = 0 if args.require_rejection_code == "SPLICE_ENDPOINT_COLLISION" else 1
            if measured != expected or measured[index]:
                raise ValueError("independent GPU check contradicts endpoint rejection")
            summary["gpu_endpoint_rejection_confirmed"] = measured
        if report["success"] and report.get("schema_version", 1) >= 3:
            document, world = terminal_evidence(report)
            adapter.validate(document, adapter.scene(world))
            summary["gpu_terminal_revalidated"] = True
        summary["gpu_revalidated_trajectories"] = len(events) + 1
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
