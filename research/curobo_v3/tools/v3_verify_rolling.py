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


def audit(report, config_path, expect_complete=True, require_obstacle_response=False):
    schedule = report["schedule"]
    t1, t2, t3, n = (schedule[key] for key in ("duration_s", "compute_bound_s", "link_bound_s", "slices"))
    if not np.isfinite([t1, t2, t3]).all() or t2 <= 0 or t3 < 0 or type(n) is not int or not 1 <= n <= t1/(t2+t3):
        raise ValueError("invalid t1/t2/t3/n")
    if report["success"] != expect_complete or report["hardware_commands"]:
        raise ValueError("unexpected outcome or hardware-enabled report")
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
    requests = {e["sequence"]: e for e in report["events"] if e["event"] == "requested"}
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
        if report["completed_at"] < epochs[-1]+curves[-1].duration:
            raise ValueError("completion recorded before reaching the endpoint")
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
    parser.add_argument("--gpu", action="store_true", help="independently recheck installed trajectories in recorded collision envelopes")
    args = parser.parse_args()
    report = json.loads(args.result.read_text())
    summary = audit(report, args.robot_config, args.expect == "complete", args.require_obstacle_response)
    if args.gpu:
        from curobo_core.backend import CuroboBackend
        from curobo_core.rolling_curobo import SegmentReplanner
        from curobo_core.scene import SceneSnapshot, digest
        from curobo_core.rolling import SceneObservation
        adapter = SegmentReplanner(CuroboBackend(PlannerAssets()), report["task_boxes"], payload=report["payload"])
        adapter.validate(report["initial_trajectory"], adapter.scene(SceneSnapshot.from_dict(report["initial_snapshot"])))
        solves = {s["sequence"]: s for s in report["solves"] if "sequence" in s}
        events = [e for e in report["events"] if e["event"] == "activated"]
        observations = {o["identity"]: SceneObservation(
            SceneSnapshot.from_dict(o["snapshot"]), o["observed_at"], o["received_at"], o["valid_until"],
            tuple(o["velocities"]), o["position_error_m"], o["velocity_error_m_s"], o["source"])
            for o in report["observations"]}
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
        summary["gpu_revalidated_trajectories"] = len(events) + 1
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    main()
