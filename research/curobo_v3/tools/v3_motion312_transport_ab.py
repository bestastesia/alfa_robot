#!/usr/bin/env python3
"""MOTION-323 A/B: single-arm transport with the joint RRT baseline vs the Cartesian direct search.

Arms share one planner instance and its warm caches; only the transport mode changes per run:
  joint_rrt          baseline joint-space RRT-Connect (pre-registered arm A)
  cartesian          Cartesian direct search with the pipeline timing (pre-registered arm B)
  cartesian_toppra   Cartesian direct search with the MOTION-324 TOPP-RA timing (reported separately)
Every run plans the complete cycle. The untimed result is checked with the single-arm validator,
and per-run metrics are measured on the executed (timed) transport segment. Test sets, seeds and
the proposed decision rules come from config/motion312_transport_ab.json.
"""
import argparse
import copy
from dataclasses import replace
import json
import math
import subprocess
import time
from pathlib import Path

import numpy as np
import torch
import yourdfpy

from curobo.types import JointState
from curobo_core.contracts import PlannerAssets
from v3_cartesian_transport_planner import CartesianTransportPlanner
from v3_research_assets import ROOT
from v3_single_arm_planner import ActiveRotationCost
from v3_single_arm_regression import validate
from v3_wall_ik_benchmark import ACTIVE_JOINTS

ARMS = {"joint_rrt": ("joint_rrt", "pipeline"), "cartesian": ("cartesian", "pipeline"),
        "cartesian_toppra": ("cartesian", "toppra")}
GOAL_WEIGHTS = np.array([math.radians(15) / .1] + [1.] * 14)
CONFIG = ROOT / "config/motion312_transport_ab.json"


class HarnessPlanner(CartesianTransportPlanner):
    def time_parameterize_result(self, result):
        self.last_untimed = copy.deepcopy(result)
        return super().time_parameterize_result(result)


def task_name(boxes):
    side, box = next(iter(boxes.items()))
    return f"{side}{box:02d}"


def gpu_state():
    try:
        fields = subprocess.check_output(["nvidia-smi", "--query-gpu=clocks.sm,temperature.gpu,utilization.gpu",
                                          "--format=csv,noheader,nounits"], text=True).split(",")
        return dict(sm_clock_mhz=int(fields[0]), temperature_c=int(fields[1]), utilization_percent=int(fields[2]))
    except (OSError, subprocess.SubprocessError, ValueError, IndexError):
        return None


def executed_metrics(planner, boxes, result):
    names = result["joint_names"]
    index = [names.index(name) for name in ACTIVE_JOINTS]
    frames = np.asarray(result["frames"], dtype=np.float64)
    transport = np.ascontiguousarray(frames[[i for i, phase in enumerate(result["phases"]) if phase == "transport"]][:, index])
    planner.configure_task(boxes)
    checker = planner.checker(boxes, payload=True)
    side = planner.active_side
    values = torch.tensor(transport, device="cuda", dtype=torch.float32)
    rotation = ActiveRotationCost(checker, values[0], values[-1], planner.top_grasp, side).metrics(values)[side]
    state = checker.kinematics.compute_kinematics(JointState.from_position(values[None], joint_names=ACTIVE_JOINTS))
    positions = state.tool_poses.to_dict()[f"{side}_tool0"].position.reshape(-1, 3).cpu().numpy().astype(np.float64)
    tool_path = float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())
    chord = float(np.linalg.norm(positions[-1] - positions[0]))
    arm = transport[:, 1:8] if side == "left" else transport[:, 8:15]
    signs = np.sign(arm[:, [1, 3, 5]])
    segment = next(item for item in result["timed_segments"] if item["name"] == "transport")
    return dict(
        top_grasp=bool(planner.top_grasp), transport_duration_s=segment["duration_s"],
        transport_parameterizer=segment["parameterizer"], round_motion_s=result["motion_duration_s"],
        required_rotation_deg=rotation["required_rotation_deg"], total_rotation_deg=rotation["total_rotation_deg"],
        extra_rotation_deg=rotation["total_rotation_deg"] - rotation["required_rotation_deg"],
        backtracking_deg=rotation["backtracking_deg"], max_rotation_deviation_deg=rotation["max_deviation_deg"],
        tool_path_m=tool_path, chord_m=chord, path_to_chord=tool_path / max(chord, 1e-9),
        weighted_joint_arc=float(np.linalg.norm(np.diff(transport, axis=0) * GOAL_WEIGHTS, axis=1).sum()),
        branch_flips=int(np.count_nonzero(np.any(np.diff(signs, axis=0) != 0, axis=1))),
        min_singularity_margin_deg=float(np.degrees(np.abs(arm[:, [1, 3, 5]]).min())))


def run_once(planner, model, boxes, arm, seed):
    mode, timing = ARMS[arm]
    planner.transport_search_mode, planner.transport_timing, planner.transport_seed = mode, timing, seed
    planner.last_transport_stats = planner.last_untimed = planner.last_toppra_error = None
    record = dict(task=task_name(boxes), arm=arm, seed=seed, gpu_before=gpu_state())
    began = time.perf_counter()
    try:
        planner.configure_task(boxes)
        result = planner.full_cycle(boxes)
    except RuntimeError as error:
        record.update(success=False, failure_class="exception", error=str(error)[:300],
                      wall_ms=(time.perf_counter() - began) * 1000)
        return record, None
    stats = planner.last_transport_stats or {}
    record.update(success=bool(result.get("success")), wall_ms=(time.perf_counter() - began) * 1000,
                  planning=dict(round_total_ms=result.get("total_ms"), transport_stage_ms=result.get("timing_ms", {}).get("transport_ms"),
                                **{key: stats.get(key) for key in ("search_ms", "first_solution_ms", "first_verified_solution_ms",
                                                                   "iterations", "tree_nodes", "tier", "ik_calls", "ik_ms",
                                                                   "selected_goal", "raw_waypoints", "failure")}),
                  toppra_error=planner.last_toppra_error)
    if not result.get("success"):
        stage = result.get("blocked_stage")
        record.update(blocked_stage=stage, blocked_details=str(result.get("blocked_details"))[:300],
                      failure_class="transport_search" if stage == "运输RRT" else f"other_stage:{stage}")
        return record, result
    validation = result["timed_validation"]
    record["timing"] = dict(time_parameterization_ms=result.get("trajectory_timing_ms"), timed_ok=validation["success"],
                            ratios=[validation[key] for key in ("velocity_limit_ratio", "acceleration_limit_ratio", "jerk_limit_ratio")])
    record["executed"] = executed_metrics(planner, boxes, result)
    untimed = planner.last_untimed
    names = untimed["joint_names"]
    rows = np.asarray(untimed["frames"])[[i for i, phase in enumerate(untimed["phases"]) if phase == "transport"]]
    record["executed"]["untimed_max_adjacent_step_deg"] = float(np.degrees(np.abs(np.diff(
        rows[:, [names.index(name) for name in ACTIVE_JOINTS[1:]]], axis=0)).max()))
    try:
        record["safety"] = dict(validated=True, **validate(untimed, planner, model))
    except Exception as error:  # a failed or crashing check is evidence, not a reason to stop the batch
        record["safety"] = dict(validated=False, error=f"{type(error).__name__}: {str(error)[:300]}")
    return record, result


def run(args):
    config = json.loads(CONFIG.read_text())
    tasks = config["test_sets"][{"dev": "dev_single", "holdout": "holdout_single"}[args.tasks]]
    seeds = [int(seed) for seed in args.seeds.split(",")] if args.seeds else config["seeds"]["per_method"]
    arms = args.arms.split(",")
    runs, full = args.output / "runs", args.output / "full"
    runs.mkdir(parents=True, exist_ok=True)
    full.mkdir(exist_ok=True)
    assets = replace(PlannerAssets(), **{key: getattr(args, key) for key in ("robot_config", "mobile_robot_config")
                                         if getattr(args, key)})
    if assets != PlannerAssets():
        (args.output / "assets.json").write_text(json.dumps({key: str(getattr(assets, key)) for key in
                                                          ("robot_config", "mobile_robot_config")}, indent=1) + "\n")
    options = json.loads(args.cartesian_options) if args.cartesian_options else config.get("cartesian_options_frozen", {})
    planner = HarnessPlanner(assets, cartesian_options=options)
    planner.warmup_resources()
    model = yourdfpy.URDF.load(assets.mobile_robot_config.parent / "model/robot_meter_meshes_mobile18.urdf", load_meshes=False)
    for task_index, boxes in enumerate(tasks):
        for seed_index, seed in enumerate(seeds):
            order = arms if (task_index + seed_index) % 2 == 0 else arms[::-1]
            for arm in order:
                path = runs / f"{task_name(boxes)}-s{seed}-{arm}.json"
                if path.exists():
                    continue
                record, result = run_once(planner, model, boxes, arm, seed)
                record.update(order=order, cartesian_options=planner.cartesian_options)
                path.write_text(json.dumps(record, ensure_ascii=False, default=float) + "\n")
                if result is not None and (args.keep_full == "all" or (args.keep_full == "first" and seed_index == 0)):
                    (full / path.name).write_text(json.dumps(result, ensure_ascii=False) + "\n")
                print(json.dumps({key: record.get(key) for key in ("task", "arm", "seed", "success", "failure_class")}
                                 | {"transport_ms": (record.get("planning") or {}).get("transport_stage_ms"),
                                    "extra_rot": (record.get("executed") or {}).get("extra_rotation_deg"),
                                    "duration": (record.get("executed") or {}).get("transport_duration_s"),
                                    "valid": (record.get("safety") or {}).get("validated")}, ensure_ascii=False, default=float),
                      flush=True)


def percentile(values, fraction):
    values = sorted(values)
    return values[max(0, math.ceil(fraction * len(values)) - 1)] if values else None


def summarize(args):
    records = [json.loads(path.read_text()) for path in sorted((args.input / "runs").glob("*.json"))]
    arms = sorted({record["arm"] for record in records})
    by_key = {(record["task"], record["seed"], record["arm"]): record for record in records}
    summary = dict(runs=len(records), arms={})
    for arm in arms:
        rows = [record for record in records if record["arm"] == arm]
        ok = [record for record in rows if record.get("success")]
        metric = lambda path: [value for value in (_get(record, path) for record in ok) if value is not None]
        summary["arms"][arm] = dict(
            runs=len(rows), success=len(ok), validated=sum(bool((record.get("safety") or {}).get("validated")) for record in ok),
            failures=sorted({record.get("failure_class") for record in rows if not record.get("success")} - {None}),
            **{f"{name}_{stat}": function(metric(path)) for name, path in (
                ("transport_stage_ms", "planning.transport_stage_ms"), ("search_ms", "planning.search_ms"),
                ("first_verified_ms", "planning.first_verified_solution_ms"),
                ("round_total_ms", "planning.round_total_ms"), ("transport_duration_s", "executed.transport_duration_s"),
                ("round_motion_s", "executed.round_motion_s"), ("extra_rotation_deg", "executed.extra_rotation_deg"),
                ("path_to_chord", "executed.path_to_chord"))
               for stat, function in (("median", lambda values: float(np.median(values)) if values else None),
                                      ("p95", lambda values: percentile(values, .95)))})
    paired = {}
    for arm in arms:
        if arm == "joint_rrt":
            continue
        pairs = [(by_key[(task, seed, "joint_rrt")], by_key[(task, seed, arm)]) for task, seed, candidate in by_key
                 if candidate == arm and (task, seed, "joint_rrt") in by_key]
        both = [(a, b) for a, b in pairs if a.get("success") and b.get("success")]
        ratio = lambda path: [(_get(b, path) / _get(a, path) - 1) * 100 for a, b in both if _get(a, path)]
        side = [(a, b) for a, b in both if not a["executed"]["top_grasp"]]
        reduction = [(1 - b["executed"]["extra_rotation_deg"] / a["executed"]["extra_rotation_deg"]) * 100
                     for a, b in side if a["executed"]["extra_rotation_deg"] > 1e-6]
        paired[arm] = dict(
            pairs=len(pairs), both_success=len(both),
            transport_stage_ms_change_percent_median=_median(ratio("planning.transport_stage_ms")),
            transport_duration_change_percent_median=_median(ratio("executed.transport_duration_s")),
            round_motion_change_percent_median=_median(ratio("executed.round_motion_s")),
            side_extra_rotation_reduction_percent_median=_median(reduction),
            path_to_chord_change_percent_median=_median(ratio("executed.path_to_chord")))
    summary["paired_vs_joint_rrt"] = paired
    summary["decisions"] = decisions(summary, records)
    output = args.output or args.input / "summary.json"
    output.write_text(json.dumps(summary, ensure_ascii=False, indent=1, default=float) + "\n")
    print(json.dumps(summary, ensure_ascii=False, indent=1, default=float))


def decisions(summary, records, budget_ms=2000.):
    """Pre-registered rules per arm against joint_rrt; planning time: paired median primary, per-arm secondary."""
    base = summary["arms"].get("joint_rrt")
    output = {}
    for arm, paired in summary["paired_vs_joint_rrt"].items():
        stats = summary["arms"][arm]
        change = lambda key: (stats[key] / base[key] - 1) * 100 if base and base.get(key) else None
        searches = [_get(record, "planning.search_ms") for record in records if record["arm"] == arm and record.get("success")]
        output[arm] = dict(
            planning_time=dict(
                paired_stage_change_percent=paired["transport_stage_ms_change_percent_median"],
                per_arm_stage_median_change_percent=change("transport_stage_ms_median"),
                per_arm_stage_p95_change_percent=change("transport_stage_ms_p95"),
                max_search_ms=max((value for value in searches if value is not None), default=None),
                passed=bool(paired["transport_stage_ms_change_percent_median"] is not None
                            and paired["transport_stage_ms_change_percent_median"] <= 10
                            and max((value for value in searches if value is not None), default=0) <= budget_ms)),
            success=dict(arm=stats["success"], baseline=base["success"] if base else None,
                         passed=bool(base and stats["success"] / stats["runs"] >= base["success"] / base["runs"])),
            quality=dict(side_extra_rotation_reduction_percent=paired["side_extra_rotation_reduction_percent_median"],
                         path_to_chord_change_percent=paired["path_to_chord_change_percent_median"],
                         passed=bool((paired["side_extra_rotation_reduction_percent_median"] or 0) >= 30
                                     and (paired["path_to_chord_change_percent_median"] or 0) <= 0)),
            execution_duration=dict(paired_change_percent=paired["transport_duration_change_percent_median"],
                                    passed=bool(paired["transport_duration_change_percent_median"] is not None
                                                and paired["transport_duration_change_percent_median"] <= 20)))
    return output


def audit(args):
    """CPU FCL mesh audit of the executed transport frames of every kept full result."""
    import yaml
    from curobo_core.mesh_validation import MeshStateChecker
    from curobo_core.scene import SceneSnapshot

    config = yaml.safe_load((ROOT / "generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml").read_text())["kinematics"]
    report = []
    for path in sorted((args.input / "full").glob("*.json")):
        result = json.loads(path.read_text())
        if not result.get("success"):
            continue
        event = next(item for item in result["predicted_scene_events"] if item["phase"] == "attach")
        snapshot = SceneSnapshot.from_dict(event["snapshot"])
        checker = MeshStateChecker(config, snapshot, dict(result["task"]), snapshot.attachments)
        names, advance = result["joint_names"], float(result.get("base_advance_m", 0.))
        frames = [frame for frame, phase in zip(result["frames"], result["phases"]) if phase == "transport"]
        began, contacts = time.perf_counter(), {}
        for index, frame in enumerate(frames):
            positions = dict(zip(names, frame))
            positions["base_x"] -= advance  # results add the top-suction base advance; the snapshot is in the planner frame
            found = checker.check(positions, attached=True, payload_enabled=True)
            if found:
                pair = " | ".join(found.get("pair", ["unknown"]))
                depth = float(found.get("contact_depth_m") or 0.)
                entry = contacts.setdefault(pair, dict(frames=0, max_depth_mm=0., first_frame=index, last_frame=index))
                entry.update(frames=entry["frames"] + 1, max_depth_mm=max(entry["max_depth_mm"], depth * 1000), last_frame=index)
        report.append(dict(run=path.stem, passed=not contacts, transport_frames=len(frames),
                           contact_frames=sum(item["frames"] for item in contacts.values()), contacts=contacts,
                           ms=(time.perf_counter() - began) * 1000))
        print(json.dumps({key: report[-1][key] for key in ("run", "passed", "contact_frames", "transport_frames", "contacts")},
                         ensure_ascii=False), flush=True)
    (args.input / "fcl-audit.json").write_text(json.dumps(dict(
        scope="all executed transport frames; first reported pair per frame; FCL meshes and box primitives; "
              "task/payload exemptions as in MeshStateChecker",
        runs=report), ensure_ascii=False, indent=1, default=str) + "\n")


def _get(record, path):
    value = record
    for key in path.split("."):
        value = (value or {}).get(key)
    return value


def _median(values):
    return float(np.median(values)) if values else None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    runner = commands.add_parser("run")
    runner.add_argument("--tasks", choices=("dev", "holdout"), required=True)
    runner.add_argument("--arms", default=",".join(ARMS))
    runner.add_argument("--seeds", help="comma separated; default: pre-registered seeds")
    runner.add_argument("--keep-full", choices=("none", "first", "all"), default="first")
    runner.add_argument("--cartesian-options", help="JSON overrides of v3_cartesian_transport.DEFAULTS; "
                                                     "default: cartesian_options_frozen from the config")
    runner.add_argument("--robot-config", type=Path, help="research copy of the fixed-base robot config (default: frozen)")
    runner.add_argument("--mobile-robot-config", type=Path, help="research copy of the mobile robot config; its "
                        "directory must contain model/ like the frozen one (default: frozen)")
    runner.add_argument("--output", type=Path, required=True)
    summary = commands.add_parser("summarize")
    summary.add_argument("--input", type=Path, required=True)
    summary.add_argument("--output", type=Path)
    auditor = commands.add_parser("audit")
    auditor.add_argument("--input", type=Path, required=True)
    args = parser.parse_args()
    {"run": run, "summarize": summarize, "audit": audit}[args.command](args)


if __name__ == "__main__":
    main()
