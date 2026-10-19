#!/usr/bin/env python3
"""MOTION-323 L1 spikes for a Cartesian direct search on the single-arm transport stage.

The unchanged baseline planner runs every task. After its transport search, the same inputs
(start, goal pool and loaded checker) are analysed in-process:
S1 branch/redundancy statistics, S2 direct Cartesian line feasibility, S3 analytic IK
round trip and latency, S5 execution timing with and without the joint shortcut.
"""
import argparse
import collections
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import torch
from scipy.spatial.transform import Rotation, Slerp

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from curobo.types import JointState
from curobo_core.contracts import PlannerAssets
from curobo_core.trajectory import time_parameterize_path
from v3_batched_loaded_search import GpuValidity, densify
from v3_research_assets import ROOT, atlas_path
from v3_single_arm_planner import ActiveRotationCost
from v3_timed_single_arm_planner import TimedSingleArmPlanner
from v3_timed_wall_sequence import WALL_SEQUENCE
from v3_wall_ik_benchmark import ACTIVE_JOINTS

DEV_SINGLE = [task for task in WALL_SEQUENCE if len(task) == 1]
GOAL_WEIGHTS = np.array([math.radians(15) / .1] + [1.] * 14)


def branch(arm):
    return tuple(int(value) for value in np.sign(np.asarray(arm)[[1, 3, 5]]))


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def layout(planner):
    side = planner.active_side
    index = 0 if side == "left" else 1
    return side, index, 1 + 7 * index


def tool_and_carriage(planner, row, side):
    planner.fk(row)
    base = planner.fk_robot.base_link
    return (planner.fk_robot.get_transform(f"{side}_tool0", base).copy(),
            np.linalg.inv(planner.fk_robot.get_transform("arm_carriage", base)))


def describe(planner, row, arm_layout):
    side, index, offset = arm_layout
    tool, _ = tool_and_carriage(planner, row, side)
    arm = row[offset:offset + 7]
    return dict(tool=tool, psi=planner.analytic.swivel(index, arm), updown=float(row[0]), branch=branch(arm))


def goal_pool(planner, start, checker):
    # Same pool and filter as SingleArmPlanner._full_cycle: atlas x idle-arm variants, validity, envelope.
    atlas = np.load(atlas_path(planner))["joints"]
    pool = [atlas.copy()]
    for idle in planner.free_arm_variants(start):
        values = atlas.copy()
        values[:, planner.idle_indices] = idle
        pool.append(values)
    pool = np.unique(np.round(np.concatenate(pool), 6), axis=0).astype(np.float32)
    limit = json.loads((ROOT / "config/placement_task_policy.json").read_text())["side"]["placement_radius_m"]
    validity = GpuValidity(checker, check_ground=True)
    keep = []
    for offset in range(0, len(pool), 512):
        batch = torch.tensor(pool[offset:offset + 512], device="cuda")
        keep.extend((validity.mask(batch) & (planner.full_envelope_radius(checker, batch) <= limit)).cpu().tolist())
    return pool[np.asarray(keep, dtype=bool)].astype(np.float64)


def path_metrics(planner, checker, values, side):
    values = np.ascontiguousarray(values, dtype=np.float64)
    tensor = torch.tensor(values, device="cuda", dtype=torch.float32)
    state = checker.kinematics.compute_kinematics(JointState.from_position(tensor[None], joint_names=ACTIVE_JOINTS))
    positions = state.tool_poses.to_dict()[f"{side}_tool0"].position.reshape(-1, 3).cpu().numpy().astype(np.float64)
    tool_path = float(np.linalg.norm(np.diff(positions, axis=0), axis=1).sum())
    chord = float(np.linalg.norm(positions[-1] - positions[0]))
    rotation = ActiveRotationCost(checker, tensor[0], tensor[-1], planner.top_grasp, side).metrics(tensor)[side]
    return dict(tool_path_m=tool_path, chord_m=chord, path_to_chord=tool_path / max(chord, 1e-9),
                weighted_joint_arc=float(np.linalg.norm(np.diff(values, axis=0) * GOAL_WEIGHTS, axis=1).sum()),
                **rotation)


def direct_line(planner, start, goal, arm_layout, validity, max_step_deg):
    side, index, offset = arm_layout
    began = time.perf_counter()
    first, _ = tool_and_carriage(planner, start, side)
    last, _ = tool_and_carriage(planner, goal, side)
    psi = planner.analytic.swivel(index, start[offset:offset + 7])
    turn = wrap(planner.analytic.swivel(index, goal[offset:offset + 7]) - psi)
    rotations = Rotation.from_matrix(np.stack([first[:3, :3], last[:3, :3]]))
    angle = (rotations[0].inv() * rotations[1]).magnitude()
    shift = last[:3, 3] - first[:3, 3]
    lift = goal[0] - start[0]
    count = max(1, math.ceil(np.linalg.norm(shift) / .01), math.ceil(angle / math.radians(1)),
                math.ceil(abs(turn) / math.radians(1)), math.ceil(abs(lift) / .01))
    slerp = Slerp([0., 1.], rotations)
    wanted = branch(start[offset:offset + 7])
    rows, largest = [start.copy()], 0.
    outcome = dict(steps=count, psi_turn_deg=math.degrees(turn), lift_m=float(lift),
                   tool_shift_m=float(np.linalg.norm(shift)), tool_rotation_deg=math.degrees(angle))
    for step in range(1, count + 1):
        fraction = step / count
        row = rows[-1].copy()
        row[0] = start[0] + fraction * lift
        _, carriage = tool_and_carriage(planner, row, side)
        target = np.eye(4)
        target[:3, :3] = slerp([fraction]).as_matrix()[0]
        target[:3, 3] = first[:3, 3] + fraction * shift
        previous = rows[-1][offset:offset + 7]
        solutions = [arm for arm in planner.analytic.solve(index, carriage @ target, psi + fraction * turn, previous)
                     if branch(arm) == wanted]
        if not solutions:
            return dict(outcome, success=False, failure="no_same_branch_ik", step=step,
                        ms=(time.perf_counter() - began) * 1000)
        arm = min(solutions, key=lambda values: float(np.sum((values - previous) ** 2)))
        jump = float(np.degrees(np.max(np.abs(arm - previous))))
        largest = max(largest, jump)
        if jump > max_step_deg:
            return dict(outcome, success=False, failure="joint_jump", step=step, jump_deg=jump,
                        ms=(time.perf_counter() - began) * 1000)
        row[offset:offset + 7] = arm
        rows.append(row)
    end_error = float(np.max(np.abs(rows[-1] - goal)))
    outcome.update(max_step_deg=largest, end_error_rad=end_error)
    if end_error > 1e-4:
        return dict(outcome, success=False, failure="end_mismatch", ms=(time.perf_counter() - began) * 1000)
    rows[-1] = goal.copy()
    ik_ms = (time.perf_counter() - began) * 1000
    dense = np.asarray(densify(np.asarray(rows)))
    valid = validity.mask(torch.tensor(dense, device="cuda", dtype=torch.float32))
    torch.cuda.synchronize()
    outcome.update(dense_count=len(dense), ik_ms=ik_ms, ms=(time.perf_counter() - began) * 1000)
    if not bool(valid.all().item()):
        invalid = int((~valid).nonzero()[0].item())
        return dict(outcome, success=False, failure="collision", dense_index=invalid,
                    fraction=invalid / max(len(dense) - 1, 1))
    return dict(outcome, success=True, dense=dense)


def analytic_round_trip(planner, rows, arm_layout, repeats=300):
    side, index, offset = arm_layout
    errors = []
    for row in rows:
        tool, carriage = tool_and_carriage(planner, row, side)
        arm = row[offset:offset + 7]
        solutions = planner.analytic.solve(index, carriage @ tool, planner.analytic.swivel(index, arm), arm)
        errors.append(float(np.min(np.max(np.abs(solutions - arm), axis=1))) if len(solutions) else math.inf)
    tool, carriage = tool_and_carriage(planner, rows[0], side)
    arm = rows[0][offset:offset + 7]
    target, psi = carriage @ tool, planner.analytic.swivel(index, arm)
    began = time.perf_counter()
    for _ in range(repeats):
        planner.analytic.solve(index, target, psi, arm)
    return dict(rows=len(rows), max_error_rad=max(errors), within_1e6=sum(error < 1e-6 for error in errors),
                solve_us=(time.perf_counter() - began) / repeats * 1e6)


def analyse_transport(planner, start, goals, checker, search, top_k, max_step_deg):
    arm_layout = layout(planner)
    side, _, offset = arm_layout
    idle = planner.idle_indices
    began = time.perf_counter()
    pool = goal_pool(planner, start, checker)
    origin = describe(planner, start, arm_layout)
    same_idle = pool[np.all(np.abs(pool[:, idle] - start[idle]) <= 1e-5, axis=1)]
    infos = [describe(planner, row, arm_layout) for row in same_idle]
    tool_spread = max((float(np.linalg.norm(info["tool"][:3, 3] - infos[0]["tool"][:3, 3])) for info in infos), default=0.)
    matching = [(row, info) for row, info in zip(same_idle, infos) if info["branch"] == origin["branch"]]
    in_baseline = sum(any(np.allclose(row, goal, atol=1e-5) for goal in goals) for row, _ in matching)
    path, stats = search
    chosen = describe(planner, np.asarray(path[-1], dtype=np.float64), arm_layout) if path is not None else None
    report = dict(
        active_side=side, top_grasp=bool(planner.top_grasp), start_branch=origin["branch"],
        start_psi_deg=math.degrees(origin["psi"]), start_updown_m=origin["updown"],
        s1=dict(pool=len(pool), same_idle=len(same_idle), same_branch=len(matching),
                same_branch_in_baseline_top64=int(in_baseline), goal_tool_spread_m=tool_spread,
                branches=collections.Counter(str(info["branch"]) for info in infos).most_common(),
                psi_turn_deg=sorted(round(math.degrees(wrap(info["psi"] - origin["psi"])), 1) for _, info in matching)[:12],
                baseline_goal_branch=chosen and chosen["branch"],
                baseline_goal_psi_turn_deg=chosen and math.degrees(wrap(chosen["psi"] - origin["psi"])),
                baseline_flips_branch=bool(chosen and chosen["branch"] != origin["branch"])),
        baseline_search={key: value for key, value in (stats or {}).items() if not isinstance(value, (list, dict))},
    )
    if path is not None:
        report["baseline_raw_path"] = path_metrics(planner, checker, path, side)
    ranked = sorted(matching, key=lambda item: math.hypot(
        (item[1]["updown"] - origin["updown"]) * GOAL_WEIGHTS[0], wrap(item[1]["psi"] - origin["psi"])))
    validity = GpuValidity(checker, check_ground=True)
    lines = []
    for row, _ in ranked[:top_k]:
        outcome = direct_line(planner, start, row, arm_layout, validity, max_step_deg)
        if outcome["success"]:
            outcome["metrics"] = path_metrics(planner, checker, outcome["dense"], side)
        lines.append(outcome)
    report["s2"] = dict(tried=len(lines), succeeded=sum(line["success"] for line in lines),
                        failures=collections.Counter(line.get("failure") for line in lines if not line["success"]).most_common(),
                        lines=[{key: value for key, value in line.items() if key != "dense"} for line in lines])
    report["s3"] = analytic_round_trip(planner, [row for row, _ in matching[:200]] or [start], arm_layout)
    report["analysis_ms"] = (time.perf_counter() - began) * 1000
    planner.transport_lines = [line["dense"] for line in lines if line["success"]]
    return report


class SpikePlanner(TimedSingleArmPlanner):
    """Baseline planner that records an analysis of its transport inputs without changing the result."""

    spike_options = dict(top_k=8, max_step_deg=5.)

    def single_path(self, start, goals, checker, rotation=False, strict=None, budget=2.):
        search = super().single_path(start, goals, checker, rotation, strict, budget)
        if rotation and len(self.selected_task) == 1:
            self.transport_report = analyse_transport(
                self, np.asarray(start, dtype=np.float64), np.asarray(goals, dtype=np.float64),
                checker, search, **self.spike_options)
        return search


def timing_effect(planner, result, boxes, lines):
    names = list(result["joint_names"])
    phases = result["phases"]
    frames = np.asarray(result["frames"], dtype=np.float64)
    indices = [names.index(name) for name in ACTIVE_JOINTS]
    executed = frames[[i for i, phase in enumerate(phases) if phase == "transport"]]
    segment = next(item for item in result["timed_segments"] if item["name"] == "transport")
    planner.configure_task(boxes)
    snapshot = planner.snapshot
    checker = planner.checker(boxes, payload=True)
    side = planner.active_side
    report = dict(baseline=dict(duration_s=segment["duration_s"], parameterizer=segment["parameterizer"],
                                raw_waypoints=segment["raw_waypoints"], shortcut_waypoints=segment["shortcut_waypoints"],
                                executed=path_metrics(planner, checker, executed[:, indices], side)))
    if not lines:
        return report
    rows = np.repeat(executed[:1], len(lines[0]), axis=0)
    rows[:, indices] = lines[0]
    variants = {}
    for label, source in (("cartesian_no_shortcut", rows),
                          ("cartesian_shortcut", planner._shortcut_phase("transport", rows, boxes, snapshot)[0])):
        trajectory, validation = time_parameterize_path(source, names, *planner._timed_limits,
                                                        sample_dt=planner.trajectory_sample_dt)
        valid = planner._timed_phase_valid("transport", validation, boxes, snapshot)
        variants[label] = dict(duration_s=trajectory["duration_s"], waypoints=len(source), timed_samples_valid=valid,
                               executed=path_metrics(planner, checker, np.asarray(validation)[:, indices], side))
    report.update(variants)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--frozen", type=Path, help="frozen baseline-wall directory for equivalence of baseline stats")
    parser.add_argument("--top-k", type=int, default=8)
    parser.add_argument("--max-step-deg", type=float, default=5.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    planner = SpikePlanner(PlannerAssets())
    planner.spike_options = dict(top_k=args.top_k, max_step_deg=args.max_step_deg)
    planner.warmup_resources()
    summary = []
    for boxes in DEV_SINGLE:
        print(f"L1 {boxes}", flush=True)
        planner.transport_report, planner.transport_lines = None, []
        planner.configure_task(boxes)
        result = planner.full_cycle(boxes)
        report = dict(task=boxes, cycle_success=bool(result.get("success")),
                      blocked_stage=result.get("blocked_stage"), transport=planner.transport_report)
        if result.get("success"):
            try:
                report["s5"] = timing_effect(planner, result, boxes, planner.transport_lines)
            except Exception as error:  # keep the spike evidence even when one analysis fails
                report["s5_error"] = repr(error)
        if args.frozen:
            number = WALL_SEQUENCE.index(boxes) + 1
            frozen = json.loads((args.frozen / f"round-{number:02d}.json").read_text())
            stats, ours = frozen["transport_search_stats"], result.get("transport_search_stats") or {}
            keys = [key for key in stats if not key.endswith("_ms") and not isinstance(stats[key], (list, dict))]
            report["baseline_matches_frozen"] = all(stats[key] == ours.get(key) for key in keys)
            if len(frozen["frames"]) == len(result.get("frames", ())):
                report["frames_max_diff_vs_frozen"] = float(np.max(np.abs(
                    np.asarray(frozen["frames"]) - np.asarray(result["frames"]))))
        name = "-".join(f"{side}{box:02d}" for side, box in boxes.items())
        (args.output / f"{name}.json").write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str) + "\n")
        transport = report["transport"] or {}
        summary.append(dict(task=name, cycle_success=report["cycle_success"],
                            matches_frozen=report.get("baseline_matches_frozen"),
                            frames_max_diff=report.get("frames_max_diff_vs_frozen"),
                            same_branch=transport.get("s1", {}).get("same_branch"),
                            baseline_flips_branch=transport.get("s1", {}).get("baseline_flips_branch"),
                            lines_ok=f'{transport.get("s2", {}).get("succeeded")}/{transport.get("s2", {}).get("tried")}',
                            failures=transport.get("s2", {}).get("failures")))
        print(json.dumps(summary[-1], ensure_ascii=False), flush=True)
    (args.output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=1) + "\n")


if __name__ == "__main__":
    main()
