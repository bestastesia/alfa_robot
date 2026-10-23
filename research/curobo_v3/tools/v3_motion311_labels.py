#!/usr/bin/env python3
"""MOTION-311 labels: force every contact candidate (S1) and sampled placement goals (S2) through the real planner.

Per registry instance (task x docking offset), with the search recorder installed:
  baseline  the unchanged planner (untimed): contact pool, lazy ranking, selected candidate, cost
  S1        each contact candidate forced alone. Dual arm: one lazy batch holding only that contact
            with the full 2 s transport budget (the exact single-candidate batch of the network arm, whose
            smaller budget is a prefix of the same search); single arm: the contact order is [i]
  S2        for the baseline-selected root and one random successful root, 16 goals forced alone
            (dual: the 8 baseline goals + 8 distance-quantile goals of the 512 shortlist; single: the
            reached goal + 7 of the baseline 64 + 8 distance-quantile goals beyond them)
The conveyor suffix depends only on the transport end and is memoised per instance. Every run is
classified with v3_label_classes (timeouts are censored, never infeasible) and costed with
J_arc = 15-axis weighted joint arc (updown 10 cm = 15 deg) over the round + the planner's box rotation
soft cost (BoxRotationCost.edge_cost) on the densified transport path.

The run command processes pending instances until --max-minutes elapsed, then exits 3 so the caller can
release and re-acquire the GPU lock; finished instances are skipped on restart.
"""
import argparse
import copy
import hashlib
import json
import math
import os
import platform
import subprocess
import sys
import time
import traceback
from pathlib import Path

import numpy as np
import torch

from curobo_core.contracts import PlannerAssets
from curobo_core.planner import CycleBlocked
from v3_box_rotation_cost import BoxRotationCost
from v3_goal_matching import greedy_diverse
from v3_label_classes import SCENE_MARKERS, classify
from v3_scene_variants import DockingOffsetMixin, load_registry
from v3_search_recorder import SearchRecorder
from v3_single_arm_planner import ActiveRotationCost
from v3_timed_single_arm_planner import TimedSingleArmPlanner
from v3_wall_ik_benchmark import ACTIVE_JOINTS

WEIGHTS = np.array([math.radians(15) / .1] + [1.] * 14)
MOTION_PHASES = ("approach", "contact_approach", "extract", "transport", "conveyor_place", "return_home")
SCHEMA_VERSION = 1
MORE_WORK = 3


def array_digest(values):
    values = np.ascontiguousarray(values)
    return hashlib.sha1(str(values.shape).encode() + values.tobytes()).hexdigest()


def rounded(values, digits=6):
    return np.round(np.asarray(values, dtype=np.float64), digits).tolist()


def gpu_state():
    try:
        fields = subprocess.check_output(["nvidia-smi", "--query-gpu=clocks.sm,temperature.gpu,utilization.gpu",
                                          "--format=csv,noheader,nounits"], text=True).split(",")
        return dict(sm_clock_mhz=int(fields[0]), temperature_c=int(fields[1]), utilization_percent=int(fields[2]))
    except (OSError, subprocess.SubprocessError, ValueError, IndexError):
        return None


class LabelPlanner(DockingOffsetMixin, TimedSingleArmPlanner):
    """The baseline planner plus forcing hooks; with nothing forced it plans exactly like the baseline."""

    timed = False

    def reset_instance(self, recorder, memo=True):
        self.recorder = recorder
        self.forced_contact = self.forced_goal = self.goal_probe = None
        self.instance_contacts = None
        self.suffix_memo = {} if memo else None
        self.trace = []
        self.last_suffix = None

    def time_parameterize_result(self, result):
        return super().time_parameterize_result(result) if self.timed else result

    def rank_contacts(self, boxes):
        if self.forced_contact is None:
            ranked = super().rank_contacts(boxes)
            self.instance_contacts = self.analytic_result.js_solution.position.detach().cpu().numpy().reshape(-1, 15).astype(np.float64)
            return ranked
        self.ik_solver(boxes)
        contacts = self.analytic_result.js_solution.position.detach().cpu().numpy().reshape(-1, 15).astype(np.float64)
        if self.instance_contacts is None or not np.array_equal(contacts, self.instance_contacts):
            raise RuntimeError("forced run sees a different contact pool than the baseline run")
        self.prediction_records = []
        return [self.forced_contact] + [index for index in range(len(contacts)) if index != self.forced_contact]

    def lazy_batches(self, boxes, ranked):
        if self.forced_contact is None:
            yield from super().lazy_batches(boxes, ranked)
            return
        yield [self.forced_contact], 1, self.lazy_search_budget_ms, "forced"

    def single_contact_order(self, boxes, contacts):
        contacts = np.asarray(contacts, dtype=np.float64)
        if self.forced_contact is None:
            self.instance_contacts = contacts.copy()
            return super().single_contact_order(boxes, contacts)
        if self.instance_contacts is None or not np.array_equal(contacts, self.instance_contacts):
            raise RuntimeError("forced run sees a different contact pool than the baseline run")
        return [self.forced_contact]

    def placement_goal_order(self, root, shortlisted, distances, baseline_scores):
        if self.forced_goal is not None:
            pool_digest, index = self.forced_goal
            if array_digest(shortlisted.cpu().numpy()) != pool_digest:
                raise RuntimeError("forced goal run sees a different 512-goal shortlist")
            return [index]
        if self.goal_probe is None:
            return None
        order = torch.argsort(baseline_scores()).cpu().tolist()
        weights = torch.tensor(WEIGHTS.tolist(), device="cuda")
        self.goal_probe.append(dict(root=root.cpu().numpy().astype(np.float64), pool=shortlisted.cpu().numpy(),
                                    distances=distances.cpu().numpy().astype(np.float64),
                                    baseline=greedy_diverse(shortlisted, order, weights), order=order))
        return order

    def order_transport_goals(self, start, candidates):
        order = super().order_transport_goals(start, candidates)
        if self.forced_goal is not None:
            matches = np.flatnonzero(np.all(candidates == self.forced_goal[None], axis=1))
            if len(matches) != 1:
                raise RuntimeError("forced single-arm goal is not unique in the filtered pool")
            return matches
        if self.goal_probe is not None:
            pool = order[:512]
            self.goal_probe.append(dict(root=np.asarray(start, dtype=np.float64), pool=np.asarray(candidates[pool]),
                                        distances=np.linalg.norm((candidates[pool] - start) * WEIGHTS, axis=1),
                                        baseline=list(range(min(64, len(pool)))), order=list(range(len(pool)))))
        return order

    def cartesian_single(self, contact, distances, along_normal=False):
        frames, detail = super().cartesian_single(contact, distances, along_normal)
        self.trace.append(dict(call="cartesian_single", along_normal=bool(along_normal), ok=frames is not None,
                               detail=None if frames is not None else str(detail)[:200]))
        return frames, detail

    def single_path(self, start, goals, checker, rotation=False, strict=None, budget=2.):
        path, stats = super().single_path(start, goals, checker, rotation, strict, budget)
        self.trace.append(dict(call="single_path", goals=int(np.asarray(goals).reshape(-1, 15).shape[0]), rotation=bool(rotation),
                               ok=path is not None, failure=(stats or {}).get("failure")))
        return path, stats

    def conveyor_suffix(self, boxes, report, append, progress):
        if self.suffix_memo is None:
            mark = self.recorder.mark()
            began = time.perf_counter()
            try:
                return super().conveyor_suffix(boxes, report, append, progress)
            finally:
                self.last_suffix = dict(cached=False, ms=(time.perf_counter() - began) * 1000, searches=self.recorder.since(mark))
        key = array_digest(np.round(np.asarray(report["frames"][-1], dtype=np.float64), 9))
        entry = self.suffix_memo.get(key)
        if entry is None:
            rows, before, mark, trace_mark = [], set(report), self.recorder.mark(), len(self.trace)

            def recording(values, phase, payload):
                rows.append((np.array(values, dtype=np.float64), phase, payload))
                append(values, phase, payload)
            began = time.perf_counter()
            blocked = None
            try:
                super().conveyor_suffix(boxes, report, recording, progress)
            except CycleBlocked as error:
                blocked = (error.stage, error.details)
            entry = dict(rows=rows, fields={name: copy.deepcopy(report[name]) for name in report if name not in before},
                         blocked=blocked, ms=(time.perf_counter() - began) * 1000, searches=self.recorder.since(mark),
                         trace=self.trace[trace_mark:])
            self.suffix_memo[key] = entry
            self.last_suffix = dict(cached=False, ms=entry["ms"])
        else:
            for values, phase, payload in entry["rows"]:
                append(values, phase, payload)
            report.update(copy.deepcopy(entry["fields"]))
            self.trace.extend(entry["trace"])
            self.last_suffix = dict(cached=True, ms=entry["ms"])
        self.last_suffix["searches"] = entry["searches"]
        if entry["blocked"] is not None:
            raise CycleBlocked(entry["blocked"][0], entry["blocked"][1], report)


def round_costs(planner, boxes, result):
    """Weighted 15-axis arc per phase and the box rotation soft cost on the densified transport path."""
    names = result["joint_names"]
    frames = np.asarray(result["frames"], dtype=np.float64)[:, [names.index(name) for name in ACTIVE_JOINTS]]
    phases = result["phases"]
    steps = np.linalg.norm(np.diff(frames, axis=0) * WEIGHTS, axis=1)
    by_phase = {}
    for step, phase in zip(steps, phases[1:]):
        by_phase[phase] = by_phase.get(phase, 0.) + float(step)
    transport = frames[[index for index, phase in enumerate(phases) if phase == "transport"]]
    rotation = 0.
    if len(transport) > 1:
        checker = planner.checker(boxes, payload=True)
        values = torch.tensor(transport, device="cuda", dtype=torch.float32)
        metric = (BoxRotationCost(checker, ACTIVE_JOINTS, values[0], values[-1], planner.top_grasp) if len(boxes) == 2 else
                  ActiveRotationCost(checker, values[0], values[-1], planner.top_grasp, planner.active_side))
        rotation = float(metric.edge_cost(values[:-1].contiguous(), values[1:].contiguous()).sum())
    arc = float(steps.sum())
    ends = {}
    for index, phase in enumerate(phases):
        if phase in MOTION_PHASES:
            first, _ = ends.get(phase, (frames[index - 1] if index else frames[index], None))
            ends[phase] = (first, frames[index])
    return dict(arc=arc, arc_by_phase=by_phase, rotation_cost=rotation, j_arc=arc + rotation,
                transport_end=rounded(transport[-1]) if len(transport) else None,
                phase_ends={phase: [rounded(first), rounded(last)] for phase, (first, last) in ends.items()})


def compact_search(call):
    return dict(module=call["module"], seed=call["seed"], budget_s=call["budget_s"], starts=call["starts"], goals=call["goals"],
                success=call["success"], start_ms=round(call["start_ms"], 3), wall_ms=round(call["wall_ms"], 3), **call["stats"])


def timings(result):
    keys = ("total_ms", "timing_ms", "pipeline_timing_ms", "validation_detail_ms", "conveyor_suffix_ms")
    output = {key: copy.deepcopy(result.get(key)) for key in keys if key in result}
    lazy = result.get("lazy_extraction")
    if lazy:
        output["lazy"] = dict(timings_ms=lazy.get("timings_ms"), batches=lazy.get("batches"),
                              checked_candidates=lazy.get("checked_candidates"))
    return output


def extraction_detail(result, contact):
    """Dual: the preparation attempt of this contact; single: the logged extraction rejection (success is unlogged)."""
    attempts = (result.get("multiroot_preparation") or {}).get("attempts") or result.get("attempts") or []
    for attempt in attempts:
        if attempt.get("contact_index") == contact or attempt.get("candidate") == contact + 1:
            return attempt.get("detail") if attempt.get("success") else str(attempt.get("detail"))[:200]
    return None


def forced_run(planner, boxes, kind, contact=None, goal=None, probe=False):
    planner.forced_contact, planner.forced_goal = contact, goal
    planner.goal_probe = [] if probe else None
    planner.trace, planner.last_suffix = [], None
    mark = planner.recorder.mark()
    began = time.perf_counter()
    start_ms = (began - planner.recorder.t0) * 1000
    error = None
    try:
        planner.configure_task(boxes)
        result = planner.full_cycle(boxes)
    except Exception as exception:  # a crash is a labelled outcome, not a reason to stop the batch
        result, error = None, f"{type(exception).__name__}: {exception}"[:500]
        traceback.print_exc()
    wall_ms = (time.perf_counter() - began) * 1000
    planner.forced_contact = planner.forced_goal = None
    label, stage = classify(kind, result, planner.trace)
    if error is not None and any(marker in error for marker in SCENE_MARKERS):
        label = "scene_infeasible"
    entry = dict(class_=label, stage=stage, details=None if result is None else str(result.get("blocked_details") or "")[:300] or None,
                 error=error, start_ms=start_ms, wall_ms=wall_ms, searches=[compact_search(call) for call in planner.recorder.since(mark)],
                 suffix=None if planner.last_suffix is None else dict(cached=planner.last_suffix["cached"], ms=planner.last_suffix["ms"],
                                                                     searches=[compact_search(call) for call in planner.last_suffix["searches"]]),
                 trace=planner.trace)
    if result is not None:
        entry["timings"] = timings(result)
        if contact is not None:
            entry["extraction"] = extraction_detail(result, contact)
        if result.get("success"):
            entry["cost"] = round_costs(planner, boxes, result)
    probes, planner.goal_probe = planner.goal_probe, None
    return entry, result, probes


def matched_goal(pool, transport_end):
    if transport_end is None or pool is None or not len(pool):
        return None
    error = np.abs(np.asarray(pool, dtype=np.float64) - np.asarray(transport_end)[None]).max(axis=1)
    index = int(error.argmin())
    return index if error[index] < 1e-5 else None


def s2_goals(kind, probe, reached):
    count = len(probe["pool"])
    baseline = list(probe["baseline"])
    if kind == "dual":
        chosen = baseline[:]
        rest = [index for index in range(count) if index not in chosen]
    else:
        inside = [index for index in baseline if index != reached]
        sampled = [inside[int(round(q * (len(inside) - 1)))] for q in np.linspace(0, 1, 7)] if inside else []
        chosen = ([reached] if reached is not None else []) + sampled
        rest = [index for index in range(len(baseline), count)]
    extra = [rest[int(round(q * (len(rest) - 1)))] for q in np.linspace(0, 1, 8)] if rest else []
    return list(dict.fromkeys(chosen + extra))


def run_instance(planner, item, args):
    boxes = {side: int(box) for side, box in item["task"].items()}
    kind = item["kind"]
    planner.scene_offset = tuple(float(value) for value in item["offset_m"])
    record = dict(schema_version=SCHEMA_VERSION, instance=item["instance"], task=boxes, kind=kind, split=item["split"],
                  group=item["group"], offset_m=item["offset_m"], seed_offset=args.seed_offset, budget_scale=args.budget_scale,
                  suffix_memo=not args.no_suffix_memo,
                  gpu_before=gpu_state(), load_before=os.getloadavg())
    began = time.perf_counter()
    with SearchRecorder(args.seed_offset, budget_scale=args.budget_scale) as recorder:
        planner.reset_instance(recorder, memo=not args.no_suffix_memo)
        baseline, result, _ = forced_run(planner, boxes, kind)
        pool = planner.instance_contacts
        record["context"] = dict(top_grasp=bool(planner.top_grasp), base_advance_m=float(planner.base_advance),
                                 contact_poses={name: dict(position=rounded(pose["position"]), quaternion=rounded(pose["quaternion"]))
                                                for name, pose in planner.contact_poses_for_task(boxes).items()},
                                 box_centers={side: rounded(planner.snapshot.object(f"wall_box_{box:02d}").pose.position)
                                              for side, box in boxes.items()},
                                 home=rounded(planner.home_values))
        if result is not None:
            baseline["selected_index"] = (result.get("selected_original_contact_index") if kind == "dual" else
                                          (result.get("selected_candidate") - 1 if result.get("selected_candidate") else None))
            lazy = result.get("lazy_extraction") or {}
            baseline["ranking"] = lazy.get("ranking")
            baseline["predicted_endpoints"] = [None if row.get("endpoint") is None else rounded(row["endpoint"])
                                               for row in lazy.get("predictions") or []] or None
        record["baseline"] = baseline
        record["contacts"] = None if pool is None else rounded(pool, 7)
        s1, probes = [], {}
        if args.stage in ("s1", "all") and pool is not None:
            subset = args.candidate_map.get(item["instance"]) if args.candidate_map is not None else None
            for index in range(len(pool)):
                if subset is not None and index not in subset:
                    continue
                entry, _, probe = forced_run(planner, boxes, kind, contact=index, probe=True)
                entry["index"] = index
                if probe:
                    probes[index] = probe[-1]
                    entry["pool_digest"] = array_digest(probe[-1]["pool"])
                    entry["reached_goal"] = matched_goal(probe[-1]["pool"], (entry.get("cost") or {}).get("transport_end"))
                s1.append(entry)
                print(json.dumps(dict(instance=item["instance"], s1=index, label=entry["class_"], stage=entry["stage"],
                                      j=(entry.get("cost") or {}).get("j_arc"), ms=round(entry["wall_ms"])), ensure_ascii=False), flush=True)
        if args.timed:
            planner.timed = True
            ranked = sorted((entry for entry in s1 if entry["class_"] == "success"), key=lambda entry: entry["cost"]["j_arc"])
            picks = [entry for entry in ranked if entry["index"] == baseline.get("selected_index")]
            for q in np.linspace(0, 1, args.timed):
                entry = ranked[int(round(q * (len(ranked) - 1)))] if ranked else None
                if entry is not None and entry not in picks:
                    picks.append(entry)
            for entry in picks:
                timed, timed_result, _ = forced_run(planner, boxes, kind, contact=entry["index"])
                entry["timed"] = dict(class_=timed["class_"], j_arc_timed=(timed.get("cost") or {}).get("j_arc"),
                                      motion_duration_s=(timed_result or {}).get("motion_duration_s"),
                                      trajectory_timing_ms=(timed_result or {}).get("trajectory_timing_ms"),
                                      segments=[[segment["name"], segment["duration_s"]] for segment in (timed_result or {}).get("timed_segments", [])])
            planner.timed = False
        record["s1"] = s1
        record["s2"], record["s2_pools"] = [], {}
        if args.stage == "all" and s2_selected(item, args):
            rng = np.random.default_rng(int(hashlib.sha1(item["instance"].encode()).hexdigest()[:8], 16))
            successes = [entry["index"] for entry in s1 if entry["class_"] == "success" and entry["index"] in probes]
            roots = [baseline.get("selected_index")] if baseline.get("selected_index") in successes else []
            others = [index for index in successes if index not in roots]
            while others and len(roots) < 2:
                roots.append(int(others.pop(int(rng.integers(len(others))))))
            for root in roots:
                probe = probes[root]
                goals = s2_goals(kind, probe, s1[root].get("reached_goal"))
                record["s2_pools"][str(root)] = dict(root=rounded(probe["root"], 7), pool=rounded(probe["pool"], 7), distances=rounded(probe["distances"]),
                                                    baseline=list(probe["baseline"]), labeled=goals)
                for goal_index in goals:
                    forced = (array_digest(probe["pool"]), goal_index) if kind == "dual" else np.asarray(probe["pool"][goal_index])
                    entry, _, _ = forced_run(planner, boxes, kind, contact=root, goal=forced)
                    entry.update(root=root, goal=goal_index, in_baseline=goal_index in probe["baseline"])
                    record["s2"].append(entry)
                    print(json.dumps(dict(instance=item["instance"], s2=[root, goal_index], label=entry["class_"],
                                          j=(entry.get("cost") or {}).get("j_arc")), ensure_ascii=False), flush=True)
    record.update(wall_s=time.perf_counter() - began, gpu_after=gpu_state(), load_after=os.getloadavg())
    return record


def s2_selected(item, args):
    if args.s2 == "all":
        return True
    if args.s2 == "half":
        return int(hashlib.sha1(item["instance"].encode()).hexdigest()[:8], 16) % 2 == 0
    return False


def provenance(args):
    worktree = os.environ.get("MOTION311_WORKTREE")
    def git(*command):
        try:
            return subprocess.check_output(["git", "-C", worktree, *command], text=True).strip()
        except (OSError, subprocess.SubprocessError, TypeError):
            return None
    library = Path(__file__).resolve().parents[1] / "generated/v3_analytic_071cb95/libv3_analytic_bridge.so"
    import curobo
    return dict(created_at=time.strftime("%Y-%m-%dT%H:%M:%S%z"), argv=sys.argv, runtime=str(Path(__file__).resolve().parents[1]),
                worktree=worktree, head=git("rev-parse", "HEAD"), dirty=bool(git("status", "--porcelain")),
                analytic_library_sha256=hashlib.sha256(library.read_bytes()).hexdigest(),
                registry_sha256=load_registry(args.registry)["sha256"], python=platform.python_version(), torch=torch.__version__,
                curobo=getattr(curobo, "__version__", None), gpu=torch.cuda.get_device_name(0), host=platform.node())


def run(args):
    registry = load_registry(args.registry)
    wanted = set(args.instances.split(",")) if args.instances else None
    items = [item for item in registry["instances"] if item["group"] in args.groups.split(",") and item["kind"] in args.kinds.split(",")
             and (wanted is None or item["instance"] in wanted)]
    folder = args.output / "instances"
    folder.mkdir(parents=True, exist_ok=True)
    pending = [item for item in items if not (folder / f"{item['instance']}.json").exists()]
    print(json.dumps(dict(selected=len(items), pending=len(pending)), ensure_ascii=False), flush=True)
    if not pending:
        return 0
    with open(args.output / "provenance.jsonl", "a") as handle:
        handle.write(json.dumps(provenance(args), ensure_ascii=False) + "\n")
    planner = LabelPlanner(PlannerAssets(), .025)
    planner.warmup_resources()
    started = time.perf_counter()
    for item in pending:
        if time.perf_counter() - started > args.max_minutes * 60:
            return MORE_WORK
        record = run_instance(planner, item, args)
        path = folder / f"{item['instance']}.json"
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(record, ensure_ascii=False, default=float) + "\n")
        temporary.rename(path)
        counts = {}
        for entry in record["s1"]:
            counts[entry["class_"]] = counts.get(entry["class_"], 0) + 1
        print(json.dumps(dict(done=item["instance"], wall_s=round(record["wall_s"], 1), baseline=record["baseline"]["class_"],
                              s1=counts, s2=len(record["s2"])), ensure_ascii=False), flush=True)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--groups", default="train,dev")
    parser.add_argument("--kinds", default="dual,single")
    parser.add_argument("--instances", help="comma separated instance ids (default: all selected)")
    parser.add_argument("--stage", choices=("baseline", "s1", "all"), default="all",
                        help="all = S1 plus S2 for the instances chosen by --s2")
    parser.add_argument("--s2", choices=("none", "half", "all"), default="half")
    parser.add_argument("--seed-offset", type=int, default=0)
    parser.add_argument("--budget-scale", type=float, default=1.0)
    parser.add_argument("--timed", type=int, default=0,
                        help="diagnostic: time-parameterize the baseline pick and K J_arc-quantile S1 successes")
    parser.add_argument("--no-suffix-memo", action="store_true", help="L1 check: plan every conveyor suffix afresh")
    parser.add_argument("--candidate-map", type=Path, help="JSON {instance: [S1 indices]}: re-run only these candidates (censoring probe)")
    parser.add_argument("--max-minutes", type=float, default=25.)
    args = parser.parse_args()
    args.candidate_map = None if args.candidate_map is None else {key: set(value) for key, value in json.loads(args.candidate_map.read_text()).items()}
    raise SystemExit(run(args))


if __name__ == "__main__":
    main()
