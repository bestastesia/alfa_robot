#!/usr/bin/env python3
"""MOTION-311 D3 labels: forced (carry IK g, place IK p) pairs through the real conveyor suffix.

Per mode (dual/single-left/single-right x side/top) a representative train/dev task is configured and the
placement candidate set P is enumerated once in the exact suffix context (v3_place_ik). Carry IKs g are sampled
from the transport ends observed in the P3 labels of that mode. For each g the suffix (parking, place RRT,
release, return-home RRT) runs from a synthetic transport end with the place IK forced to p: 4 p with the best
analytic T_lb(g->p) + T_lb(p->Home) (the choices a T_lb-only selection would make) and 4 random p. The cuRobo
place-IK solver is not built for forced runs and return-home plans are memoised per p (deterministic).
The independence check re-runs pairs under other tasks/offsets of the same mode.
"""
import argparse
import hashlib
import json
import time
from pathlib import Path

import numpy as np
import torch

from curobo_core.contracts import PlannerAssets
from curobo_core.planner import CycleBlocked
from v3_batched_loaded_search import GpuValidity
from v3_label_classes import classify
from v3_motion311_labels import LabelPlanner, compact_search, round_costs, rounded
from v3_place_ik import active_velocities, dual_place_set, single_place_set, time_to
from v3_search_recorder import SearchRecorder
from v3_wall_ik_benchmark import ACTIVE_JOINTS

MODES = {"dual-side": dict(task={"left": 24, "right": 21}, check=[{"left": 23, "right": 20}, {"left": 14, "right": 10}]),
         "dual-top": dict(task={"left": 8, "right": 2}, check=[{"left": 9, "right": 5}, {"left": 4, "right": 1}]),
         "single-right-side": dict(task={"right": 21}, check=[{"right": 17}, {"right": 10}]),
         "single-left-side": dict(task={"left": 23}, check=[{"left": 19}, {"left": 13}]),
         "single-right-top": dict(task={"right": 1}, check=[{"right": 5}, {"right": 7}]),
         "single-left-top": dict(task={"left": 3}, check=[{"left": 8}, {"left": 9}])}
MORE_WORK = 3
WEIGHTS = np.array([np.radians(15) / .1] + [1.] * 14)


class CaptureDone(Exception):
    pass


class PlaceLabelPlanner(LabelPlanner):
    """LabelPlanner (search/IK traces, docking offsets) without the suffix memo, plus forced or captured place IKs."""

    forced_place = None
    capture = None
    return_memo = None

    def conveyor_ik_solver(self, boxes):
        return None if (self.forced_place is not None or self.capture is not None) else super().conveyor_ik_solver(boxes)

    def dual_place_candidates(self, start, targets, validity, solver, report):
        if self.capture is not None:
            rows, params = dual_place_set(self, targets, validity)
            self.capture.update(rows=rows, params=params, targets={side: value.tolist() for side, value in targets.items()})
            raise CaptureDone()
        if self.forced_place is not None:
            candidate = torch.tensor(self.forced_place[None], device="cuda", dtype=torch.float32)
            report["conveyor_goal_ik"] = dict(mode="forced")
            if not bool(validity.mask(candidate).all()):
                raise CycleBlocked("放箱IK碰撞", "强制的放箱IK在当前场景碰撞或越限", report)
            return candidate
        return super().dual_place_candidates(start, targets, validity, solver, report)

    def single_place_candidates(self, targets, start, checker):
        if self.capture is not None:
            rows, params = single_place_set(self, targets, checker)
            self.capture.update(rows=rows, params=params, targets={side: value.tolist() for side, value in targets.items()})
            raise CaptureDone()
        if self.forced_place is not None:
            if not bool(GpuValidity(checker, check_ground=True).mask(torch.tensor(self.forced_place[None], device="cuda", dtype=torch.float32)).all()):
                raise CycleBlocked("放箱IK碰撞", "强制的放箱IK在当前场景碰撞或越限", {})
            return self.forced_place[None].astype(np.float32)
        return super().single_place_candidates(targets, start, checker)

    def joint_plan(self, start, target, checker, validity, seed):
        if self.return_memo is None:
            return super().joint_plan(start, target, checker, validity, seed)
        key = (np.round(np.asarray(start, dtype=np.float64), 9).tobytes(), np.round(np.asarray(target, dtype=np.float64), 9).tobytes())
        if key not in self.return_memo:
            path, stats = super().joint_plan(start, target, checker, validity, seed)
            self.return_memo[key] = (None if path is None else np.array(path), dict(stats or {}))
        path, stats = self.return_memo[key]
        return (None if path is None else path.copy()), dict(stats)


def mobile_row(planner, values):
    mapping = dict(zip(ACTIVE_JOINTS, np.asarray(values, dtype=np.float64)))
    return [float(mapping.get(name, 0.)) for name in planner.mobile_robot["kinematics"]["cspace"]["joint_names"]]


def run_suffix(planner, boxes, carry):
    """Conveyor suffix from a synthetic transport end; returns (result-like dict, wall ms)."""
    names = list(planner.mobile_robot["kinematics"]["cspace"]["joint_names"])
    report = dict(success=False, task=boxes, joint_names=names, frames=[mobile_row(planner, carry)], phases=["transport"], payload=[True])

    def append(rows, phase, payload):
        for row in rows:
            mapping = dict(zip(ACTIVE_JOINTS, row)) if len(row) == 15 else dict(zip(names, row))
            values = [float(mapping.get(name, 0.)) for name in names]
            if report["frames"] and phase not in ("release", "attach") and np.allclose(report["frames"][-1], values, atol=1e-10, rtol=0.):
                continue
            report["frames"].append(values)
            report["phases"].append(phase)
            report["payload"].append(payload)
    began = time.perf_counter()
    try:
        planner.conveyor_suffix(boxes, report, append, lambda text: None)
        report["success"] = True
    except CycleBlocked as error:
        report["blocked_stage"], report["blocked_details"] = error.stage, str(error.details)
    return report, (time.perf_counter() - began) * 1000


def label_pair(planner, boxes, kind, carry, place, recorder):
    """One suffix run from carry IK `carry`; `place` = None keeps the baseline placement rule (cuRobo IK, nearest first)."""
    planner.forced_place = None if place is None else np.asarray(place, dtype=np.float32)
    planner.trace = []
    mark = recorder.mark()
    try:
        planner.configure_task(boxes)
        report, wall_ms = run_suffix(planner, boxes, carry)
    finally:
        planner.forced_place = None
    label, stage = classify(kind, report, getattr(planner, "trace", []))
    entry = dict(class_=label, stage=stage, details=report.get("blocked_details"), wall_ms=wall_ms,
                 searches=[compact_search(call) for call in recorder.since(mark)])
    if report["success"]:
        cost = round_costs(planner, boxes, report)
        entry["cost"] = {key: cost[key] for key in ("arc_by_phase", "t_path_by_phase", "t_lb_by_phase", "phase_ends")}
        ends = cost["phase_ends"].get("conveyor_place")
        entry["placed"] = None if ends is None else ends[1]
    return entry


def observed_carries(labels, mode, cap, rng):
    """Distinct transport ends (carry IKs) of the P3 labels whose task matches the mode."""
    kind, *rest = mode.split("-")
    side, grasp = (rest[0], rest[1]) if kind == "single" else (None, rest[0])
    found = {}
    for path in sorted(Path(labels, "instances").glob("*.json")):
        record = json.loads(path.read_text())
        if record["kind"] != kind or record["context"]["top_grasp"] != (grasp == "top") or (side and side not in record["task"]):
            continue
        for entry in record["s1"] + record["s2"]:
            end = (entry.get("cost") or {}).get("transport_end")
            if end is not None:
                found.setdefault(tuple(np.round(end, 5)), end)
    rows = list(found.values())
    order = rng.permutation(len(rows))[:cap]
    return [rows[index] for index in order]


def mode_contexts(registry, mode, rng):
    """(task, docking offset) of the train/dev instances matching the mode, shuffled."""
    kind, *rest = mode.split("-")
    side, grasp = (rest[0], rest[1]) if kind == "single" else (None, rest[0])
    items = [item for item in json.loads(Path(registry).read_text())["instances"] if item["group"] in ("train", "dev")
             and item["kind"] == kind and item["top_grasp"] == (grasp == "top") and (side is None or side in item["task"])]
    order = rng.permutation(len(items))
    return [({key: int(value) for key, value in items[index]["task"].items()}, items[index]["offset_m"]) for index in order]


def pick_places(carry, places, home, velocity, rng, placed=None, best=4, near=2, random=2):
    """4 best analytic T_lb(g->p) + T_lb(p->Home), 2 enumerated candidates nearest to the baseline-rule placement, 2 random."""
    score = time_to(places, np.asarray(carry), velocity) + time_to(places, home, velocity)
    chosen = list(np.argsort(score, kind="stable")[:best])
    if placed is not None:
        distance = np.linalg.norm((places - np.asarray(placed)) * WEIGHTS, axis=1)
        chosen += [index for index in np.argsort(distance, kind="stable") if index not in chosen][:near]
    rest = [index for index in range(len(places)) if index not in chosen]
    picks = rng.choice(rest, min(random, len(rest)), replace=False) if rest else []
    return [int(index) for index in chosen] + [int(index) for index in picks]


def run(args):
    planner = PlaceLabelPlanner(PlannerAssets(), .025)
    planner.warmup_resources()
    started = time.perf_counter()
    for mode in args.modes.split(","):
        spec = MODES[mode]
        kind = "dual" if mode.startswith("dual") else "single"
        folder = args.output / mode
        folder.mkdir(parents=True, exist_ok=True)
        rng = np.random.default_rng(int(hashlib.sha1(mode.encode()).hexdigest()[:8], 16))
        carries = observed_carries(args.labels, mode, args.carries, rng)
        place_file = folder / "place_set.npz"
        with SearchRecorder(0) as recorder:
            planner.reset_instance(recorder, memo=False)
            if not place_file.exists():
                planner.configure_task(spec["task"])
                planner.capture = {}
                try:
                    run_suffix(planner, spec["task"], carries[0])
                except CaptureDone:
                    pass
                captured, planner.capture = planner.capture, None
                np.savez(place_file, rows=captured["rows"], params=captured["params"], targets=json.dumps(captured["targets"]))
                print(json.dumps(dict(mode=mode, place_set=len(captured["rows"])), ensure_ascii=False), flush=True)
            place = np.load(place_file)
            places = place["rows"]
            velocity = active_velocities(planner)
            home = np.asarray(planner.home_values, dtype=np.float64)
            out = folder / "labels.jsonl"
            done = {}
            if out.exists():
                for line in out.read_text().splitlines():
                    row = json.loads(line)
                    done[row["key"]] = row
            contexts = mode_contexts(args.registry, mode, rng)
            planner.return_memo = None  # contexts rotate per carry, so a return-home memo would not be exact
            with open(out, "a") as handle:
                def label(carry_index, carry, place_index, boxes, offset):
                    key = f"{carry_index}-{'baseline' if place_index is None else place_index}"
                    if key in done:
                        return done[key]
                    planner.scene_offset = tuple(offset)
                    place_row = None if place_index is None else places[place_index]
                    entry = label_pair(planner, boxes, kind, carry, place_row, recorder)
                    reached = place_row if place_row is not None else entry.get("placed")
                    entry.update(key=key, mode=mode, carry_index=carry_index, place_index=place_index, task=boxes, offset_m=list(offset),
                                 carry=rounded(carry, 7),
                                 t_lb_place=None if reached is None else float(time_to(reached, np.asarray(carry), velocity)),
                                 t_lb_return=None if reached is None else float(time_to(reached, home, velocity)))
                    handle.write(json.dumps(entry, ensure_ascii=False, default=float) + "\n")
                    handle.flush()
                    done[key] = entry
                    return entry
                for carry_index, carry in enumerate(carries):
                    boxes, offset = contexts[carry_index % len(contexts)]
                    if time.perf_counter() - started > args.max_minutes * 60:
                        return MORE_WORK
                    baseline = label(carry_index, carry, None, boxes, offset)
                    for place_index in pick_places(carry, places, home, velocity, rng, baseline.get("placed")):
                        if time.perf_counter() - started > args.max_minutes * 60:
                            return MORE_WORK
                        label(carry_index, carry, place_index, boxes, offset)
            planner.scene_offset = (0., 0.)
            check = folder / "independence.jsonl"
            if args.check_pairs and not check.exists():
                rows = [row for row in (json.loads(line) for line in out.read_text().splitlines()) if row["place_index"] is not None][:args.check_pairs]
                with open(check, "w") as handle:
                    for row in rows:
                        for boxes, offset in [(spec["check"][0], (0., 0.)), (spec["check"][1], (.03, -.02)), (spec["task"], (-.04, .03))]:
                            planner.scene_offset = offset
                            planner.return_memo = None
                            entry = label_pair(planner, boxes, kind, row["carry"], places[row["place_index"]], recorder)
                            handle.write(json.dumps(dict(key=row["key"], task=boxes, offset=offset, reference_class=row["class_"],
                                                         reference_t=(row.get("cost") or {}).get("t_path_by_phase"), class_=entry["class_"],
                                                         t=(entry.get("cost") or {}).get("t_path_by_phase")), ensure_ascii=False) + "\n")
                planner.scene_offset = (0., 0.)
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--labels", type=Path, required=True, help="P3 label folder (source of observed carry IKs)")
    parser.add_argument("--registry", type=Path, default=Path(__file__).resolve().parents[1] / "config/motion311_registry.json")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--modes", default=",".join(MODES))
    parser.add_argument("--carries", type=int, default=200)
    parser.add_argument("--check-pairs", type=int, default=0)
    parser.add_argument("--max-minutes", type=float, default=25.)
    args = parser.parse_args()
    raise SystemExit(run(args))


if __name__ == "__main__":
    main()
