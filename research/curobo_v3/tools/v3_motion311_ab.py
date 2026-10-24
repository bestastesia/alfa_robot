#!/usr/bin/env python3
"""MOTION-311 v2 online A/B (MOTION-337): arms A (baseline), B0 (T_lb only), B (networks) [and Bx] on registry instances.

Per instance the arms plan the same scene (task, docking offset) back to back, in an order that alternates between
instances (as listed on even instances, reversed on odd ones); each arm is a complete round including its own
fallback. A warm-up round per arm (not recorded) starts every chunk, so no arm carries the cold start. Recorded per arm:
outcome class and stage (v3_label_classes), T_lb_path / T_lb / J_arc by phase (round_costs), motion duration,
planning wall time (incl. features, inference, verification and any fallback round), lazy batches, the stage
selection report and a digest of the frames (reproducibility, G3'). Models and V3 tables come from --models
(n1.npz, n2.npz, n3_place.npz with sha256 from train_v2_report.json; assets/<mode>/). Resumable per instance;
--max-minutes ends a chunk with exit status 3 (more work).
"""
import argparse
import json
import os
import time
import traceback
from pathlib import Path

import numpy as np

import v3_candidate_ranker as ranker
import v3_triple_ranker as triple
from curobo_core.contracts import PlannerAssets
from v3_label_classes import classify
from v3_motion311_labels import array_digest, compact_search, gpu_state, provenance, round_costs, timings
from v3_scene_variants import DockingOffsetMixin, load_registry
from v3_search_recorder import SearchRecorder
from v3_triple_selection_planner import TripleSelectionPlanner

MORE_WORK = 3
WARMUP_TASK = {"left": 22, "right": 17}
MODELS = {"n1": ("n1.npz", triple.CONTACT_NAMES, "n1"), "n2": ("n2.npz", triple.TRANSPORT_NAMES, "n2"),
          "n3": ("n3_place.npz", triple.PLACE_NAMES, "n3")}


class ABPlanner(DockingOffsetMixin, TripleSelectionPlanner):
    """The stage-wise selection planner with the single-arm call trace the outcome classifier reads."""

    trace = ()

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


def load_models(folder):
    report = json.loads((folder / "train_v2_report.json").read_text())
    return {name: ranker.Ensemble.load(folder / file, report[stage]["model_sha256"], names) for name, (file, names, stage) in MODELS.items()}


def run_arm(planner, boxes, kind, arm, recorder):
    planner.arm, planner.trace = arm, []
    mark = recorder.mark()
    began = time.perf_counter()
    error = None
    try:
        planner.configure_task(boxes)
        result = planner.full_cycle(boxes)
    except Exception as exception:  # a crash is an outcome of the arm, not a reason to stop the run
        result, error = None, f"{type(exception).__name__}: {exception}"[:500]
        traceback.print_exc()
    wall_ms = (time.perf_counter() - began) * 1000
    label, stage = classify(kind, result, planner.trace)
    entry = dict(arm=arm, class_=label, stage=stage, error=error, wall_ms=wall_ms,
                 searches=[compact_search(call) for call in recorder.since(mark)], trace=planner.trace)
    if result is not None:
        frames = np.asarray(result.get("frames") or [], dtype=np.float64)
        entry.update(total_ms=result.get("total_ms"), timings=timings(result), selection=result.get("selection"),
                     details=str(result.get("blocked_details") or "")[:300] or None,
                     motion_duration_s=result.get("motion_duration_s"), trajectory_timing_ms=result.get("trajectory_timing_ms"),
                     frames_digest=array_digest(frames) if len(frames) else None,
                     selected_contact=result.get("selected_original_contact_index", result.get("selected_candidate")))
        if result.get("success"):
            entry["cost"] = round_costs(planner, boxes, result)
    return entry


def run_instance(planner, item, arms, number):
    boxes = {side: int(box) for side, box in item["task"].items()}
    planner.scene_offset = tuple(float(value) for value in item["offset_m"])
    order = arms if number % 2 == 0 else arms[::-1]
    record = dict(instance=item["instance"], task=boxes, kind=item["kind"], split=item["split"], group=item["group"],
                  offset_m=item["offset_m"], order=order, gpu_before=gpu_state(), load_before=os.getloadavg(), arms={})
    began = time.perf_counter()
    with SearchRecorder(0) as recorder:
        for arm in order:
            entry = run_arm(planner, boxes, item["kind"], arm, recorder)
            record["arms"][arm] = entry
            record["mode"] = planner.mode(boxes)
            print(json.dumps(dict(instance=item["instance"], arm=arm, label=entry["class_"], stage=entry["stage"],
                                  t_path=(entry.get("cost") or {}).get("t_path"), ms=round(entry["wall_ms"]),
                                  fallback=bool((entry.get("selection") or {}).get("fallback"))), ensure_ascii=False), flush=True)
    record.update(wall_s=time.perf_counter() - began, gpu_after=gpu_state(), load_after=os.getloadavg())
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--models", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--groups", default="dev")
    parser.add_argument("--kinds", default="dual,single")
    parser.add_argument("--instances", help="comma separated instance ids (default: all selected)")
    parser.add_argument("--arms", default="A,B0,B")
    parser.add_argument("--max-minutes", type=float, default=25.)
    args = parser.parse_args()
    registry = load_registry(args.registry)
    wanted = set(args.instances.split(",")) if args.instances else None
    items = [item for item in registry["instances"] if item["group"] in args.groups.split(",") and item["kind"] in args.kinds.split(",")
             and (wanted is None or item["instance"] in wanted)]
    folder = args.output / "instances"
    folder.mkdir(parents=True, exist_ok=True)
    pending = [(number, item) for number, item in enumerate(items) if not (folder / f"{item['instance']}.json").exists()]
    print(json.dumps(dict(selected=len(items), pending=len(pending)), ensure_ascii=False), flush=True)
    if not pending:
        raise SystemExit(0)
    planner = ABPlanner(PlannerAssets(), .025)
    planner.warmup_resources()
    models = load_models(args.models)
    planner.setup_selection(args.models / "assets", models)
    with open(args.output / "provenance.jsonl", "a") as handle:
        handle.write(json.dumps(dict(provenance(args), arms=args.arms, theta=planner.theta, m=planner.m, place_attempts=planner.place_attempts,
                                     allocation_ms=planner.allocation_ms, models={name: model.sha256 for name, model in models.items()},
                                     selection_setup_ms=planner.selection_setup_ms), ensure_ascii=False) + "\n")
    with SearchRecorder(0) as recorder:
        planner.scene_offset = (0., 0.)
        for arm in args.arms.split(","):
            run_arm(planner, WARMUP_TASK, "dual", arm, recorder)
    started = time.perf_counter()
    for number, item in pending:
        if time.perf_counter() - started > args.max_minutes * 60:
            raise SystemExit(MORE_WORK)
        record = run_instance(planner, item, args.arms.split(","), number)
        path = folder / f"{item['instance']}.json"
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(record, ensure_ascii=False, default=float) + "\n")
        temporary.rename(path)
    raise SystemExit(0)


if __name__ == "__main__":
    main()
