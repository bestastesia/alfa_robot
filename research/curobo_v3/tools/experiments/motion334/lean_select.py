#!/usr/bin/env python3
"""MOTION-334: a lean version of the full patch, picked on the patch editor's frames (patch_editor.py).

Most elements of the full patch never reject an audited frame. This drops elements greedily (the most
costly first, then those that detect least) while every contact the full patch detects on the cached
frames stays detected, by at least --redundancy elements (or element pairs) where the full patch has
that many. Leave-one-task-out cross-validation estimates how well a pick carries over to tasks it did
not see: per task, pick on the other tasks' contacts, then count the held-out task's detections the
pick keeps. The pick on all tasks is written as an editor state (--state, opened by patch_editor.py
with the same option) and exported as the editor exports (patch.json, cuboids.json, robot configs).

  python lean_select.py <patch_editor.py data options> --hits EDITOR_HITS.npz --redundancy 1 2 \
      --use 2 --state lean-state.json --export DIR --report report.json
"""
import argparse
import collections
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from experiments.motion334.patch_editor import CLASSES, GROUPS, PHASES, add_data_arguments, export_selection, load_engine


def pick(engine, selection, goal, redundancy, cost, fixed=(), keep=frozenset()):
    """Drops elements of the selection (not of the fixed cost groups, not those in keep) while every goal item keeps
    min(redundancy, its cover in the selection) detectors."""
    def cover(chosen):
        out = collections.Counter()
        for detector in engine.active(chosen):
            out.update(detector.detected & goal)
        return out

    need = {item: min(redundancy, count) for item, count in cover(selection).items()}
    current = set(selection)
    for element_id in sorted(selection, key=lambda item: (-cost(engine.elements[item]), len(engine.static[item].detected), item)):
        if engine.elements[element_id].cost_group in fixed or element_id in keep:
            continue
        trial = frozenset(current - {element_id})
        found = cover(trial)
        if all(found[item] >= count for item, count in need.items()):
            current = set(trial)
    return frozenset(current)


def label(item):
    key, group, threshold, _ = item
    return f"{CLASSES[key]}（{GROUPS[group]}）>{threshold:g} mm"


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_data_arguments(parser)
    parser.add_argument("--redundancy", type=int, nargs="+", default=[1, 2], help="rules to cross-validate")
    parser.add_argument("--use", type=int, required=True, help="the rule of the pick on all tasks")
    parser.add_argument("--fixed", nargs="*", default=[], help="element groups kept whole (e.g. base_cuboid: no measurable cost)")
    parser.add_argument("--keep-state", type=Path, help="an earlier pick (editor state) whose kept elements stay: add to it only")
    parser.add_argument("--state", type=Path, required=True, help="the pick as an editor state")
    parser.add_argument("--export", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    frames, model, engine = load_engine(args)
    costs = json.loads(args.costs.read_text())
    cost = lambda element: costs["ms_per"][element.cost_group]
    everything = engine.kept()
    keep = frozenset(item["id"] for item in json.loads(args.keep_state.read_text())["elements"] if item["kept"]) if args.keep_state else frozenset()
    full = engine.detected(everything)
    task = lambda rp: frames.meta[rp // len(PHASES)]["run"].split("-")[0]
    tasks = sorted({task(item[3]) for item in full})
    report = dict(fixed=args.fixed, keep_state=str(args.keep_state), full=dict(elements=len(everything), cost_ms=round(sum(cost(engine.elements[i]) for i in everything), 1),
                            clean=len(engine.summary(everything)["clean"])), cross_validation={}, tasks=tasks)
    for redundancy in args.redundancy:
        held, kept, sizes, lost = collections.Counter(), collections.Counter(), [], collections.Counter()
        for name in tasks:
            chosen = pick(engine, everything, {item for item in full if task(item[3]) != name}, redundancy, cost, args.fixed, keep)
            sizes.append(len(chosen))
            found = engine.detected(chosen)
            for item in full:
                if task(item[3]) == name:
                    held[label(item)] += 1
                    kept[label(item)] += item in found
                    if item not in found:
                        lost[f"{name} {label(item)}"] += 1
        report["cross_validation"][redundancy] = dict(
            held_out={key: f"{kept[key]}/{count}" for key, count in sorted(held.items())},
            total=f"{sum(kept.values())}/{sum(held.values())}", elements_per_fold=sizes, lost=dict(lost))
        print(f"redundancy {redundancy}: held-out detections kept {sum(kept.values())}/{sum(held.values())}, "
              f"elements per fold {min(sizes)}-{max(sizes)}", flush=True)
        for key, count in sorted(held.items()):
            print(f"   {key:48s} {kept[key]}/{count}")
    for redundancy in sorted(set(args.redundancy) | {args.use}):
        chosen = pick(engine, everything, full, redundancy, cost, args.fixed, keep)
        groups = collections.Counter(engine.elements[i].cost_group for i in chosen)
        report.setdefault("picks", {})[redundancy] = dict(
            elements=sorted(chosen), groups=dict(groups), cost_ms=round(sum(cost(engine.elements[i]) for i in chosen), 1),
            clean=len(engine.summary(chosen)["clean"]), detections_kept=f"{len(engine.detected(chosen) & full)}/{len(full)}")
        print(f"pick on all tasks, redundancy {redundancy}: {len(chosen)} elements {dict(groups)}, "
              f"≈{report['picks'][redundancy]['cost_ms']} ms per box, mesh-clean rejected phases {report['picks'][redundancy]['clean']}")
        if redundancy == args.use:
            for element in engine.elements.values():
                element.kept = element.id in chosen
            args.state.parent.mkdir(parents=True, exist_ok=True)
            args.state.write_text(json.dumps(dict(elements=engine.state()), ensure_ascii=False) + "\n")
            export_selection(engine, model, args, costs, args.export)
            for element in engine.elements.values():
                element.kept = True
    args.report.write_text(json.dumps(report, ensure_ascii=False, indent=1) + "\n")


if __name__ == "__main__":
    main()
