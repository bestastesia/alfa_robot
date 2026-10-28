#!/usr/bin/env python3
"""MOTION-334: planning time per patch element, from the bench of the patch parts (bench.sh).

bench.sh plans the same cycles (v2 independent test split, one seed, joint_rrt) with the frozen model,
then with each part of the full patch alone (carriage spheres, link7 spheres, base cuboids), then with
the full patch, then with the frozen model again. Every run is paired with the two frozen runs of the
same cycle, the reference interpolated linearly between them by the segment's place in that order
(the frozen runs drift); per variant the median paired change of the round planning time divided by
the number of elements the part adds is the cost per element of its group, as patch_editor.py uses it
(negative ones, within the drift, count as 0). The full patch against the sum of the parts shows how far
the costs add up; the conveyor suffix (which builds its IK solver every cycle) is split out.

  python bench_costs.py --frozen DIR DIR --part carriage carriage_sphere 41 DIR --part ... --full DIR --output costs.json
"""
import argparse
import json
from pathlib import Path

import numpy as np

STAGES = {"round_total_ms": lambda result: result.get("total_ms"),
          "contact_candidates_ms": lambda result: result.get("timing_ms", {}).get("contact_candidates_ms"),
          "extraction_and_approach_ms": lambda result: result.get("timing_ms", {}).get("extraction_and_approach_ms"),
          "transport_ms": lambda result: result.get("timing_ms", {}).get("transport_ms"),
          "conveyor_suffix_ms": lambda result: result.get("conveyor_suffix_ms")}


def load(directory):
    """(task, seed, arm) -> (top grasp, stage times) of the successful runs."""
    runs = {}
    for path in sorted((Path(directory) / "full").glob("*.json")):
        result = json.loads(path.read_text())
        if result.get("success"):
            runs[path.stem] = (bool(result.get("top_grasp")), {stage: get(result) for stage, get in STAGES.items()})
    return runs


def paired(variant, frozen, place):
    """Paired changes (ms and %) per stage against the frozen runs of the same cycle at this place in the order (0..1)."""
    rows = {}
    for kind in ("all", "top", "side"):
        out = {}
        for stage in STAGES:
            delta, percent = [], []
            for name, (top, times) in variant.items():
                base = [runs[name][1][stage] for runs in frozen if name in runs and runs[name][1][stage]]
                if len(base) == len(frozen) and times[stage] is not None and kind in ("all", "top" if top else "side"):
                    reference = base[0] + (base[-1] - base[0]) * place
                    delta.append(times[stage] - reference)
                    percent.append((times[stage] / reference - 1) * 100)
            out[stage] = dict(paired=len(delta), median_ms=float(np.median(delta)) if delta else None, mean_ms=float(np.mean(delta)) if delta else None,
                              median_pct=float(np.median(percent)) if percent else None)
        rows[kind] = out
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frozen", nargs=2, required=True, metavar=("DIR_A", "DIR_B"), help="the frozen runs before and after")
    parser.add_argument("--part", nargs=4, action="append", required=True, metavar=("LABEL", "GROUP", "ELEMENTS", "DIR"))
    parser.add_argument("--full", required=True, help="the full patch (all parts)")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    frozen = [load(directory) for directory in args.frozen]
    baseline = [np.mean([runs[name][1]["round_total_ms"] for runs in frozen]) for name in frozen[0] if all(name in runs for runs in frozen)]
    places = len(args.part) + 2  # frozen-a, the parts, full, frozen-b
    report = dict(source="", baseline_round_ms=float(np.median(baseline)), ms_per={}, parts={},
                  drift=paired(frozen[1], frozen[:1], 0.)["all"]["round_total_ms"],
                  full=paired(load(args.full), frozen, (places - 1) / places))
    for place, (label, group, elements, directory) in enumerate(args.part, 1):
        result = paired(load(directory), frozen, place / places)
        report["parts"][label] = dict(group=group, elements=int(elements), **result)
        report["ms_per"][group] = round(max(0., result["all"]["round_total_ms"]["median_ms"]) / int(elements), 2)
    if "link7_sphere" in report["ms_per"]:
        report["ms_per"].setdefault("arm_sphere", report["ms_per"]["link7_sphere"])  # not measured: as a link7 sphere
    parts = sum(part["all"]["round_total_ms"]["median_ms"] for part in report["parts"].values())
    conveyor = sum(part["all"]["conveyor_suffix_ms"]["median_ms"] for part in report["parts"].values())
    report["source"] = (f"GPU 分项基准：v2 独立测试集 joint_rrt {report['full']['all']['round_total_ms']['paired']} 个周期配对中位，"
                        f"参照按两次冻结线性插值（两次冻结相差 {report['drift']['median_pct']:+.1f}%）；"
                        f"增量约 {conveyor / parts * 100:.0f}% 在 conveyor 段（每箱重建 IK 求解器）；"
                        f"全补丁实测 {report['full']['all']['round_total_ms']['median_ms']:+.0f} ms，分项相加 {parts:+.0f} ms；"
                        f"link5/6 球未测，按 link7 球计")
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=1) + "\n")
    print(f"baseline round median {report['baseline_round_ms']:.0f} ms; drift frozen-b vs a: {report['drift']}")
    for label, part in report["parts"].items():
        row = part["all"]["round_total_ms"]
        print(f"{label:9s} +{part['elements']:2d} {part['group']:16s} paired {row['paired']} median {row['median_ms']:+.1f} ms "
              f"({row['median_pct']:+.1f}%), mean {row['mean_ms']:+.1f} ms -> {report['ms_per'][part['group']]:.2f} ms per element | "
              f"top {part['top']['round_total_ms']['median_pct']:+.1f}% side {part['side']['round_total_ms']['median_pct']:+.1f}%")
    row = report["full"]["all"]["round_total_ms"]
    print(f"full      paired {row['paired']} median {row['median_ms']:+.1f} ms ({row['median_pct']:+.1f}%), mean {row['mean_ms']:+.1f} ms; "
          f"sum of parts {parts:+.1f} ms")


if __name__ == "__main__":
    main()
