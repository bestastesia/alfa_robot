"""Build a paired MOTION-365 B1 ablation report from persisted per-instance JSON; never reruns planning."""
import argparse
import itertools
import json
from pathlib import Path

import numpy as np


def load_arm(spec):
    label, separator, value = spec.partition("=")
    if not separator or not label or not value:
        raise ValueError("arm must be LABEL=INSTANCES_DIR")
    directory = Path(value)
    rows = [json.loads(path.read_text()) for path in sorted(directory.glob("*.json"))]
    if not rows or len({row["instance"] for row in rows}) != len(rows):
        raise ValueError(f"arm {label} is empty or has duplicate instances")
    config = json.loads((directory.parent / "config.json").read_text())
    return label, {row["instance"]: row for row in rows}, config


def percentile(values, value):
    return float(np.percentile(values, value)) if values else None


def summary(rows, config):
    rows = list(rows.values())
    successful = [row for row in rows if row["success"]]
    selection = [row.get("selection_total_ms") for row in rows if row.get("selection_total_ms") is not None]
    risk = [outcome.get("risk_ms") for row in rows for outcome in row.get("outcomes", [])
            if outcome.get("risk_ms") is not None]
    reasons = {}
    for row in rows:
        if not row["success"]:
            reasons[row.get("reason")] = reasons.get(row.get("reason"), 0) + 1
    return dict(n=len(rows), successes=len(successful), success_rate=len(successful) / len(rows),
                statuses={name: sum(row["status"] == name for row in rows) for name in sorted({row["status"] for row in rows})},
                failure_reasons=reasons, selection_ms=dict(median=percentile(selection, 50), p95=percentile(selection, 95),
                                                           maximum=max(selection) if selection else None,
                                                           over_budget=sum(bool(row.get("selection_budget_exceeded")) for row in rows)),
                direct_risk_ms=dict(median=percentile(risk, 50), p95=percentile(risk, 95)),
                successful_costs={name: dict(median=percentile([row[name] for row in successful if row.get(name) is not None], 50))
                                  for name in ("raw_t_path_s", "executed_t_path_s", "motion_duration_s")},
                provenance=dict(source_sha256=config.get("source_sha256"), registry_sha256=config.get("registry_sha256"),
                                checkpoint=config.get("checkpoint")))


def paired(a_label, a, b_label, b):
    if set(a) != set(b):
        raise ValueError(f"instance sets differ for {a_label} and {b_label}")
    identities = sorted(a)
    both = [key for key in identities if a[key]["success"] and b[key]["success"]]
    costs = {}
    for name in ("raw_t_path_s", "executed_t_path_s", "motion_duration_s"):
        delta = [b[key][name] - a[key][name] for key in both if a[key].get(name) is not None and b[key].get(name) is not None]
        costs[name] = dict(n=len(delta), median_b_minus_a=percentile(delta, 50), mean_b_minus_a=float(np.mean(delta)) if delta else None)
    return dict(a_label=a_label, b_label=b_label, n=len(identities),
                a_only_success=sum(a[key]["success"] and not b[key]["success"] for key in identities),
                b_only_success=sum(b[key]["success"] and not a[key]["success"] for key in identities),
                both_success=len(both), neither_success=sum(not a[key]["success"] and not b[key]["success"] for key in identities),
                b_minus_a_success=sum(b[key]["success"] for key in identities) - sum(a[key]["success"] for key in identities),
                both_success_costs=costs)


def build(arms):
    labels = list(arms)
    reference = set(next(iter(arms.values()))[0])
    if any(set(rows) != reference for rows, _ in arms.values()):
        raise ValueError("all arms must contain the same instance identities")
    return dict(schema="motion365-b1-ablation-v1", instances=len(reference),
                summaries={label: summary(*arms[label]) for label in labels},
                paired={a + "__" + b: paired(a, arms[a][0], b, arms[b][0])
                        for a, b in itertools.combinations(labels, 2)},
                interpretation="strict single action per stage; all persisted failures remain in the denominator")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--arm", action="append", required=True, help="LABEL=instances directory")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    loaded = [load_arm(spec) for spec in args.arm]
    arms = {label: (rows, config) for label, rows, config in loaded}
    if len(arms) != len(loaded):
        raise ValueError("duplicate arm label")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(build(arms), ensure_ascii=False, indent=1) + "\n")


if __name__ == "__main__":
    main()
