#!/usr/bin/env python3
"""MOTION-311 v2 training of the stage networks (CPU torch): N3 (place) from D3, N2 (transport) and N1 (contact) from P3.

Each stage uses the v1 trainer (two outputs: success logit, log correction): grouped 5-fold CV over a small grid
(width {32, 64} x depth {1, 2} x weight decay {1e-4, 1e-3}), early stopping inside the training folds, selection by
out-of-fold loss with an Occam tie-break, a final 5-seed ensemble exported as .npz, and convergence evidence
(plateau of the training loss, seed spread, calibration). Offline evaluation restricts every choice to the labelled
options: the realized cost of the network's pick against the baseline rule, B0 (pure T_lb) and the best label.
"""
import argparse
import hashlib
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import v3_triple_ranker as triple
from experiments.motion311 import train_ranker as base

ROOT = Path(__file__).resolve().parents[3]
URDF = ROOT / "generated/v3_analytic_071cb95/model/robot_meter_meshes.urdf"
GRID = [dict(width=width, depth=depth, weight_decay=decay, tier="pair") for width in (32, 64) for depth in (1, 2) for decay in (1e-4, 1e-3)]


def velocities():
    import xml.etree.ElementTree as ET
    joints = {joint.attrib["name"]: joint for joint in ET.parse(URDF).getroot().findall("joint")}
    return np.array([float(joints[name].find("limit").attrib["velocity"]) for name in triple.ranker.ACTIVE_JOINTS])


def mode_context(mode):
    kind, *rest = mode.split("-")
    sides = [rest[0]] if kind == "single" else ["left", "right"]
    return dict(kind=kind, sides=sides, top_grasp=rest[-1] == "top", box_centers={})


def place_return_time(entry):
    times = (entry.get("cost") or {}).get("t_path_by_phase") or {}
    return times.get("conveyor_place", 0.) + times.get("return_home", 0.) if times else None


def n3_rows(d3, home, velocity):
    """N3 samples from D3: features, success, ambiguity mask (none), log(realized / T_lb), group (mode, carry)."""
    rows = []
    for folder in sorted(Path(d3).iterdir()):
        labels = folder / "labels.jsonl"
        if not labels.exists():
            continue
        places = np.load(folder / "place_set.npz")["rows"]
        context = mode_context(folder.name)
        for line in labels.read_text().splitlines():
            try:
                entry = json.loads(line)
            except json.JSONDecodeError:  # the D3 job may still be appending
                continue
            if entry["class_"] in ("exception", "scene_infeasible"):
                continue
            place = places[entry["place_index"]] if entry["place_index"] is not None else entry.get("placed")
            if place is None:
                continue
            features = triple.place_features(entry["carry"], np.asarray(place)[None], home, velocity, context)[0]
            bound = triple.time_bound(entry["carry"], place, velocity) + triple.time_bound(place, home, velocity)
            realized = place_return_time(entry) if entry["class_"] == "success" else None
            rows.append(dict(x=features, y=float(entry["class_"] == "success"), r=math.log(realized / bound) if realized else np.nan,
                             group=f"{folder.name}:{entry['carry_index']}", mode=folder.name, carry=entry["carry_index"],
                             option=entry["place_index"], bound=float(bound), realized=realized))
    return rows


def validation_plateau(curve, window=50):
    """Relative improvement of the best validation loss within the last `window` epochs before the best one (< 0.005 = converged)."""
    checks = np.array([check for _, check in curve])
    best = int(np.argmin(checks))
    if best < window:
        return None
    return float((checks[best - window] - checks[best]) / max(abs(checks[best - window]), 1e-12))


SUFFIX_STAGES = {"反算放置底盘位", "龙头车放置目标IK", "龙头车放置起点", "龙头车放置RRT", "龙头车放置RRT细验", "保持转身姿态回Home"}


def transport_outcome(record, entry, budget_ms=350.):
    """1/0 if the forced transport search was decided (dual: connected with first_verified <= budget), None if not reached."""
    if record["kind"] == "dual":
        calls = [call for call in entry["searches"] if call["module"] == "multiroot"]
        if not calls:
            return None
        verified = calls[0].get("first_verified_solution_ms")
        return float(calls[0]["success"] and verified is not None and verified <= budget_ms)
    flags = [call for call in entry.get("trace", []) if call.get("call") == "single_path"]
    arm = [call for call in entry["searches"] if call["module"] == "single_arm"]
    calls = [call for call, flag in zip(arm, flags) if flag.get("rotation")]
    if calls:
        return float(calls[0]["success"])
    return 0. if entry.get("stage") in ("运输终点", "运输终点圆柱验收") else None


def record_context(record):
    context = record["context"]
    return dict(kind=record["kind"], sides=sorted(record["task"]), top_grasp=context["top_grasp"], box_centers=context["box_centers"],
                contact_poses=context["contact_poses"], home=context["home"])


def cells_key(record):
    return "|".join(sorted(f"{int(box) // 5}:{min(int(box) % 5, 4 - int(box) % 5)}" for box in record["task"].values()))


def n2_rows(records, velocity):
    """N2 samples from the S2 forced-goal runs: (root, carry) features and whether the transport connected."""
    rows = []
    for record in records:
        context = record_context(record)
        for root, pool in record.get("s2_pools", {}).items():
            root_state = np.asarray(pool["root"], dtype=np.float64)
            goals = np.asarray(pool["pool"], dtype=np.float64)
            for entry in (item for item in record["s2"] if item["root"] == int(root)):
                outcome = transport_outcome(record, entry)
                if outcome is None:
                    continue
                features = triple.transport_features(root_state, goals[entry["goal"]][None], velocity, context)[0]
                rows.append(dict(x=features, y=outcome, r=np.nan, group=cells_key(record), instance=record["instance"],
                                 root=int(root), option=entry["goal"], in_baseline=entry["in_baseline"]))
    return rows


def prefix_outcome(record, entry):
    """Extraction + approach success of a forced contact (None when the dual-arm approach was never tried)."""
    if entry["class_"] == "success":
        return 1.
    stage = entry.get("stage")
    if record["kind"] == "dual":
        if stage == "抽离候选验收":
            return 0.
        if stage == "全链路候选验证":
            attempts = [call for call in entry["searches"] if call["module"] == "multiroot" and call["success"]]
            return 0. if attempts else None
        return 1. if stage in SUFFIX_STAGES else None
    if stage == "抽离与到位":
        return 0.
    return 1. if stage in SUFFIX_STAGES | {"运输RRT", "运输终点", "运输终点圆柱验收"} else None


def as_arrays(rows):
    return (np.stack([row["x"] for row in rows]), np.array([row["y"] for row in rows]), np.zeros(len(rows), dtype=bool),
            np.array([row["r"] for row in rows]))


def group_folds(groups, folds=5, seed=311):
    unique = sorted(set(groups))
    order = np.random.default_rng(seed).permutation(len(unique))
    assignment = {unique[index]: fold % folds for fold, index in enumerate(order)}
    return np.array([assignment[group] for group in groups])


def cross_validate(rows, config, folds, seed=0):
    """Out-of-fold predictions (probability, ratio) and the per-fold epochs of one configuration."""
    x, y, mask, r = as_arrays(rows)
    probability, ratio, epochs = np.zeros(len(rows)), np.ones(len(rows)), []
    for fold in sorted(set(folds)):
        train, valid = folds != fold, folds == fold
        inner = group_folds([rows[index]["group"] for index in np.flatnonzero(train)], folds=5, seed=seed + 1) == 0
        index = np.flatnonzero(train)
        fit_index, stop_index = index[~inner], index[inner]
        fitted = base.fit((x[fit_index], y[fit_index], mask[fit_index], r[fit_index]),
                          (x[stop_index], y[stop_index], mask[stop_index], r[stop_index]) if len(stop_index) else None, config, seed)
        probability[valid], ratio[valid] = base.predict(fitted, x[valid])
        epochs.append(fitted["epochs"])
    return probability, ratio, epochs


def scores(rows, probability, ratio):
    y = np.array([row["y"] for row in rows])
    r = np.array([row["r"] for row in rows])
    eps = 1e-6
    loss = float(-np.mean(y * np.log(probability + eps) + (1 - y) * np.log(1 - probability + eps)))
    success = ~np.isnan(r)
    order = np.argsort(probability)
    ranks = np.empty(len(order))
    ranks[order] = np.arange(len(order))
    positives = y == 1
    auc = float((ranks[positives].mean() - (positives.sum() - 1) / 2) / max((~positives).sum(), 1)) if positives.any() and (~positives).any() else None
    return dict(bce=loss, auc=auc, log_ratio_mae=float(np.mean(np.abs(np.log(ratio[success]) - r[success]))) if success.any() else None,
                ece=ece(probability, y))


def ece(probability, y, bins=10):
    edges, error = np.linspace(0, 1, bins + 1), 0.
    for low, high in zip(edges[:-1], edges[1:]):
        inside = (probability >= low) & ((probability < high) if high < 1 else (probability <= high))
        if inside.any():
            error += inside.mean() * abs(probability[inside].mean() - y[inside].mean())
    return float(error)


def restricted_choice(rows, probability, ratio, theta=.5):
    """Per (mode, carry): realized place+return time of the network's pick, the baseline rule, B0 and the best label."""
    groups = {}
    for index, row in enumerate(rows):
        groups.setdefault(row["group"], []).append(index)
    output = []
    for group, indices in groups.items():
        baseline = next((index for index in indices if rows[index]["option"] is None), None)
        options = list(indices)  # the baseline rule's own placement is one of the options (B deviates only on a predicted gain)
        if baseline is None or not options or rows[baseline]["realized"] is None:
            continue
        values = np.array([rows[index]["bound"] * ratio[index] + (1 - probability[index]) * triple.C_FAIL_S for index in options])
        keep = [position for position, index in enumerate(options) if probability[index] >= theta]
        network = options[min(keep, key=lambda position: values[position])] if keep else baseline
        b0 = min((index for index in options if rows[index]["option"] is not None), key=lambda index: rows[index]["bound"])
        def realized(index):  # a failed place IK costs one failed attempt, then the baseline rule
            return rows[index]["realized"] if rows[index]["realized"] is not None else rows[baseline]["realized"] + triple.C_FAIL_S
        feasible = [index for index in indices if rows[index]["realized"] is not None]
        output.append(dict(group=group, mode=rows[baseline]["mode"], baseline=realized(baseline), network=realized(network),
                           b0=realized(b0), best=min(realized(index) for index in feasible) if feasible else None,
                           network_success=rows[network]["realized"] is not None, b0_success=rows[b0]["realized"] is not None,
                           network_is_baseline=network == baseline))
    return output


def summarise_choice(choices):
    def relative(key):
        values = [(item[key] - item["baseline"]) / item["baseline"] for item in choices]
        return dict(median=float(np.median(values)), mean=float(np.mean(values)), q90=float(np.quantile(values, .9))) if values else None
    return dict(groups=len(choices), network_vs_baseline=relative("network"), b0_vs_baseline=relative("b0"), best_vs_baseline=relative("best"),
                network_success=float(np.mean([item["network_success"] for item in choices])) if choices else None,
                b0_success=float(np.mean([item["b0_success"] for item in choices])) if choices else None,
                network_kept_baseline=float(np.mean([item["network_is_baseline"] for item in choices])) if choices else None)


def train_n3(args, home, velocity, output):
    rows = n3_rows(args.d3, home, velocity)
    folds = group_folds([row["group"] for row in rows])
    report = dict(samples=len(rows), groups=len(set(row["group"] for row in rows)), success_rate=float(np.mean([row["y"] for row in rows])), cv=[])
    for config in GRID:
        probability, ratio, epochs = cross_validate(rows, config, folds)
        entry = dict(config=config, epochs=epochs, **scores(rows, probability, ratio))
        entry["choice"] = summarise_choice(restricted_choice(rows, probability, ratio))
        report["cv"].append(entry)
        print(json.dumps(dict(stage="N3", **{key: entry[key] for key in ("config", "bce", "auc", "log_ratio_mae", "ece")},
                              network_vs_baseline=entry["choice"]["network_vs_baseline"]), ensure_ascii=False, default=float), flush=True)
    best = min(entry["bce"] for entry in report["cv"])
    close = [entry for entry in report["cv"] if entry["bce"] <= best * 1.02]
    selected = min(close, key=lambda entry: (entry["config"]["width"] * entry["config"]["depth"], entry["config"]["weight_decay"]))["config"]
    report["selected"] = selected
    spread = []
    for seed in range(5):
        probability, ratio, _ = cross_validate(rows, selected, folds, seed)
        spread.append(scores(rows, probability, ratio)["bce"])
    report["seed_bce"] = spread
    report["seed_spread"] = float((max(spread) - min(spread)) / np.median(spread))
    x, y, mask, r = as_arrays(rows)
    inner = group_folds([row["group"] for row in rows], seed=99) == 0
    members = [base.fit((x[~inner], y[~inner], mask[~inner], r[~inner]), (x[inner], y[inner], mask[inner], r[inner]), selected, seed)
               for seed in range(5)]
    report["members"] = [dict(epochs=member["epochs"], best_epoch=int(np.argmin([check for _, check in member["curve"]])),
                              validation_plateau=validation_plateau(member["curve"])) for member in members]
    report["curves"] = [member["curve"] for member in members]
    report["model_sha256"] = base.export(output / "n3_place.npz", members, triple.PLACE_NAMES, selected, dict(stage="N3", c_fail_s=triple.C_FAIL_S))
    return report


def n1_rows(records, velocity):
    """N1 samples from the S1 forced contacts: contact features and extraction + approach success (censored dropped)."""
    from curobo_core.planner import AnalyticBridge
    home = np.asarray(records[0]["context"]["home"], dtype=np.float64)
    atlases = {mode: np.load(path, allow_pickle=False)["joints"] for mode, path in base.ATLASES.items()}
    kinematics = triple.ranker.Kinematics(URDF, analytic=AnalyticBridge(), carriage=triple.ranker.CarriageModel(triple.ranker.UrdfFk(URDF), home),
                                          atlases=atlases)
    rows = []
    for record in records:
        if not record["s1"] or record["contacts"] is None:
            continue
        features, _, _ = triple.contact_features(np.asarray(record["contacts"]), record_context(record), kinematics, velocity)
        for entry in record["s1"]:
            outcome = prefix_outcome(record, entry)
            if outcome is not None:
                rows.append(dict(x=features[entry["index"]], y=outcome, r=np.nan, group=cells_key(record), instance=record["instance"],
                                 option=entry["index"]))
    return rows


def single_carry_set(atlas, home, side):
    """Single-arm carry candidates: the placement atlas with the idle arm in each of the planner's 5 distinct variants."""
    idle = slice(8, 15) if side == "left" else slice(1, 8)
    variants = [home[idle]]
    for shoulder, elbow in [(-20, 15), (20, -15), (-35, 30), (35, -30)]:
        arm = home[idle].copy()
        arm[1] += math.radians(shoulder)
        arm[3] += math.radians(elbow)
        variants.append(arm)
    blocks = []
    for arm in variants:
        block = atlas.copy()
        block[:, idle] = arm
        blocks.append(block)
    return np.concatenate(blocks)


def downstream_table(carries, places, home, velocity, context, model=None, chunk=64):
    """V3(g) and argmin place index for every carry candidate (model=None: B0, pure T_lb)."""
    values, choice = np.empty(len(carries)), np.empty(len(carries), dtype=int)
    for offset in range(0, len(carries), chunk):
        for row, carry in enumerate(carries[offset:offset + chunk]):
            q, probability = triple.place_values(carry, places, home, velocity, context, model)
            index = int(np.argmin(q))
            values[offset + row], choice[offset + row] = q[index], index
    return values, choice


def train_binary(rows, name, names, output):
    """Grouped CV over the grid on BCE, Occam tie-break, 5-seed ensemble export (for N2 and the N1 prefix head)."""
    folds = group_folds([row["group"] for row in rows])
    report = dict(samples=len(rows), groups=len(set(row["group"] for row in rows)), success_rate=float(np.mean([row["y"] for row in rows])), cv=[])
    for config in GRID:
        probability, ratio, epochs = cross_validate(rows, config, folds)
        entry = dict(config=config, epochs=epochs, **scores(rows, probability, ratio))
        report["cv"].append(entry)
        print(json.dumps(dict(stage=name, **{key: entry[key] for key in ("config", "bce", "auc", "ece")}), ensure_ascii=False, default=float), flush=True)
    best = min(entry["bce"] for entry in report["cv"])
    close = [entry for entry in report["cv"] if entry["bce"] <= best * 1.02]
    selected = min(close, key=lambda entry: (entry["config"]["width"] * entry["config"]["depth"], entry["config"]["weight_decay"]))["config"]
    report["selected"] = selected
    x, y, mask, r = as_arrays(rows)
    inner = group_folds([row["group"] for row in rows], seed=99) == 0
    members = [base.fit((x[~inner], y[~inner], mask[~inner], r[~inner]), (x[inner], y[inner], mask[inner], r[inner]), selected, seed)
               for seed in range(5)]
    report["members"] = [dict(epochs=member["epochs"], best_epoch=int(np.argmin([check for _, check in member["curve"]])),
                              validation_plateau=validation_plateau(member["curve"])) for member in members]
    report["curves"] = [member["curve"] for member in members]
    report["model_sha256"] = base.export(output / f"{name.lower()}.npz", members, names, selected, dict(stage=name))
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--d3", type=Path, required=True)
    parser.add_argument("--primary", type=Path, help="P3 primary label folder (N2, N1)")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--stages", default="n3")
    parser.add_argument("--threads", type=int, default=4)
    args = parser.parse_args()
    torch.set_num_threads(args.threads)
    args.output.mkdir(parents=True, exist_ok=True)
    velocity = velocities()
    home = np.array([-0.3] + list(np.radians([155, -90, 20, 90, -90, -40, 0])) + list(-np.radians([155, -90, 20, 90, -90, -40, 0])))
    report = dict(created_at=time.strftime("%Y-%m-%dT%H:%M:%S%z"), d3=str(args.d3))
    if "n3" in args.stages:
        report["n3"] = train_n3(args, home, velocity, args.output)
    if "n2" in args.stages:
        records = [json.loads(path.read_text()) for path in sorted((args.primary / "instances").glob("*.json"))]
        train_records = [record for record in records if record["split"] == "train"]
        report["n2"] = train_binary(n2_rows(train_records, velocity), "N2", triple.TRANSPORT_NAMES, args.output)
    if "n1" in args.stages:
        records = [json.loads(path.read_text()) for path in sorted((args.primary / "instances").glob("*.json"))]
        train_records = [record for record in records if record["split"] == "train"]
        report["n1"] = train_binary(n1_rows(train_records, velocity), "N1", triple.CONTACT_NAMES, args.output)
    (args.output / "train_v2_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1, default=float) + "\n")
    print(json.dumps({stage: {key: value for key, value in item.items() if key in ("samples", "selected", "seed_spread", "model_sha256")}
                      for stage, item in report.items() if isinstance(item, dict)}, ensure_ascii=False, default=float))


if __name__ == "__main__":
    main()
