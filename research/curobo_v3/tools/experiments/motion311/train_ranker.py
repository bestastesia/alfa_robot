#!/usr/bin/env python3
"""MOTION-311 ranker training (CPU torch): datasets from labels, cell-grouped CV, convergence evidence, export.

Targets per S1 candidate (per S2 goal): y = success within the single-candidate allocation (forced label succeeded
and its transport first_verified_solution_ms <= --budget-ms; within +-15 % of the budget masked as ambiguous) and,
for successes, log J (S1: J_arc of the round; S2: transport + conveyor place + return-home arc + rotation cost).
Censored failures are 'not within budget' for y; that is the operational quantity the policy needs, not a
feasibility claim. Folds: validation = tasks touching the fold's cells, training = tasks touching none of them.
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
import v3_candidate_ranker as ranker
from experiments.motion311 import label_analysis as analysis

ROOT = Path(__file__).resolve().parents[3]
URDF = ROOT / "generated/v3_analytic_071cb95/model/robot_meter_meshes.urdf"
ATLASES = {"side": ROOT / "generated/selected_placement_20261013/all_solutions.npz", "top": ROOT / "generated/placement_analytic_atlas/top.npz"}


def kinematics(tier, home):
    """T0 needs only the URDF limits; T1 adds the analytic bridge, the closed-form carriage and atlas k-d trees."""
    if tier == "t0":
        return ranker.Kinematics(URDF)
    from curobo_core.planner import AnalyticBridge
    atlases = {mode: np.load(path, allow_pickle=False)["joints"] for mode, path in ATLASES.items()}
    return ranker.Kinematics(URDF, analytic=AnalyticBridge(), carriage=ranker.CarriageModel(ranker.UrdfFk(URDF), home), atlases=atlases)


def context_of(record):
    context = record["context"]
    return dict(kind=record["kind"], sides=sorted(record["task"]), top_grasp=context["top_grasp"], box_centers=context["box_centers"],
                contact_poses=context["contact_poses"], home=context["home"])


def cells_of(record):
    return {(int(box) // 5, min(int(box) % 5, 4 - int(box) % 5)) for box in record["task"].values()}


def s1_rows(record, tools, tier, budget_ms):
    """Features, success target, ambiguity mask and log cost of every S1 candidate of one instance."""
    contacts = np.asarray(record["contacts"], dtype=np.float64)
    context = context_of(record)
    features = ranker.s1_t0(contacts, context, tools["kinematics"])
    eligible = np.ones(len(contacts), dtype=bool)
    if tier == "t1":
        extra, eligible = ranker.s1_t1(contacts, context, tools["kinematics"])
        features = np.concatenate([features, extra], axis=1)
    target, ambiguous, cost = np.zeros(len(contacts)), np.zeros(len(contacts), dtype=bool), np.full(len(contacts), np.nan)
    for entry in record["s1"]:
        facts = analysis.candidate(record, entry)
        verified = facts["transport_verified_ms"]
        index = entry["index"]
        if facts["label"] == "success":
            cost[index] = math.log(facts["j_arc"])
            target[index] = float(verified is not None and verified <= budget_ms)
        if verified is not None and abs(verified - budget_ms) <= .15 * budget_ms:
            ambiguous[index] = True
    return features, target, ambiguous, cost, eligible


def s2_rows(record, budget_ms):
    """Features, success target, ambiguity mask and log cost of every labelled S2 goal of one instance."""
    output = []
    context = context_of(record)
    for root, pool in record["s2_pools"].items():
        features = ranker.s2_t0(np.asarray(pool["root"], dtype=np.float64), np.asarray(pool["pool"], dtype=np.float64), context)
        for label in (item for item in record["s2"] if item["root"] == int(root)):
            facts = analysis.candidate(record, label)
            verified = facts["transport_verified_ms"]
            phases = facts["arc_by_phase"] or {}
            j = None if facts["label"] != "success" else (phases.get("transport", 0.) + phases.get("conveyor_place", 0.)
                                                          + phases.get("return_home", 0.) + facts["rotation_cost"])
            output.append((features[label["goal"]], float(facts["label"] == "success" and verified is not None and verified <= budget_ms),
                           verified is not None and abs(verified - budget_ms) <= .15 * budget_ms,
                           math.log(j) if j else np.nan))
    return output


class Network(torch.nn.Module):
    def __init__(self, inputs, width, depth):
        super().__init__()
        layers, size = [], inputs
        for _ in range(depth):
            layers += [torch.nn.Linear(size, width), torch.nn.SiLU()]
            size = width
        layers.append(torch.nn.Linear(size, 2))
        self.body = torch.nn.Sequential(*layers)

    def forward(self, values):
        return self.body(values)


def fit(train, valid, config, seed, epochs=600, patience=80):
    """Full-batch AdamW + cosine; early stopping on validation loss; returns the model and its loss curves."""
    torch.manual_seed(seed)
    x, y, mask, cost = (torch.tensor(part, dtype=torch.float64) for part in train)
    mean, scale = x.mean(0), x.std(0).clamp_min(1e-6)
    model = Network(x.shape[1], config["width"], config["depth"]).double()
    optimiser = torch.optim.AdamW(model.parameters(), lr=3e-3, weight_decay=config["weight_decay"])
    schedule = torch.optim.lr_scheduler.CosineAnnealingLR(optimiser, epochs)
    validation = None if valid is None else tuple(torch.tensor(part, dtype=torch.float64) for part in valid)

    clip = config.get("clip")

    def loss(model, x, y, mask, cost):
        z = (x - mean) / scale
        output = model(z.clamp(-clip, clip) if clip else z)
        keep = ~mask.bool()
        classification = torch.nn.functional.binary_cross_entropy_with_logits(output[keep, 0], y[keep])
        success = ~torch.isnan(cost)
        regression = torch.nn.functional.huber_loss(output[success, 1], cost[success], delta=.1) if success.any() else output.sum() * 0
        return classification + .5 * regression

    curve, best, best_state, waited = [], math.inf, None, 0
    for epoch in range(epochs):
        model.train()
        optimiser.zero_grad()
        value = loss(model, x, y, mask, cost)
        value.backward()
        optimiser.step()
        schedule.step()
        model.eval()
        with torch.no_grad():
            check = float(loss(model, *validation)) if validation is not None else float(value)
        curve.append((float(value.detach()), check))
        if check < best - 1e-6:
            best, best_state, waited = check, {key: item.clone() for key, item in model.state_dict().items()}, 0
        else:
            waited += 1
            if waited >= patience:
                break
    model.load_state_dict(best_state)
    return dict(model=model, mean=mean, scale=scale, curve=curve, epochs=len(curve), clip=clip)


def predict(fitted, features):
    with torch.no_grad():
        z = (torch.tensor(features, dtype=torch.float64) - fitted["mean"]) / fitted["scale"]
        output = fitted["model"](z.clamp(-fitted["clip"], fitted["clip"]) if fitted.get("clip") else z).numpy()
    return 1. / (1. + np.exp(-output[:, 0])), np.exp(output[:, 1])


def stack(rows):
    return tuple(np.asarray([row[index] for row in rows], dtype=np.float64) for index in range(4))


def evaluate_s1(records, fitted, data, budget_ms, m=1, theta=0.):
    """Policy simulation per instance against the hindsight best and the baseline order (A' for dual arm).

    Dual: the network's top-m single-candidate attempts, then the baseline fallback; single: the network's
    top-m followed by the remaining candidates in baseline order, tried until one succeeds (2 s searches).
    An abstaining network (no candidate with P >= theta) behaves like the baseline order.
    """
    output = []
    for record in records:
        if record["baseline"]["class_"] != "success" or not record["s1"]:
            continue
        features, _, _, _, eligible = data[record["instance"]]
        probability, cost = predict(fitted, features) if isinstance(fitted, dict) else fitted(features)
        order = ranker.attempt_order(probability, cost, eligible, theta=theta)
        count = len(record["s1"])
        if record["kind"] == "dual":
            lazy = record["baseline"].get("ranking") or list(range(count))
            policy = analysis.simulate(record, order[:m] if order else lazy, m, budget_ms)
            reference = analysis.simulate(record, lazy, m, budget_ms, analysis.ranking_ms(record))
            best = analysis.oracle_order(record, budget_ms)
            oracle = analysis.simulate(record, best, 1, budget_ms) if best else None
        else:
            top = (order or [])[:m]
            full = top + [index for index in range(count) if index not in top]
            policy = analysis.simulate(record, full, count, analysis.SINGLE_BUDGET_MS)
            reference = analysis.simulate(record, list(range(count)), count, analysis.SINGLE_BUDGET_MS)
            best = analysis.oracle_order(record, analysis.SINGLE_BUDGET_MS)
            oracle = analysis.simulate(record, best, 1, analysis.SINGLE_BUDGET_MS) if best else None
        output.append(dict(instance=record["instance"], subset=analysis.subset(record), policy_j=policy["j_arc"], baseline_order_j=reference["j_arc"],
                           oracle_j=None if oracle is None else oracle["j_arc"], baseline_j=record["baseline"]["cost"]["j_arc"],
                           policy_ms=policy["plan_ms"], baseline_order_ms=reference["plan_ms"], policy_fallback=policy["fallback"],
                           abstained=order is None, first_attempt_success=not policy["fallback"] and policy["attempts"] == 1))
    return output


def summary(rows):
    def median(key, relative_to=None):
        values = [row[key] / row[relative_to] - 1. if relative_to else row[key] for row in rows
                  if row[key] is not None and (relative_to is None or row[relative_to])]
        return float(np.median(values)) if values else None
    return dict(instances=len(rows), abstained=float(np.mean([row["abstained"] for row in rows])) if rows else None,
                regret_to_oracle=median("policy_j", "oracle_j"), j_vs_baseline_order=median("policy_j", "baseline_order_j"),
                j_vs_baseline=median("policy_j", "baseline_j"), ms_vs_baseline_order=median("policy_ms", "baseline_order_ms"),
                first_attempt_success=float(np.mean([row["first_attempt_success"] for row in rows])) if rows else None)


def export(path, fitted_members, names, config, extra):
    arrays = dict(mean=fitted_members[0]["mean"].numpy(), scale=fitted_members[0]["scale"].numpy())
    for member, fitted in enumerate(fitted_members):
        linear = [module for module in fitted["model"].body if isinstance(module, torch.nn.Linear)]
        for depth, module in enumerate(linear):
            arrays[f"m{member}_w{depth}"] = module.weight.detach().numpy().T.copy()
            arrays[f"m{member}_b{depth}"] = module.bias.detach().numpy().copy()
    meta = dict(members=len(fitted_members), features=list(names), feature_hash=ranker.feature_hash(names), config=config, **extra)
    np.savez(path, meta=np.array(json.dumps(meta)), **arrays)
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_records(folders, registry, splits):
    instances = {item["instance"]: item for item in registry["instances"]}
    records = []
    for folder in folders:
        for record in analysis.load(folder):
            item = instances.get(record["instance"])
            if item is not None and item["split"] in splits and record["baseline"]["class_"] not in ("scene_infeasible", "exception"):
                record["cells"] = sorted(cells_of(record))
                records.append(record)
    return records


def fold_cells(registry, folds=5, seed=311):
    used = {tuple(cell) for cell in registry["test_cells"] + registry["dev_cells"]}
    cells = sorted({(row, column) for row in range(5) for column in range(3)} - used)
    order = np.random.default_rng(seed).permutation(len(cells))
    return [sorted(cells[index] for index in order[fold::folds]) for fold in range(folds)]


def inner_split(records, share=5):
    """Early-stopping split inside a training set: tasks with sha1(task) % share == 0 (grouped by task)."""
    def stopping(record):
        return int(hashlib.sha1(json.dumps(record["task"], sort_keys=True).encode()).hexdigest()[:8], 16) % share == 0
    return [record for record in records if not stopping(record)], [record for record in records if stopping(record)]


def train_model(records, data, subsets, config, seed):
    fit_records, stop_records = inner_split(records)
    fit_set, stop_set = stack_s1(fit_records, data, subsets), stack_s1(stop_records, data, subsets)
    if fit_set is None:
        return None
    return fit(fit_set, stop_set, config, seed)


def split_fold(records, held):
    held = set(map(tuple, held))
    valid = [record for record in records if set(map(tuple, record["cells"])) & held]
    train = [record for record in records if not set(map(tuple, record["cells"])) & held]
    return train, valid


def s1_data(records, tools, tier, budget_ms):
    return {record["instance"]: s1_rows(record, tools, tier, budget_ms) for record in records if record["s1"]}


def stack_s1(records, data, subsets=None):
    rows = [data[record["instance"]] for record in records if record["instance"] in data
            and (subsets is None or analysis.subset(record) in subsets)]
    if not rows:
        return None
    return (np.concatenate([row[0] for row in rows]), np.concatenate([row[1] for row in rows]),
            np.concatenate([row[2] for row in rows]), np.concatenate([row[3] for row in rows]))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--labels", type=Path, nargs="+", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--budget-ms", type=float, default=350.)
    parser.add_argument("--subsets", default="dual-side,dual-top,single-side,single-top", help="G1-passing subsets used for training")
    parser.add_argument("--tiers", default="t0,t1")
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--debug-train-splits", default="train", help="splits treated as training data (debug only)")
    args = parser.parse_args()
    torch.set_num_threads(args.threads)
    args.output.mkdir(parents=True, exist_ok=True)
    registry = json.loads(args.registry.read_text())
    subsets = set(args.subsets.split(","))
    train_records = load_records(args.labels, registry, set(args.debug_train_splits.split(",")))
    dev_records = load_records(args.labels, registry, {"dev"})
    home = np.asarray(train_records[0]["context"]["home"])
    report = dict(created_at=time.strftime("%Y-%m-%dT%H:%M:%S%z"), labels=[str(folder) for folder in args.labels], budget_ms=args.budget_ms,
                  subsets=sorted(subsets), train_instances=len(train_records), dev_instances=len(dev_records), cv=[], configs=[])
    folds = fold_cells(registry)
    report["fold_cells"] = folds
    grid = [dict(width=width, depth=depth, weight_decay=decay, tier=tier) for tier in args.tiers.split(",")
            for width in (32, 64) for depth in (1, 2) for decay in (1e-4, 1e-3)]
    cache = {}
    for tier in args.tiers.split(","):
        tools = dict(kinematics=kinematics(tier, home))
        cache[tier] = s1_data(train_records + dev_records, tools, tier, args.budget_ms)
    for config in grid:
        data = cache[config["tier"]]
        out_of_fold, epochs = [], []
        for held in folds:
            train, valid = split_fold(train_records, held)
            fitted = train_model(train, data, subsets, config, seed=0)
            if fitted is None or not valid:
                continue
            epochs.append(fitted["epochs"])
            out_of_fold += evaluate_s1([record for record in valid if analysis.subset(record) in subsets], fitted, data, args.budget_ms)
        row = dict(config=config, folds=len(epochs), epochs=epochs, **summary(out_of_fold))
        report["configs"].append(row)
        print(json.dumps(row, ensure_ascii=False, default=float), flush=True)
    selected = select_config(report["configs"])
    report["selected"] = selected
    data = cache[selected["tier"]]
    report["seed_spread"] = cross_validate_seeds(train_records, data, subsets, selected, folds, args.budget_ms)
    report["learning_curve"] = learning_curve(train_records, data, subsets, selected, folds, args.budget_ms)
    members = [train_model(train_records, data, subsets, selected, seed) for seed in range(5)]
    report["final_members"] = [dict(epochs=member["epochs"], plateau=plateau(member["curve"])) for member in members]
    ensemble = lambda features: ensemble_predict(members, features)
    dev = [record for record in dev_records if analysis.subset(record) in subsets]
    report["dev"] = {f"m{m}": summary(evaluate_s1(dev, ensemble, data, args.budget_ms, m, theta=.5)) for m in (1, 2, 3)}
    report["dev_ece"] = calibration_ensemble(dev, members, data)
    names = ranker.S1_T0_NAMES + (ranker.S1_T1_NAMES if selected["tier"] == "t1" else [])
    report["model_sha256"] = export(args.output / "s1_ranker.npz", members, names, selected,
                                    dict(stage="S1", budget_ms=args.budget_ms, subsets=sorted(subsets), registry_sha256=registry["sha256"]))
    (args.output / "train_report.json").write_text(json.dumps(report, ensure_ascii=False, indent=1, default=float) + "\n")
    print(json.dumps(dict(selected=selected, seed_spread=report["seed_spread"], dev=report["dev"], dev_ece=report["dev_ece"],
                          model_sha256=report["model_sha256"]), ensure_ascii=False, default=float))


def select_config(rows):
    """Lowest median out-of-fold regret; within 2 % of it, the smallest network (width x depth), then t0 before t1."""
    rows = [row for row in rows if row.get("regret_to_oracle") is not None]
    best = min(row["regret_to_oracle"] for row in rows)
    close = [row for row in rows if row["regret_to_oracle"] <= best + .02]
    return min(close, key=lambda row: (row["config"]["width"] * row["config"]["depth"], row["config"]["tier"] != "t0",
                                       row["config"]["weight_decay"], row["regret_to_oracle"]))["config"]


def cross_validate_seeds(records, data, subsets, config, folds, budget_ms, seeds=5):
    values = []
    for seed in range(seeds):
        out_of_fold = []
        for held in folds:
            train, valid = split_fold(records, held)
            fitted = train_model(train, data, subsets, config, seed)
            if fitted is not None and valid:
                out_of_fold += evaluate_s1([record for record in valid if analysis.subset(record) in subsets], fitted, data, budget_ms)
        values.append(summary(out_of_fold)["regret_to_oracle"])
    finite = [value for value in values if value is not None]
    middle = float(np.median(finite)) if finite else None
    return dict(regret_by_seed=values, spread=None if not finite or not middle else (max(finite) - min(finite)) / abs(middle))


def learning_curve(records, data, subsets, config, folds, budget_ms, fractions=(.25, .5, .75, 1.)):
    output = []
    for fraction in fractions:
        out_of_fold = []
        for held in folds:
            train, valid = split_fold(records, held)
            tasks = sorted({json.dumps(record["task"], sort_keys=True) for record in train})
            keep = set(np.random.default_rng(311).permutation(tasks)[:max(1, int(round(fraction * len(tasks))))])
            fitted = train_model([record for record in train if json.dumps(record["task"], sort_keys=True) in keep], data, subsets, config, 0)
            if fitted is not None and valid:
                out_of_fold += evaluate_s1([record for record in valid if analysis.subset(record) in subsets], fitted, data, budget_ms)
        output.append(dict(fraction=fraction, **summary(out_of_fold)))
    return output


def plateau(curve, window=50):
    """Relative change of the training loss over the last `window` epochs (convergence criterion < 0.005)."""
    if len(curve) <= window:
        return None
    first, last = curve[-window - 1][0], curve[-1][0]
    return abs(first - last) / max(abs(first), 1e-12)


def ensemble_predict(members, features):
    outputs = [predict(member, features) for member in members]
    logit = np.mean([np.log(np.clip(item[0], 1e-9, 1 - 1e-9) / (1 - np.clip(item[0], 1e-9, 1 - 1e-9))) for item in outputs], axis=0)
    return 1. / (1. + np.exp(-logit)), np.exp(np.mean([np.log(item[1]) for item in outputs], axis=0))


def calibration_ensemble(records, members, data, bins=10):
    probabilities, targets = [], []
    for record in records:
        features, target, ambiguous, _, _ = data[record["instance"]]
        probability, _ = ensemble_predict(members, features)
        probabilities.append(probability[~ambiguous])
        targets.append(target[~ambiguous])
    if not probabilities:
        return None
    probability, target = np.concatenate(probabilities), np.concatenate(targets)
    edges, error = np.linspace(0, 1, bins + 1), 0.
    for low, high in zip(edges[:-1], edges[1:]):
        inside = (probability >= low) & ((probability < high) if high < 1 else (probability <= high))
        if inside.any():
            error += inside.mean() * abs(probability[inside].mean() - target[inside].mean())
    return float(error)


if __name__ == "__main__":
    main()
