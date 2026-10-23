#!/usr/bin/env python3
"""MOTION-311 ranker latency and numpy/torch parity (CPU only; budgets from the pre-registration).

For stored S1 candidate sets the in-process time of T0 features, T1 extras and ensemble inference is measured
per call (perf_counter, including Python overhead), repeated over random instances; model load and the
one-off kinematics set-up (analytic bridge, carriage, atlas k-d trees) are timed separately. The numpy
ensemble is compared with a torch re-evaluation of the same weights.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
import v3_candidate_ranker as ranker
from experiments.motion311 import label_analysis as analysis
from experiments.motion311.train_ranker import ATLASES, URDF, context_of

BUDGETS = dict(t0_inference_p95_ms=2.0, t1_inference_p95_ms=10.0, model_load_ms=50.0)


def torch_reference(model, features):
    import torch
    x = torch.tensor(model.standardise(features), dtype=torch.float64)
    logits, costs = [], []
    for layers in model.members:
        hidden = x
        for depth, (weight, bias) in enumerate(layers):
            hidden = hidden @ torch.tensor(weight, dtype=torch.float64) + torch.tensor(bias, dtype=torch.float64)
            if depth < len(layers) - 1:
                hidden = torch.nn.functional.silu(hidden)
        logits.append(hidden[:, 0])
        costs.append(hidden[:, 1])
    return (torch.sigmoid(torch.stack(logits).mean(0)).numpy(), torch.exp(torch.stack(costs).mean(0)).numpy())


def percentiles(values):
    values = np.asarray(values)
    return dict(p50=float(np.percentile(values, 50)), p95=float(np.percentile(values, 95)), max=float(values.max()), calls=len(values))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--labels", type=Path, nargs="+", required=True)
    parser.add_argument("--repeats", type=int, default=1000)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    began = time.perf_counter()
    model = ranker.Ensemble.load(args.model)
    load_ms = (time.perf_counter() - began) * 1000
    tier = model.meta["config"]["tier"]
    records = [record for folder in args.labels for record in analysis.load(folder) if record.get("contacts")]
    home = np.asarray(records[0]["context"]["home"])
    began = time.perf_counter()
    from curobo_core.planner import AnalyticBridge
    kinematics = ranker.Kinematics(URDF, analytic=AnalyticBridge(), carriage=ranker.CarriageModel(ranker.UrdfFk(URDF), home),
                                   atlases={mode: np.load(path, allow_pickle=False)["joints"] for mode, path in ATLASES.items()})
    setup_ms = (time.perf_counter() - began) * 1000
    rng = np.random.default_rng(311)
    t0_ms, t1_ms, inference_ms, total_ms, parity = [], [], [], [], 0.
    for repeat in range(args.repeats):
        record = records[int(rng.integers(len(records)))]
        contacts = np.asarray(record["contacts"], dtype=np.float64)
        context = context_of(record)
        started = time.perf_counter()
        features = ranker.s1_t0(contacts, context, kinematics)
        middle = time.perf_counter()
        eligible = np.ones(len(contacts), dtype=bool)
        if tier == "t1":
            extra, eligible = ranker.s1_t1(contacts, context, kinematics)
            features = np.concatenate([features, extra], axis=1)
        features_done = time.perf_counter()
        probability, cost, max_z = model.predict(features)
        ranker.attempt_order(probability, cost, eligible, .5, max_z)
        finished = time.perf_counter()
        t0_ms.append((middle - started) * 1000)
        t1_ms.append((features_done - middle) * 1000)
        inference_ms.append((finished - features_done) * 1000)
        total_ms.append((finished - started) * 1000)
        if repeat < 20:
            reference = torch_reference(model, features)
            parity = max(parity, float(np.abs(reference[0] - probability).max()), float(np.abs(reference[1] / cost - 1).max()))
    report = dict(model=str(args.model), tier=tier, members=model.meta["members"], config=model.meta["config"], model_load_ms=load_ms,
                  kinematics_setup_ms=setup_ms, t0_features_ms=percentiles(t0_ms), t1_features_ms=percentiles(t1_ms),
                  inference_ms=percentiles(inference_ms), total_ms=percentiles(total_ms), numpy_torch_max_abs_diff=parity,
                  budgets=BUDGETS)
    report["within_budget"] = dict(
        total_p95=report["total_ms"]["p95"] <= (BUDGETS["t1_inference_p95_ms"] if tier == "t1" else BUDGETS["t0_inference_p95_ms"]),
        model_load=load_ms <= BUDGETS["model_load_ms"], parity=parity <= 1e-6)
    text = json.dumps(report, indent=1)
    if args.output:
        args.output.write_text(text + "\n")
    print(text)


if __name__ == "__main__":
    main()
