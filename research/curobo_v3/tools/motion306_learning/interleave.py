"""Interleaved strict evaluation: every route runs on the same instance back to back, the order rotating per
instance, so host-load drift (wall-clock RRT and timing budgets) hits all routes alike. Same records, screening,
denominators and report as evaluate.py; each route spec is label=route[:model]. Launch under the GPU flock.
"""
import argparse
import hashlib
import json
from pathlib import Path
import signal
import time

import torch

from motion311_learning.contracts import config_digest
from motion306_learning.evaluate import (ROUTES, SCHEMA, _atomic_json, _load_policy, _source_fingerprint, build_report,
                                         evaluate_instance, select_items, sha256_file)


def level_blocks(items, block=8):
    """Cycle through box levels in blocks, so a run stopped early still covers every level about evenly
    (blocks keep the per-level checker cache warm on the 8 GB card)."""
    per_level = {}
    for item in items:
        per_level.setdefault(item.get("box_level"), []).append(item)
    queues, ordered = [rows for _, rows in sorted(per_level.items(), key=lambda pair: str(pair[0]))], []
    while any(queues):
        for rows in queues:
            ordered.extend(rows[:block])
            del rows[:block]
    return ordered


def parse_routes(specs):
    routes = []
    for spec in specs:
        label, _, target = spec.partition("=")
        route, _, models = target.partition(":")
        if not label or route not in ROUTES:
            raise ValueError(f"route spec must be label=route[:model], got {spec!r}")
        routes.append(dict(label=label, route=route, models=models or None))
    if len({route["label"] for route in routes}) != len(routes):
        raise ValueError("route labels must be unique")
    return routes


def run(registry, output, specs, group="dev", levels=(), seed=101, stride=1, limit=None, screen=True, place_gate=False,
        contact_pool="capped", max_minutes=60., env_factory=None, stop_requested=None):
    if group == "test":
        raise ValueError("test split stays sealed for MOTION-364")
    registry, output = Path(registry), Path(output)
    items = level_blocks(select_items(json.loads(registry.read_text()), group, set(levels), limit, stride))
    routes = parse_routes(specs)
    configs = {}
    for route in routes:
        route["policy"], checkpoint = _load_policy(route["route"], route["models"])
        configuration = {"schema": SCHEMA, "label": route["label"], "route": route["route"], "screen": bool(screen),
                         "group": group, "levels": sorted(levels), "seed": int(seed), "stride": stride, "limit_per_level": limit,
                         "registry_sha256": sha256_file(registry), "checkpoint": checkpoint, "source_sha256": _source_fingerprint(),
                         "environment": "LookaheadEnv", "contact_pool": contact_pool, "place_gate": bool(place_gate),
                         "interleaved": [r["label"] for r in routes], "timed": True, "no_training": True, "strict_single_action": True}
        run_dir = output / route["label"] / f"{group}-s{seed}"
        config_file = run_dir / "config.json"
        if config_file.exists() and json.loads(config_file.read_text()) != configuration:
            raise ValueError(f"resume configuration differs for {route['label']}; use a new output directory")
        _atomic_json(config_file, configuration)
        configs[route["label"]] = (run_dir, config_digest(configuration), checkpoint)
    if env_factory is None:
        from motion306_learning.env import LookaheadEnv as env_factory
    started, env, level, completed = time.perf_counter(), None, None, True
    try:
        for position, item in enumerate(items):
            for shift in range(len(routes)):
                route = routes[(position + shift) % len(routes)]
                run_dir, config_hash, checkpoint = configs[route["label"]]
                target = run_dir / "instances" / (hashlib.sha256(item["instance"].encode()).hexdigest()[:24] + ".json")
                if target.exists():
                    continue
                if time.perf_counter() - started >= max_minutes * 60 or (stop_requested and stop_requested()):
                    return False
                if env is None:
                    env = env_factory(timed=True, physical=False, contact_pool=contact_pool, place_gate=place_gate)
                if level is not None and item.get("box_level") != level and hasattr(env, "planner"):
                    env.planner.persistent_checkers.clear()
                    torch.cuda.empty_cache()
                level = item.get("box_level")
                row = evaluate_instance(env, item, route["route"], route["policy"], seed, screen)
                row.update(label=route["label"], config_sha256=config_hash, interleave_position=shift,
                           provenance=dict(registry_sha256=sha256_file(registry), source_sha256=_source_fingerprint(),
                                           checkpoint_sha256=None if checkpoint is None else checkpoint.get("sha256")))
                _atomic_json(target, row)
                print(json.dumps({"label": route["label"], "instance": item["instance"], "success": row["success"],
                                  "reason": row.get("reason"), "wall_s": round(row["wall_s"], 1)}, ensure_ascii=False), flush=True)
                if row["status"] == "infrastructure":
                    env = None
    finally:
        records = [json.loads(path.read_text()) for path in sorted(output.glob("*/*/instances/*.json"))]
        _atomic_json(output / "report.json", build_report([row for row in records if row.get("schema") == SCHEMA]))
    return completed


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--registry", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--route", action="append", required=True, help="label=route[:model], repeatable")
    parser.add_argument("--group", choices=("train", "dev"), default="dev")
    parser.add_argument("--levels", default="")
    parser.add_argument("--seed", type=int, default=101)
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--max-instances", type=int)
    parser.add_argument("--place-gate", action="store_true")
    parser.add_argument("--contact-pool", choices=("capped", "gated"), default="capped")
    parser.add_argument("--max-minutes", type=float, default=60.)
    parser.add_argument("--threads", type=int, default=4)
    args = parser.parse_args(argv)
    torch.set_num_threads(args.threads)
    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    levels = [level for level in args.levels.split(",") if level]
    completed = run(args.registry, args.output, args.route, args.group, levels, args.seed, args.stride, args.max_instances,
                    True, args.place_gate, args.contact_pool, args.max_minutes, stop_requested=lambda: stop[0])
    return 0 if completed else 3


if __name__ == "__main__":
    raise SystemExit(main())
