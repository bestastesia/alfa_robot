"""MOTION-306 M0a diagnostic: box-size feasibility envelope of the current contact pools.

For a grid of box dims and one train instance per mode, the stage-0 pool is checked with the empty checker the
strict environment uses at contact. Colliding candidates are attributed by removing object groups (row/bottom
fillers, task boxes, container walls, everything); "robot" means still colliding with all objects removed.
"""
import argparse
from dataclasses import replace
import gc
import itertools
import json
from pathlib import Path
import sys

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from experiments.motion306.size_smoke import size_items
from motion311_learning.stage_env import EpisodeFailure, StageEnv
from v3_batched_loaded_search import GpuValidity
from v3_box_geometry import DEFAULT_BOX_DIMS
from v3_scene_variants import load_registry

ROOT = Path(__file__).resolve().parents[3]
GROUPS = {"fillers": lambda name: name.startswith("task_fill_"), "task_boxes": lambda name: name.startswith("wall_box_"),
          "container": lambda name: name in ("left_wall", "right_wall", "front_wall", "ceiling", "ground"),
          "all_objects": lambda name: True}


def valid(env, rows):
    return GpuValidity(env.planner.checker(env.boxes), check_ground=True).mask(
        torch.tensor(rows, device="cuda", dtype=torch.float32)).cpu().numpy()


def attribute(env, rows):
    planner, snapshot = env.planner, env.planner.snapshot
    freed = {}
    for group, member in GROUPS.items():
        planner.snapshot = replace(snapshot, objects=tuple(o for o in snapshot.objects if not member(o.object_id)),
                                   revision=snapshot.revision + 1000)
        planner.prepare_task(env.boxes)
        freed[group] = int(valid(env, rows).sum())
    planner.snapshot = snapshot
    planner.prepare_task(env.boxes)
    return freed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--widths", default="0.36,0.37,0.38,0.40")
    parser.add_argument("--heights", default="0.25,0.30,0.35,0.40")
    parser.add_argument("--depths", default="0.30")
    parser.add_argument("--extra", default="0.25x0.40x0.40,0.45x0.40x0.40", help="depth x width x height")
    args = parser.parse_args()
    parse = lambda text: [float(value) for value in text.split(",") if value]
    grid = [(d, w, h) for d, w, h in itertools.product(parse(args.depths), parse(args.widths), parse(args.heights))]
    grid += [tuple(float(v) for v in item.split("x")) for item in args.extra.split(",") if item]
    registry = load_registry(ROOT / "config/motion306_registry.json")
    bases = size_items(registry, [next(level for level, dims in registry["size_levels"].items() if tuple(dims) == DEFAULT_BOX_DIMS)])
    env = StageEnv(timed=True, physical=False)
    args.output.mkdir(parents=True, exist_ok=True)
    rows = []
    for dims in grid:
        for base in bases:
            item = dict(base, box_dims=list(dims), instance=f"{base['base_instance']}-grid-{'x'.join(f'{v:.2f}' for v in dims)}")
            entry = dict(base=base["base_instance"], mode=f"{'dual' if len(base['task']) == 2 else 'single-' + next(iter(base['task']))}-{'top' if base['top_grasp'] else 'side'}",
                         dims=list(dims))
            try:
                candidates = np.asarray(env.reset(item, seed=101).candidates, dtype=np.float32)
                mask = valid(env, candidates) if len(candidates) else np.zeros(0, dtype=bool)
                entry.update(pool=len(candidates), collision_free=int(mask.sum()))
                if (~mask).any():
                    entry["freed_by_removing"] = attribute(env, candidates[~mask])
                    entry["colliding"] = int((~mask).sum())
                    entry["freed_without_ground_plane"] = int(GpuValidity(env.planner.checker(env.boxes), check_ground=False).mask(
                        torch.tensor(candidates[~mask], device="cuda", dtype=torch.float32)).sum())
            except EpisodeFailure as failure:
                entry.update(pool=0, collision_free=0, reset_failure=str(failure))
            rows.append(entry)
            print(json.dumps(entry, ensure_ascii=False), flush=True)
        env.planner.persistent_checkers.clear()
        gc.collect()
        torch.cuda.empty_cache()
        (args.output / "size_envelope.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1) + "\n")


if __name__ == "__main__":
    main()
