"""MOTION-306 M0a diagnostic: how many current contact candidates are collision-free per box size, and what collides.

For each size level and mode (train, zero offset) the stage-0 pool is checked with the empty checker that the
strict environment uses at contact; colliding candidates are attributed to world objects by re-checking with
one object at a time removed (exclusion attribution, not a distance query).
"""
import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from experiments.motion306.size_smoke import MODES, size_items
from motion311_learning.stage_env import StageEnv
from v3_batched_loaded_search import GpuValidity
from v3_scene_variants import load_registry

ROOT = Path(__file__).resolve().parents[3]


def valid(env, rows):
    return GpuValidity(env.planner.checker(env.boxes), check_ground=True).mask(
        torch.tensor(rows, device="cuda", dtype=torch.float32)).cpu().numpy()


def attribute(env, rows):
    """Objects whose removal makes the colliding rows valid (one at a time); robot-only if none does."""
    planner, snapshot = env.planner, env.planner.snapshot
    names = [item.object_id for item in snapshot.objects if not item.object_id.startswith("conveyor")]
    blame = {}
    for name in names:
        planner.snapshot = replace(snapshot, objects=tuple(o for o in snapshot.objects if o.object_id != name),
                                   revision=snapshot.revision + 1000)
        planner.prepare_task(env.boxes)
        freed = valid(env, rows)
        if freed.any():
            blame[name] = int(freed.sum())
    planner.snapshot = snapshot
    planner.prepare_task(env.boxes)
    return blame


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--levels", default="")
    args = parser.parse_args()
    registry = load_registry(ROOT / "config/motion306_registry.json")
    levels = args.levels.split(",") if args.levels else list(registry["size_levels"])
    env = StageEnv(timed=True, physical=False)
    rows = []
    for item in size_items(registry, levels):
        observation = env.reset(item, seed=101)
        candidates = np.asarray(observation.candidates, dtype=np.float32)
        mask = valid(env, candidates)
        entry = dict(instance=item["instance"], level=item["box_level"], dims=item["box_dims"], pool=len(candidates),
                     collision_free=int(mask.sum()))
        if (~mask).any():
            entry["blame_of_colliding"] = attribute(env, candidates[~mask])
        rows.append(entry)
        print(json.dumps(entry, ensure_ascii=False), flush=True)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "contact_pool.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1) + "\n")


if __name__ == "__main__":
    main()
