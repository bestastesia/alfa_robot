"""MOTION-364 diagnostic: does the 64-row contact cap hide gate-passing contacts?

For each selected instance, the capped stage-0 pool (as the environment exposes it) and the full analytic grid
(same generator, no cap) are gated with the environment's own contact checks: batched collision first, then the
deterministic extraction / pre-approach gates for every collision-free row.
"""
import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from motion311_learning.evaluate import mode
from motion311_learning.stage_env import EpisodeFailure, StageEnv

ROOT = Path(__file__).resolve().parents[3]


def gate(env, rows):
    rows = np.asarray(rows, dtype=np.float64).reshape(-1, 15)
    if not len(rows):
        return dict(rows=0, collision_free=0, passing=0)
    free = env.empty_valid.mask(torch.tensor(rows, device='cuda', dtype=torch.float32)).cpu().numpy()
    reasons = {}
    passing = 0
    for row in rows[free]:
        try:
            env.contact_checks(row)
            passing += 1
        except EpisodeFailure as failure:
            reason = failure.reason.split(':')[0]
            reasons[reason] = reasons.get(reason, 0) + 1
    return dict(rows=len(rows), collision_free=int(free.sum()), passing=passing, rejected_after_collision=reasons)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--registry', type=Path, default=ROOT / 'config/motion311_registry.json')
    parser.add_argument('--group', default='dev')
    parser.add_argument('--modes', default='single-left-top,single-right-top,single-left-side,single-right-side')
    parser.add_argument('--per-mode', type=int, default=6)
    args = parser.parse_args()
    modes = args.modes.split(',')
    chosen = {}
    for item in sorted(json.loads(args.registry.read_text())['instances'], key=lambda row: row['instance']):
        if item['split'] == args.group and item.get('group') == args.group and mode(item) in modes:
            chosen.setdefault(mode(item), [])
            if len(chosen[mode(item)]) < args.per_mode:
                chosen[mode(item)].append(item)
    env = StageEnv(timed=True, physical=False)
    rows = []
    for item in [item for name in modes for item in chosen.get(name, [])]:
        began = time.perf_counter()
        entry = dict(instance=item['instance'], mode=mode(item))
        try:
            observation = env.reset(item, seed=101)
            capped = observation.candidates
            full, _ = env.decoder.candidate_set(env.targets, env.previous, 100000, 0)
            entry.update(capped=gate(env, capped), full=gate(env, full))
        except EpisodeFailure as failure:
            entry['reset_failure'] = failure.reason
        entry['wall_s'] = time.perf_counter() - began
        rows.append(entry)
        print(json.dumps(entry, ensure_ascii=False), flush=True)
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / 'contact_pool_cap.json').write_text(json.dumps(rows, ensure_ascii=False, indent=1) + '\n')


if __name__ == '__main__':
    main()
