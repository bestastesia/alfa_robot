#!/usr/bin/env python3
"""Train-only environment/decoder integration probe. Branch exploration is an OFFLINE teacher diagnostic, not actor inference."""
import argparse
import json
import signal
import time
from pathlib import Path

import numpy as np

from motion311_learning.stage_env import StageEnv, EpisodeFailure


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--registry', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-minutes', type=float, default=20.)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    items = json.loads(args.registry.read_text())['instances']
    selected = {}
    for item in items:
        if item['split'] != 'train' or item['group'] != 'train':
            continue
        mode = ('dual' if item['kind'] == 'dual' else 'single-' + next(iter(item['task']))) + ('-top' if item['top_grasp'] else '-side')
        selected.setdefault(mode, item)
    env = StageEnv(timed=True, physical=True)
    rng = np.random.default_rng(311)
    started = time.perf_counter()
    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    summary = []
    for mode, item in selected.items():
        done = args.output / (mode + '.json')
        if done.exists():
            summary.append(json.loads(done.read_text())); continue
        rows, success = [], None
        try:
            obs = env.reset(item, seed=0)
            root = env.checkpoint()
            indices = list(range(min(8, len(obs.candidates)))) + rng.permutation(len(obs.candidates)).tolist()[:8]
            for first in dict.fromkeys(indices):
                if stop[0] or time.perf_counter() - started > args.max_minutes * 60:
                    break
                obs = env.restore(root)
                # Grid candidates must decode to their represented branch.
                decoded = env.decoder.decode(obs.actions[first], env.targets, env.previous)
                if decoded is None or not np.allclose(decoded, obs.candidates[first], atol=1e-10):
                    raise AssertionError('candidate decoder roundtrip failed')
                r1 = env.step(first); rows.append(dict(stage=0, index=first, info=r1.info))
                if r1.terminated or r1.truncated:
                    continue
                after_contact = env.checkpoint()
                order = list(range(min(8, len(r1.observation.candidates)))) + rng.permutation(len(r1.observation.candidates)).tolist()[:24]
                for second in dict.fromkeys(order):
                    if stop[0] or time.perf_counter() - started > args.max_minutes * 60:
                        break
                    obs = env.restore(after_contact); r2 = env.step(second); rows.append(dict(stage=1, index=second, info=r2.info))
                    if r2.terminated or r2.truncated:
                        continue
                    after_carry = env.checkpoint()
                    order3 = list(range(min(8, len(r2.observation.candidates)))) + rng.permutation(len(r2.observation.candidates)).tolist()[:16]
                    for third in dict.fromkeys(order3):
                        if stop[0] or time.perf_counter() - started > args.max_minutes * 60:
                            break
                        obs = env.restore(after_carry); r3 = env.step(third); rows.append(dict(stage=2, index=third, info=r3.info))
                        if r3.info.get('success'):
                            success = dict(choices=[first, second, third], cost_s=env.cost_s,
                                           motion_duration_s=env.report.get('motion_duration_s'), timed_validation=env.report.get('timed_validation'))
                            break
                    if success is not None: break
                if success is not None: break
        except EpisodeFailure as error:
            rows.append(dict(stage=-1, reason=error.reason))
        result = dict(mode=mode, instance=item['instance'], success=success, attempts=rows)
        if stop[0] or time.perf_counter() - started > args.max_minutes * 60:
            interrupted = args.output / (mode + '-partial.json')
            interrupted.write_text(json.dumps(result, ensure_ascii=False)+'\n')
            raise SystemExit(3)
        temporary = done.with_suffix('.tmp'); temporary.write_text(json.dumps(result, ensure_ascii=False)+'\n'); temporary.rename(done)
        summary.append(result)
        print(json.dumps(dict(mode=mode, success=success, attempts=len(rows))), flush=True)
    (args.output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=1)+'\n')


if __name__ == '__main__':
    main()
