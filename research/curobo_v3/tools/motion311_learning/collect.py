#!/usr/bin/env python3
"""Train-only bounded branching teacher: full real suffixes for several alternatives at each stage.

Offline exploration may branch; deployed policies may not. Every leaf preserves the selected prefix and its
actual complete return. No minimum over failed/unseen continuations is fabricated. Supervised training groups
these leaves by exact state/action and can distinguish the observed-return mean from the optimistic best leaf.
"""
import argparse
import json
import signal
import gzip
import time
from pathlib import Path

import numpy as np

from motion311_learning.contracts import config_digest
from motion311_learning.dataset import transition_record, write_episode
from motion311_learning.stage_env import StageEnv, EpisodeFailure
from motion311_learning.provenance import fingerprint, assert_fingerprint, stream_hash


def select_alternatives(observation, count, rng):
    n = len(observation.candidates)
    if not n:
        return []
    q = observation.candidates
    # This teacher may use known Home and current-stage target costs, not unobserved future results.
    previous = observation.state[7:22]
    home = observation.state[22:37]
    weights = np.array([2.617993877991494] + [1.] * 14)
    costs = np.linalg.norm((q - previous) * weights, axis=1)
    if observation.stage == 2:
        costs += np.linalg.norm((q - home) * weights, axis=1)
    order = np.argsort(costs, kind='stable')
    chosen = list(order[:max(1, count // 4)])
    # Include non-nearest feasible branches; no collision oracle used in candidate selection.
    remainder = np.array([i for i in range(n) if i not in chosen])
    if len(remainder):
        chosen.extend(rng.choice(remainder, min(count - len(chosen), len(remainder)), replace=False).tolist())
    return [int(i) for i in chosen]


def _jsonable(value):
    if isinstance(value, np.ndarray):
        return {'__array__': value.tolist(), 'dtype': str(value.dtype)}
    if isinstance(value, dict):
        return {key: _jsonable(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_jsonable(item) for item in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def _restore_arrays(value):
    if isinstance(value, dict):
        if set(value) == {'__array__', 'dtype'}:
            if value['dtype'] not in ('float32', 'float64', 'int32', 'int64', 'bool'):
                raise ValueError('unsupported checkpoint array dtype')
            return np.asarray(value['__array__'], dtype=value['dtype'])
        return {key: _restore_arrays(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_restore_arrays(item) for item in value]
    return value


def save_node(path, data):
    """Atomic gzip JSON simulator checkpoint; no executable deserialization."""
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    temporary.write_bytes(gzip.compress(json.dumps(_jsonable(data), allow_nan=False).encode()))
    temporary.replace(path)


def load_node(path):
    return _restore_arrays(json.loads(gzip.decompress(path.read_bytes())))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--registry', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--max-minutes', type=float, default=24.)
    parser.add_argument('--branches', type=int, default=4)
    parser.add_argument('--seed', type=int, default=311)
    parser.add_argument('--limit', type=int, default=0)
    parser.add_argument('--repeats', type=int, default=1)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    config = dict(schema=3, branches=args.branches, seed=args.seed, timed=True, physics=True, caps=StageEnv.caps,
                  split='train', source='fresh-three-stage-teacher-v2-weighted-nodes', repeats=args.repeats)
    config_file = args.output / 'config.json'
    if config_file.exists() and json.loads(config_file.read_text()) != json.loads(json.dumps(config)):
        raise RuntimeError('output belongs to a different experiment')
    config_file.write_text(json.dumps(config, indent=1)+'\n')
    items = [r for r in json.loads(args.registry.read_text())['instances'] if r['split'] == 'train' and r['group'] == 'train']
    # Round-robin modes: no early timebox can silently become a side-only training set.
    grouped = {}
    for item in items:
        mode = ('dual' if item['kind'] == 'dual' else 'single-' + next(iter(item['task']))) + ('-top' if item['top_grasp'] else '-side')
        grouped.setdefault(mode, []).append(item)
    ordered = [pool[i] for i in range(max(map(len, grouped.values()))) for _, pool in sorted(grouped.items()) if i < len(pool)]
    if args.limit:
        ordered = ordered[:args.limit]
    env = StageEnv(timed=True, physical=True)
    current_identity = fingerprint(args.registry)
    assert_fingerprint(current_identity, current_identity)
    provenance_path = args.output / 'provenance.json'
    if provenance_path.exists():
        assert_fingerprint(json.loads(provenance_path.read_text()), current_identity)
    else:
        provenance_path.write_text(json.dumps(current_identity, indent=1)+'\n')
    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    began = time.perf_counter()
    total = 0
    for repeat in range(args.repeats):
        for item in ordered:
            marker = args.output / 'done' / f"{item['instance']}-r{repeat}.json"
            if marker.exists():
                continue
            if stop[0] or time.perf_counter() - began > args.max_minutes * 60:
                raise SystemExit(3)
            seed = args.seed + repeat
            rng = np.random.default_rng(int(config_digest(dict(instance=item['instance'], seed=seed))[:8], 16))
            scene_began = time.perf_counter()
            leaves, success, truncated = 0, 0, 0
            try:
                node_dir = args.output / 'nodes' / f"{item['instance']}-r{repeat}"
                node_dir.mkdir(parents=True, exist_ok=True)
                node_identity = dict(environment=current_identity['digest'], item_sha256=config_digest(item), config_sha256=config_digest(config))
                identity_path = node_dir / 'identity.json'
                if identity_path.exists():
                    if json.loads(identity_path.read_text()) != node_identity:
                        raise RuntimeError('node belongs to a changed scene/environment')
                else:
                    identity_path.write_text(json.dumps(node_identity, sort_keys=True)+'\n')
                root_file = node_dir / 'root.json.gz'
                if root_file.exists():
                    obs = env.restore(load_node(root_file))
                else:
                    obs = env.reset(item, seed=seed)
                    save_node(root_file, env.checkpoint())
                if not len(obs.candidates):
                    raise EpisodeFailure('scene_no_contact_candidates')
                def visit(observation, prefix, path):
                    nonlocal leaves, success, truncated, total
                    node_id = '-'.join(map(str, path)) or 'root'
                    complete = node_dir / (node_id + '.done')
                    if complete.exists():
                        return
                    checkpoint = env.checkpoint()
                    node_rng = np.random.default_rng(int(config_digest(dict(instance=item['instance'], seed=seed, path=path))[:8], 16))
                    picks = select_alternatives(observation, args.branches, node_rng)
                    for index in picks:
                        if stop[0] or time.perf_counter() - began > args.max_minutes * 60:
                            raise InterruptedError('chunk time limit; completed leaves preserved')
                        child = path + [index]
                        edge_file = node_dir / ('edge-' + '-'.join(map(str, child)) + '.json.gz')
                        if edge_file.exists():
                            edge = load_node(edge_file)
                            row, after = edge['row'], edge['after']
                        else:
                            obs_now = env.restore(checkpoint)
                            result = env.step(index)
                            row = transition_record(obs_now, index, result, behavior='branching_teacher')
                            row['selection_probability'] = 1. / len(picks)
                            after = None if result.terminated or result.truncated else env.checkpoint()
                            save_node(edge_file, dict(row=row, after=after))
                        transitions = prefix + [row]
                        identity = f"{item['instance']}-r{repeat}-" + '-'.join(map(str, child))
                        if row['terminated'] or row['truncated']:
                            target = args.output / 'episodes' / (identity + '.json')
                            if not target.exists():
                                write_episode(args.output / 'episodes', identity, item, transitions, 'branching_teacher', config,
                                              time.perf_counter() - scene_began)
                            leaves += 1; total += 1
                            success += int(bool(row['info'].get('success')))
                            truncated += int(row['truncated'])
                        else:
                            child_observation = env.restore(after)
                            visit(child_observation, transitions, child)
                    complete.write_text('complete\n')
                visit(obs, [], [])
                scene_files = list((args.output / 'episodes').glob(f"{item['instance']}-r{repeat}-*.json"))
                scene_records = [json.loads(path.read_text()) for path in scene_files]
                summary = dict(instance=item['instance'], repeat=repeat, leaves=len(scene_records),
                               success=sum(bool(entry['transitions'][-1]['info'].get('success')) for entry in scene_records),
                               truncated=sum(bool(entry['transitions'][-1]['truncated']) for entry in scene_records),
                               elapsed_s=time.perf_counter() - scene_began)
            except InterruptedError:
                raise SystemExit(3)
            except EpisodeFailure as error:
                summary = dict(instance=item['instance'], repeat=repeat, leaves=0, success=0, scene_failure=error.reason)
            marker.parent.mkdir(parents=True, exist_ok=True)
            temporary = marker.with_suffix('.tmp'); temporary.write_text(json.dumps(summary)+'\n'); temporary.replace(marker)
            print(json.dumps(summary), flush=True)
    print(json.dumps(dict(done=True, leaves_this_chunk=total, elapsed_s=time.perf_counter() - began)), flush=True)


if __name__ == '__main__':
    main()
