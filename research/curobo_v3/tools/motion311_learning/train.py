#!/usr/bin/env python3
"""Online CPU learning with a crash-consistent per-episode update journal.

Real environment calls still require the outer GPU flock. An episode record is
atomically durable BEFORE updates. agent.pt atomically commits both learned state
and a last_committed_episode watermark. On restart, durable uncommitted episodes
replay updates from that exact checkpoint, never rerun their environments.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import time

import numpy as np
import torch

from motion311_learning.contracts import config_digest
from motion311_learning.curriculum import FrozenCurriculum, VERSION as CURRICULUM_VERSION, sample_task, stage_for_episode
from motion311_learning.dataset import (finish_episode, learning_transition, load_episodes,
                                        normalized_actions, normalized_state, transition_record)
from motion311_learning.features import STATE_DIM
from motion311_learning.networks import DQNAgent, SACAgent


RESUME_VERSION = 1
BC_TARGET_VERSION = 'single-observed-action-weighted-success-suffix-v1'


def mode(item):
    return ('dual' if item['kind'] == 'dual' else 'single-' + next(iter(item['task']))) + ('-top' if item['top_grasp'] else '-side')


def balanced_items(items):
    groups = {}
    for item in sorted(items, key=lambda row: row['instance']):
        groups.setdefault(mode(item), []).append(item)
    if not groups:
        raise ValueError('registry has no group=train, split=train instances')
    return [rows[i] for i in range(max(map(len, groups.values()))) for _, rows in sorted(groups.items()) if i < len(rows)]


def file_sha256(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _sync_directory(directory):
    descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('w') as handle:
        json.dump(data, handle, ensure_ascii=False, allow_nan=False, sort_keys=True)
        handle.write('\n')
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)
    _sync_directory(path.parent)


def warm_data_snapshot(folder):
    """Freeze bytes and deterministic ordering; refuse any group/split contamination."""
    if folder is None:
        return [], {'path': None, 'sha256': config_digest([]), 'files': 0}
    folder = Path(folder)
    source = folder / 'episodes' if (folder / 'episodes').is_dir() else folder
    if not source.is_dir():
        raise ValueError('warm-data directory does not exist')
    manifest = []
    for path in sorted(source.glob('*.json')):
        data = json.loads(path.read_text())
        if 'transitions' not in data:
            continue
        if data.get('split') != 'train' or data.get('group') != 'train':
            raise ValueError('warm-data must contain only explicit group=train, split=train episodes')
        manifest.append([path.name, file_sha256(path)])
    episodes = list(load_episodes(source, 'train'))
    for episode in episodes:
        if episode.get('group') != 'train' or episode.get('split') != 'train':
            raise ValueError('warm episode has no explicit train provenance')
    return episodes, {'path': str(source.resolve()), 'sha256': config_digest(manifest), 'files': len(manifest)}


def _transition_key(row):
    # Include next candidate pool: identical endpoint does not imply identical
    # candidate-conditioned backup when a different pool was observed.
    digest = hashlib.sha256()
    digest.update(str((row['stage'], float(row['reward']), bool(row['terminated']))).encode())
    for field in ('state', 'action', 'next_state', 'next_candidates'):
        value = row[field]
        if value is None:
            digest.update(b'none')
        else:
            array = np.ascontiguousarray(value, dtype='<f4')
            digest.update(str(array.shape).encode())
            digest.update(array.tobytes())
    return digest.digest()


def warm_replay(episodes, route):
    replay, seen = [], set()
    for episode in episodes:
        if episode.get('split') != 'train' or episode.get('group') != 'train':
            raise ValueError('warm replay accepts only explicit train group')
        for row in episode['transitions']:
            if row.get('truncated') or not row.get('trainable', False):
                continue
            converted = learning_transition(row, continuous=route == 'r4')
            key = _transition_key(converted)
            if key not in seen:
                seen.add(key)
                replay.append(converted)
    return replay


def select_bc_targets(episodes):
    """One observed successful action per exact (stage,state), never an action mean.

    Rank each action by its suffix_weight-weighted mean return among successful
    continuations only. This is a BC initialization target, not a success-rate or
    globally optimal-Q label. Equal scores choose the lexicographically smallest
    original parameter tuple, independent of input file/leaf ordering.
    """
    groups = {}
    for episode in episodes:
        if episode.get('split') != 'train' or episode.get('group') != 'train':
            raise ValueError('BC targets accept only explicit group=train, split=train')
        transitions = episode['transitions']
        if (not transitions or not transitions[-1].get('terminated')
                or not transitions[-1].get('info', {}).get('success')
                or any(row.get('truncated') for row in transitions)):
            continue
        for row in transitions:
            if not row.get('episode_success') or not row.get('trainable', False):
                continue
            stage = row['stage']
            state = np.asarray(row['state'], dtype=np.float64)
            action = np.asarray(row['action_params'], dtype=np.float64)
            value, weight = float(row['return']), float(row.get('suffix_weight', 1.0))
            if (stage not in (0, 1, 2) or state.shape != (STATE_DIM,) or action.shape != (3,)
                    or not np.isfinite(state).all() or not np.isfinite(action).all() or np.any(np.abs(action) > 1)
                    or not math.isfinite(value) or not math.isfinite(weight) or weight <= 0):
                raise ValueError('invalid successful BC state/action/return/weight')
            key = (int(stage), tuple(float(value) for value in state))
            parameters = tuple(float(value) for value in action)
            actions = groups.setdefault(key, {})
            actions.setdefault(parameters, []).append((weight, value))
    selected = []
    for (stage, state), actions in sorted(groups.items()):
        scores = {}
        for parameters, observations in sorted(actions.items()):
            # Stable summation makes leaf ordering irrelevant even for near ties.
            observations = sorted(observations)
            mass = math.fsum(weight for weight, _ in observations)
            score = math.fsum(weight * value for weight, value in observations) / mass
            scores[parameters] = (score, mass, len(observations))
        parameters = min(scores, key=lambda action: (-scores[action][0], action))
        score, mass, count = scores[parameters]
        selected.append(dict(stage=stage, state=list(state), action_params=list(parameters),
                             bc_weighted_return=score, bc_effective_weight=mass, bc_success_rows=count,
                             bc_action_alternatives=len(actions), bc_target_version=BC_TARGET_VERSION))
    return selected


def warm_start(agent, episodes, route, steps=200):
    """Deduplicated warm replay; successful-action BC excludes inactive psi."""
    replay = warm_replay(episodes, route)
    if not replay:
        return replay
    rng = np.random.default_rng(agent.config['seed'])
    for _ in range(steps):
        ids = rng.choice(len(replay), min(64, len(replay)), replace=False)
        agent.update([replay[index] for index in ids])
    if route == 'r4':
        successful = select_bc_targets(episodes)
        for stage in range(3):
            rows = [row for row in successful if row['stage'] == stage]
            if not rows:
                continue
            states = torch.tensor(np.stack([normalized_state(row['state']) for row in rows]), dtype=torch.float32, device='cpu')
            targets = torch.tensor(np.stack([row['action_params'] for row in rows]), dtype=torch.float32, device='cpu')
            optimizer = agent.actor_optimizers[stage]
            for _ in range(200):
                ids = rng.choice(len(rows), min(128, len(rows)), replace=False)
                mask = agent.action_mask(states[ids])
                if mask is None:
                    raise ValueError('SAC warm start requires the explicit activity-mask contract')
                prediction = agent.actors[stage].deterministic(states[ids], action_mask=mask)
                delta = prediction - targets[ids]
                angle_cost = ((1. - torch.cos(torch.pi * delta[:, 1:])) * mask[:, 1:]).sum(1) / mask[:, 1:].sum(1)
                loss = (delta[:, 0].square() + angle_cost).mean()
                optimizer.zero_grad(set_to_none=True)
                loss.backward()
                optimizer.step()
    return replay


def policy_rng_state(agent):
    if isinstance(agent, SACAgent):
        return {'kind': 'torch-cpu', 'state': agent.generator.get_state().tolist()}
    return {'kind': 'numpy', 'state': copy.deepcopy(agent.rng.bit_generator.state)}


def restore_policy_rng(agent, state):
    if isinstance(agent, SACAgent):
        if state['kind'] != 'torch-cpu':
            raise ValueError('wrong policy RNG kind')
        agent.generator.set_state(torch.tensor(state['state'], dtype=torch.uint8, device='cpu'))
    else:
        if state['kind'] != 'numpy':
            raise ValueError('wrong policy RNG kind')
        agent.rng.bit_generator.state = copy.deepcopy(state['state'])


def episode_identity(route, seed, episode_index):
    return f'{route}-s{seed}-e{episode_index:06d}'


def _record_path(output, config, index, scene=False):
    return Path(output) / ('scene-failures' if scene else 'episodes') / (episode_identity(config['route'], config['seed'], index) + '.json')


def _read_record(output, config, index):
    paths = [_record_path(output, config, index), _record_path(output, config, index, True)]
    existing = [path for path in paths if path.exists()]
    if len(existing) > 1:
        raise ValueError('episode has both scene-failure and transition records')
    if not existing:
        return None, None
    path = existing[0]
    record = json.loads(path.read_text())
    if (record.get('episode_index') != index or record.get('identity') != episode_identity(config['route'], config['seed'], index)
            or record.get('config_sha256') != config_digest(config)
            or record.get('group') != 'train' or record.get('split') != 'train'):
        raise ValueError('episode provenance does not match training configuration')
    if record.get('resume', {}).get('version') != RESUME_VERSION:
        raise ValueError('episode lacks resumable update state; refusing to guess')
    return record, path


def _learnable_rows(record, route):
    if record.get('kind') == 'scene_failure':
        return []
    rows = record['transitions']
    if not rows or not rows[-1].get('terminated') and not rows[-1].get('truncated'):
        raise ValueError('pending episode is incomplete')
    if any(row.get('truncated') or not row.get('trainable', False) for row in rows):
        return []
    return [learning_transition(row, continuous=route == 'r4') for row in rows]


def write_pending(output, config, index, item, transitions, agent, update_rng, elapsed_s,
                  scene_reason=None, curriculum=None):
    if item.get('group') != 'train' or item.get('split') != 'train':
        raise ValueError('online samples must come from group=train, split=train')
    for scene in (False, True):
        if _record_path(output, config, index, scene).exists():
            raise FileExistsError('pending record already exists; recover it instead of rerunning')
    record = dict(schema=config['schema'], identity=episode_identity(config['route'], config['seed'], index),
                  episode_index=index, instance=item['instance'], split='train', group='train', task=item['task'],
                  offset_m=item['offset_m'], policy=config['route'], config_sha256=config_digest(config),
                  elapsed_s=float(elapsed_s), kind='scene_failure' if scene_reason is not None else 'episode',
                  resume=dict(version=RESUME_VERSION, policy_rng_after_actions=policy_rng_state(agent),
                              update_rng=copy.deepcopy(update_rng.bit_generator.state)))
    if curriculum is not None:
        record['curriculum'] = copy.deepcopy(curriculum)
        record['curriculum_start_stage'] = int(curriculum['curriculum_start_stage'])
        if transitions and transitions[0]['stage'] != record['curriculum_start_stage']:
            raise ValueError('pending transitions include an old teacher prefix or wrong start stage')
    if scene_reason is None:
        record['transitions'] = finish_episode(transitions)
    else:
        record['reason'] = str(scene_reason)
        record['transitions'] = []
    path = _record_path(output, config, index, scene_reason is not None)
    atomic_json(path, record)
    return record, path


def save_agent(agent, path, training_state=None):
    """Extra envelope fields remain compatible with DQNAgent/SACAgent.load."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    content = {'version': 1, 'agent': type(agent).__name__, 'config': agent.config, 'state_dict': agent.state_dict()}
    if training_state is not None:
        state = copy.deepcopy(training_state)
        state['checkpoint_rng'] = {'policy': policy_rng_state(agent), 'torch_cpu': torch.random.get_rng_state()}
        content['training_state'] = state
    temporary = path.with_suffix(path.suffix + '.tmp')
    with temporary.open('wb') as handle:
        torch.save(content, handle)
        handle.flush()
        os.fsync(handle.fileno())
    temporary.replace(path)
    _sync_directory(path.parent)


def initial_training_state(config):
    return {'version': RESUME_VERSION, 'last_committed_episode': -1,
            'configuration_sha256': config_digest(config), 'committed_records': [],
            'gradient_updates': 0}


def load_training_checkpoint(path, config):
    content = torch.load(path, map_location='cpu', weights_only=True)
    state = content.get('training_state')
    if not state or state.get('version') != RESUME_VERSION:
        raise ValueError('checkpoint lacks committed-episode metadata; refusing unsafe legacy resume')
    if state.get('configuration_sha256') != config_digest(config):
        raise ValueError('checkpoint training configuration differs')
    klass = SACAgent if config['route'] == 'r4' else DQNAgent
    if content.get('agent') != klass.__name__ or content.get('version') != 1:
        raise ValueError('checkpoint agent class/version mismatch')
    agent = klass(**content['config'])
    agent.load_state_dict(content['state_dict'])
    restore_policy_rng(agent, state['checkpoint_rng']['policy'])
    torch.random.set_rng_state(state['checkpoint_rng']['torch_cpu'])
    if config['route'] == 'r4' and agent.config.get('action_mask_state_indices') != [5, 6]:
        raise ValueError('SAC checkpoint does not use the required activity mask')
    if len(state['committed_records']) != state['last_committed_episode'] + 1:
        raise ValueError('checkpoint record manifest/watermark is inconsistent')
    return agent, state


def rebuild_replay(output, config, warm, training_state):
    replay = warm_replay(warm, config['route'])
    for index in range(training_state['last_committed_episode'] + 1):
        record, path = _read_record(output, config, index)
        if path is None or file_sha256(path) != training_state['committed_records'][index]:
            raise ValueError('committed record is missing or modified')
        replay.extend(_learnable_rows(record, config['route']))
    return replay


def commit_pending(agent, replay, training_state, record, record_path, checkpoint, config, after_update=None):
    index = record['episode_index']
    if index != training_state['last_committed_episode'] + 1:
        raise ValueError('pending episodes must commit in contiguous order')
    if record['config_sha256'] != config_digest(config):
        raise ValueError('pending configuration changed')
    # Roll forward stochastic action consumption without rerunning the environment.
    restore_policy_rng(agent, record['resume']['policy_rng_after_actions'])
    rng = np.random.default_rng()
    rng.bit_generator.state = copy.deepcopy(record['resume']['update_rng'])
    rows = _learnable_rows(record, config['route'])
    replay.extend(rows)
    update_count = 0
    for _ in range(config['updates'] * len(rows)):
        if len(replay) >= 32:
            ids = rng.choice(len(replay), min(64, len(replay)), replace=False)
            agent.update([replay[index] for index in ids])
            update_count += 1
            if after_update is not None:
                after_update(update_count)
    committed = copy.deepcopy(training_state)
    committed['last_committed_episode'] = index
    committed['committed_records'].append(file_sha256(record_path))
    committed['gradient_updates'] += update_count
    committed['last_update_rng'] = copy.deepcopy(rng.bit_generator.state)
    save_agent(agent, checkpoint, committed)
    return committed


def curriculum_start(env, library, items, index, episodes, seed, continuous, rng):
    """Restore checked history before exposing state; only new suffixes train."""
    planner_seed = int(seed) * 1000003 + int(index)
    start_stage = stage_for_episode(index, episodes) if library is not None else 0
    selection_rng_before = copy.deepcopy(rng.bit_generator.state)
    if start_stage:
        state, item, metadata = library.sample(start_stage, rng, planner_seed, continuous)
    else:
        state = None
        item = (sample_task(items, rng) if library is not None
                else copy.deepcopy(items[int(rng.integers(len(items)))]))
        metadata = dict(version=CURRICULUM_VERSION if library is not None else 'home-only-v1',
                        library_sha256=None if library is None else library.manifest['sha256'],
                        curriculum_start_stage=0, source_path=None, source_sha256=None, source_seed=None,
                        planner_seed=planner_seed, task=copy.deepcopy(item['task']), instance=item['instance'],
                        offset_m=copy.deepcopy(item['offset_m']))
    metadata['selection_rng_before'] = selection_rng_before
    metadata['selection_rng_after'] = copy.deepcopy(rng.bit_generator.state)
    # Expose selected task/provenance even when a reset raises a scene failure.
    env.curriculum_selection = (copy.deepcopy(item), copy.deepcopy(metadata))
    # seed/continuous were overwritten inside sample, BEFORE restore/observe.
    observation = (env.restore(state) if state is not None
                   else env.reset(item, seed=planner_seed, continuous=continuous))
    if observation.stage != start_stage:
        raise ValueError('curriculum restore/reset returned the wrong stage')
    return observation, item, metadata


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--registry', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--route', choices=('r3', 'r4'), required=True)
    parser.add_argument('--warm-data', type=Path)
    parser.add_argument('--curriculum-dir', type=Path)
    parser.add_argument('--episodes', type=int, default=3000)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--max-minutes', type=float, default=24.)
    parser.add_argument('--updates', type=int, default=4)
    parser.add_argument('--threads', type=int, default=4)
    args = parser.parse_args(argv)
    if args.episodes < 0 or args.updates < 0 or args.threads < 1 or not math.isfinite(args.max_minutes) or args.max_minutes <= 0:
        parser.error('invalid episode/update/thread/time budget')
    torch.set_num_threads(args.threads)
    args.output.mkdir(parents=True, exist_ok=True)
    items = balanced_items([item for item in json.loads(args.registry.read_text())['instances']
                            if item['split'] == 'train' and item.get('group') == 'train'])
    warm, warm_manifest = warm_data_snapshot(args.warm_data)
    curriculum = FrozenCurriculum(args.curriculum_dir) if args.curriculum_dir is not None else None
    if curriculum is not None:
        stage_for_episode(0, args.episodes)
    configuration = dict(route=args.route, seed=args.seed, episodes=args.episodes, updates=args.updates,
                         schema=3, resume_version=RESUME_VERSION, registry_sha256=file_sha256(args.registry),
                         warm_data=warm_manifest, curriculum=None if curriculum is None else curriculum.manifest,
                         bc_target_version=BC_TARGET_VERSION if args.route == 'r4' else None,
                         reward='success +/-1 -0.2 delta raw_T/(raw_T+40)',
                         state_dim=STATE_DIM, action_dim=3 if args.route == 'r4' else 15)
    cfg_file = args.output / 'config.json'
    if cfg_file.exists() and json.loads(cfg_file.read_text()) != configuration:
        raise RuntimeError('cannot resume a different registry/warm-data/training configuration')
    atomic_json(cfg_file, configuration)
    checkpoint = args.output / 'agent.pt'
    if checkpoint.exists():
        agent, committed = load_training_checkpoint(checkpoint, configuration)
        replay = rebuild_replay(args.output, configuration, warm, committed)
    else:
        if list((args.output / 'episodes').glob('*.json')) or list((args.output / 'scene-failures').glob('*.json')):
            raise ValueError('records exist without their initial checkpoint; cannot safely reconstruct warm-start state')
        klass = SACAgent if args.route == 'r4' else DQNAgent
        extra = dict(action_mask_state_indices=(5, 6)) if args.route == 'r4' else {}
        agent = klass(STATE_DIM, configuration['action_dim'], seed=args.seed, **extra)
        replay = warm_start(agent, warm, args.route, steps=200)
        committed = initial_training_state(configuration)
        save_agent(agent, checkpoint, committed)
    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))
    started, env = time.perf_counter(), None
    for index in range(committed['last_committed_episode'] + 1, args.episodes):
        if stop[0] or time.perf_counter() - started > args.max_minutes * 60:
            return 3
        if curriculum is not None:
            curriculum.assert_unchanged()
        record, path = _read_record(args.output, configuration, index)
        if record is None:
            if env is None:
                from motion311_learning.stage_env import StageEnv, EpisodeFailure
                env = StageEnv(timed=True, physical=False)
            # Separate per-episode streams: updates do not depend on how many
            # exploration RNG draws occurred in the completed environment run.
            erng = np.random.default_rng(args.seed * 1000003 + index)
            update_rng = np.random.default_rng(np.random.SeedSequence([args.seed, index, 311]))
            continuous = args.route == 'r4'
            began = time.perf_counter()
            try:
                observation, item, curriculum_meta = curriculum_start(
                    env, curriculum, items, index, args.episodes, args.seed, continuous, erng)
                if not continuous and not len(observation.candidates):
                    raise EpisodeFailure('scene_no_current_candidates')
            except EpisodeFailure as error:
                item, curriculum_meta = env.curriculum_selection
                record, path = write_pending(args.output, configuration, index, item, [], agent, update_rng,
                                             time.perf_counter() - began, scene_reason=error.reason,
                                             curriculum=curriculum_meta)
            else:
                transitions = []
                for stage in range(curriculum_meta['curriculum_start_stage'], 3):
                    if observation.stage != stage:
                        raise ValueError('environment skipped or repeated a suffix stage')
                    inference = time.perf_counter()
                    state = normalized_state(observation.state)
                    if continuous:
                        action = agent.select(state, stage)
                        if len(item['task']) == 1:
                            action[2 if 'left' in item['task'] else 1] = 0.
                        action_index, params = None, action
                    else:
                        epsilon = max(.1, .7 * (1 - index / args.episodes))
                        action = agent.select(state, normalized_actions(observation.candidates), stage, epsilon, erng)
                        action_index, params = action, None
                    inference_ms = (time.perf_counter() - inference) * 1000
                    result = env.step(action)
                    result.info['inference_ms'] = inference_ms
                    transitions.append(transition_record(observation, action_index, result, params,
                                                         behavior=f'{args.route}-update-{index}'))
                    if result.terminated or result.truncated:
                        break
                    observation = result.observation
                if curriculum is not None:
                    curriculum.assert_unchanged()
                record, path = write_pending(args.output, configuration, index, item, transitions, agent, update_rng,
                                             time.perf_counter() - began, curriculum=curriculum_meta)
        if curriculum is not None:
            expected_stage = stage_for_episode(index, args.episodes)
            if (record.get('curriculum_start_stage') != expected_stage
                    or record.get('curriculum', {}).get('library_sha256') != curriculum.manifest['sha256']):
                raise ValueError('pending curriculum provenance/start stage differs from schedule')
        committed = commit_pending(agent, replay, committed, record, path, checkpoint, configuration)
        if index % 100 == 0:
            save_agent(agent, args.output / f'checkpoint-{index:06d}.pt', committed)
        final = record['transitions'][-1]['info'] if record['transitions'] else {}
        print(json.dumps(dict(episode=index, instance=record['instance'], route=args.route,
                              steps=len(record['transitions']), success=bool(final.get('success')),
                              final=record.get('reason', final.get('reason')), replay=len(replay),
                              committed_episode=committed['last_committed_episode'],
                              gradient_updates=committed['gradient_updates'])), flush=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
