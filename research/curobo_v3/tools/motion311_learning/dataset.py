"""High-precision, atomic per-episode replay records shared across four routes."""
import hashlib
import json
from pathlib import Path

import numpy as np

from motion311_learning.contracts import SCHEMA, config_digest


def observation_record(observation):
    if observation is None:
        return None
    return dict(stage=observation.stage, state=observation.state.tolist(), candidates=observation.candidates.tolist(),
                actions=observation.actions.tolist(), physics=observation.physics.tolist(), targets=observation.targets.tolist())


def transition_record(observation, action_index, result, action_params=None, behavior='unknown'):
    row = dict(stage=observation.stage, state=observation.state.tolist(), candidates=observation.candidates.tolist(),
               candidate_actions=observation.actions.tolist(), physics=observation.physics.tolist(),
               action_index=action_index, action_params=None if action_params is None else np.asarray(action_params).tolist(),
               reward=float(result.reward), terminated=bool(result.terminated), truncated=bool(result.truncated),
               next=observation_record(result.observation), info=result.info, behavior=behavior)
    if action_index is not None:
        row['action_q'] = observation.candidates[action_index].tolist()
        row['action_physics'] = observation.physics[action_index].tolist()
        row['action_params'] = observation.actions[action_index].tolist()
    else:
        row['action_q'] = result.info.get('action_q')
        row['action_physics'] = None
    return row


def finish_episode(transitions):
    """Attach realized suffix rewards (not mislabeled as optimal Q) and final success."""
    if not transitions:
        raise ValueError('empty episode')
    last = transitions[-1]
    if not last['terminated'] and not last['truncated']:
        raise ValueError('incomplete episode cannot train complete-return targets')
    total = 0.
    success = bool(last['info'].get('success'))
    truncated = any(row['truncated'] for row in transitions)
    suffix_weight = 1.
    for row in reversed(transitions):
        total += row['reward']
        row['return'] = total
        row['suffix_weight'] = suffix_weight
        probability = float(row.get('selection_probability', 1.))
        if not 0. < probability <= 1.:
            raise ValueError('selection_probability must be in (0, 1]')
        suffix_weight *= probability
        row['episode_success'] = success
        row['trainable'] = not truncated
    return transitions


def write_episode(folder, identity, item, transitions, policy, config, elapsed_s):
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    if item['split'] not in ('train', 'dev', 'test') or item.get('group', item['split']) != item['split']:
        raise ValueError('only explicit train/dev/test groups may be recorded here')
    transitions = finish_episode(transitions)
    data = dict(schema=SCHEMA, identity=identity, instance=item['instance'], split=item['split'], group=item.get('group', item['split']), task=item['task'],
                offset_m=item['offset_m'], policy=policy, config_sha256=config_digest(config), elapsed_s=float(elapsed_s),
                transitions=transitions)
    target = folder / (identity + '.json')
    if target.exists():
        raise FileExistsError(target)
    temporary = target.with_suffix('.tmp')
    temporary.write_text(json.dumps(data, ensure_ascii=False, allow_nan=False)+'\n')
    temporary.replace(target)
    return target


def teacher_tree_complete(folder, item):
    """A partial DFS tree is biased even with correct branch weights; only completed trees supply MC labels."""
    if item.get('policy') != 'branching_teacher':
        return True
    identity = item['identity']
    prefix = item['instance'] + '-r'
    if not identity.startswith(prefix):
        raise ValueError('teacher identity does not match instance')
    repeat = identity[len(prefix):].split('-', 1)[0]
    marker = Path(folder).parent / 'done' / f"{item['instance']}-r{int(repeat)}.json"
    return marker.is_file()


def load_episodes(folder, split='train'):
    for path in sorted(Path(folder).glob('*.json')):
        item = json.loads(path.read_text())
        if item.get('schema') != SCHEMA or item.get('split') != split or item.get('group') != split:
            continue
        if not teacher_tree_complete(folder, item):
            continue
        if any(not row.get('trainable', False) for row in item['transitions']):
            continue
        yield item


def state_scale():
    # stage one-hot, mode flags, previous/home q, two current target poses, two carry goal poses,
    # box centers, xy offset, cost potential. All normalization is fixed from physical units, never dev/test fitted.
    return np.concatenate([np.ones(7), [1.] + [np.pi] * 14, [1.] + [np.pi] * 14,
                           np.tile([2., 2., 2.] + [1.] * 9, 4), np.full(6, 2.), [.05, .05], [1.]])


def normalized_state(values):
    return np.asarray(values, dtype=np.float32) / state_scale().astype(np.float32)


def normalized_actions(rows, physics=None):
    rows = np.asarray(rows, dtype=np.float32).reshape(-1, 15)
    values = rows / np.array([1.] + [np.pi] * 14, dtype=np.float32)
    if physics is not None:
        from motion311_learning.features import PHYSICS_DIM
        body = np.asarray(physics, dtype=np.float32).reshape(len(rows), PHYSICS_DIM)
        scale = np.array([1., np.pi, np.pi] + [1.] * 8 + [2.] * 12, dtype=np.float32)
        values = np.concatenate([values, body / scale], axis=1)
    return values


def learning_transition(row, physical=False, continuous=False):
    if row.get('truncated'):
        raise ValueError('infrastructure truncation cannot enter replay')
    future = row.get('next')
    return dict(stage=int(row['stage']), state=normalized_state(row['state']),
                action=np.asarray(row['action_params'], dtype=np.float32) if continuous else normalized_actions([row['action_q']],
                    [row['action_physics']] if physical else None)[0], reward=float(row['reward']),
                terminated=bool(row['terminated']), next_state=None if future is None else normalized_state(future['state']),
                next_candidates=None if future is None or continuous else normalized_actions(future['candidates'], future['physics'] if physical else None))
