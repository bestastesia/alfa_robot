"""Frozen, completed-train-node curriculum; gzip JSON only, never pickle.

First floor(N/5) episodes start at stage 2, the next floor(N/5) at stage 1,
and all remaining episodes start at Home. Sampling is uniform by mode, then
complete task, then unique physical checkpoint within that task.
"""
from __future__ import annotations

import copy
import gzip
import hashlib
import json
import math
from pathlib import Path

import numpy as np

from motion311_learning.contracts import config_digest


VERSION = 'completed-train-prefix-20-20-60-v1'
_ACTIVE = ['updown', *[f'{side}_joint{index}' for side in ('left', 'right') for index in range(1, 8)]]


def _hash(path):
    digest = hashlib.sha256()
    with Path(path).open('rb') as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _arrays(value):
    if isinstance(value, dict):
        if set(value) == {'__array__', 'dtype'}:
            if value['dtype'] not in ('float32', 'float64', 'int32', 'int64', 'bool'):
                raise ValueError('unsupported curriculum array dtype')
            return np.asarray(value['__array__'], dtype=value['dtype'])
        return {key: _arrays(item) for key, item in value.items()}
    if isinstance(value, list):
        return [_arrays(item) for item in value]
    return value


def _load(path):
    with gzip.open(path, 'rt', encoding='utf-8') as handle:
        return _arrays(json.load(handle))


def stage_for_episode(index, episodes):
    if isinstance(episodes, bool) or int(episodes) != episodes or episodes < 5:
        raise ValueError('curriculum requires at least five integer episodes')
    if isinstance(index, bool) or int(index) != index or not 0 <= index < episodes:
        raise ValueError('curriculum episode index is outside its fixed budget')
    chunk = int(episodes) // 5
    return 2 if index < chunk else (1 if index < 2 * chunk else 0)


def _mode(item):
    arm = 'dual' if len(item['task']) == 2 else 'single-' + next(iter(item['task']))
    return arm + ('-top' if item['top_grasp'] else '-side')


def sample_task(items, rng):
    """Uniform mode -> complete task -> instance, including Home episodes."""
    grouped = {}
    for item in sorted(items, key=lambda row: row['instance']):
        if item.get('group') != 'train' or item.get('split') != 'train':
            raise ValueError('curriculum task sampling accepts only explicit train group')
        task = json.dumps(item['task'], sort_keys=True, separators=(',', ':'))
        grouped.setdefault(_mode(item), {}).setdefault(task, []).append(item)
    if not grouped:
        raise ValueError('no train tasks available')
    modes = sorted(grouped)
    tasks = grouped[modes[int(rng.integers(len(modes)))]]
    task_keys = sorted(tasks)
    candidates = tasks[task_keys[int(rng.integers(len(task_keys)))]]
    return copy.deepcopy(candidates[int(rng.integers(len(candidates)))])


def _validate_checkpoint(state, expected_stage=None):
    if not isinstance(state, dict):
        raise ValueError('curriculum checkpoint is not an object')
    stage = state.get('stage')
    if stage not in (0, 1, 2) or state.get('done') is not False:
        raise ValueError('curriculum checkpoint is not a continuing stage')
    if expected_stage is not None and stage != expected_stage:
        raise ValueError('curriculum checkpoint stage differs from edge')
    item, report = state['item'], state['report']
    if item.get('group') != 'train' or item.get('split') != 'train':
        raise ValueError('curriculum nodes require group=train, split=train')
    if state['boxes'] != item['task'] or report['task'] != item['task']:
        raise ValueError('curriculum checkpoint task differs from item')
    blocks = state['timed_blocks']
    if len(blocks) != stage or any(block.get('timed_validation', {}).get('success') is not True for block in blocks):
        raise ValueError('curriculum prefix has not passed timing validation')
    previous = np.asarray(state['previous'], dtype=np.float64)
    home = np.asarray(state['home'], dtype=np.float64)
    if previous.shape != (15,) or home.shape != (15,) or not np.isfinite(previous).all() or not np.isfinite(home).all():
        raise ValueError('invalid curriculum physical state')
    names, frames = report['joint_names'], np.asarray(report['frames'], dtype=np.float64)
    if frames.ndim != 2 or not len(frames) or frames.shape[1] != len(names) or not np.isfinite(frames).all():
        raise ValueError('curriculum prefix frames are malformed')
    indices = [names.index(name) for name in _ACTIVE]
    if not np.allclose(frames[-1, indices], previous, atol=1e-7, rtol=0):
        raise ValueError('curriculum prefix endpoint differs from physical state')
    if not math.isfinite(state['cost_s']) or state['cost_s'] < 0:
        raise ValueError('invalid cumulative curriculum cost')
    return item


class FrozenCurriculum:
    def __init__(self, directory):
        self.directory = Path(directory).resolve()
        if not (self.directory / 'nodes').is_dir() or not (self.directory / 'done').is_dir():
            raise ValueError('curriculum directory must contain completed teacher nodes and done markers')
        self._files = self._snapshot_files()
        self.manifest = {'version': VERSION, 'path': str(self.directory),
                         'sha256': config_digest(self._files), 'file_count': len(self._files),
                         'schedule': {'stage2_fraction': .2, 'stage1_fraction': .2, 'home_remainder': .6,
                                      'rounding': 'floor(N/5), floor(N/5), remainder'},
                         'sampling': 'uniform_mode_then_complete_task_then_unique_node'}
        self.nodes = {1: {}, 2: {}}
        seen = set()
        for directory in sorted((self.directory / 'nodes').iterdir()):
            if not directory.is_dir():
                continue
            done_file, root_file = self.directory / 'done' / (directory.name + '.json'), directory / 'root.json.gz'
            if not done_file.is_file() or not (directory / 'root.done').is_file() or not root_file.is_file():
                raise ValueError('curriculum library contains an incomplete teacher tree: ' + directory.name)
            summary, root = json.loads(done_file.read_text()), _load(root_file)
            root_item = _validate_checkpoint(root, 0)
            expected = f"{summary['instance']}-r{summary['repeat']}"
            if directory.name != expected or summary['instance'] != root_item['instance']:
                raise ValueError('teacher done marker does not identify its root')
            for edge_file in sorted(directory.glob('edge-*.json.gz')):
                edge = _load(edge_file)
                row, state = edge['row'], edge['after']
                if state is None:
                    if not row['terminated'] and not row['truncated']:
                        raise ValueError('continuing teacher edge has no checkpoint')
                    continue
                if row['terminated'] or row['truncated']:
                    raise ValueError('terminal teacher edge unexpectedly has a continuation')
                item = _validate_checkpoint(state, int(row['stage']) + 1)
                if item != root_item:
                    raise ValueError('teacher edge item differs from root')
                stage = state['stage']
                # Ignore aliases for exactly the same source state; stable first
                # path wins, preventing copied prefix edges from biasing sampling.
                identity = config_digest(dict(item=item, stage=stage, seed=state['seed'],
                                               previous=np.asarray(state['previous']).tolist(), cost_s=state['cost_s'],
                                               frames=state['report']['frames']))
                if identity in seen:
                    continue
                seen.add(identity)
                relative = edge_file.relative_to(self.directory).as_posix()
                task = json.dumps(item['task'], sort_keys=True, separators=(',', ':'))
                node = {'path': relative, 'sha256': self._files[relative], 'item': copy.deepcopy(item),
                        'original_seed': int(state['seed']), 'start_stage': stage}
                self.nodes[stage].setdefault(_mode(item), {}).setdefault(task, []).append(node)
        if not all(self.nodes[stage] for stage in (1, 2)):
            raise ValueError('curriculum requires both stage-1 and stage-2 validated nodes; no Home fallback')
        self.manifest['nodes_by_stage'] = {str(stage): sum(len(rows) for tasks in self.nodes[stage].values()
                                                          for rows in tasks.values()) for stage in (1, 2)}
        self.assert_unchanged()

    def _snapshot_files(self):
        if any(self.directory.rglob('*.tmp')):
            raise ValueError('curriculum library contains in-progress temporary files')
        files = []
        config = self.directory / 'config.json'
        if not config.is_file():
            raise ValueError('curriculum library lacks its teacher configuration')
        files.append(config)
        files.extend((self.directory / 'nodes').rglob('*.json.gz'))
        files.extend((self.directory / 'nodes').rglob('*.done'))
        files.extend((self.directory / 'done').glob('*.json'))
        return {path.relative_to(self.directory).as_posix(): _hash(path) for path in sorted(files)}

    def assert_unchanged(self):
        if self._snapshot_files() != self._files:
            raise ValueError('frozen curriculum source files changed; start a new training run')

    def sample(self, stage, rng, new_seed, continuous):
        if stage not in (1, 2):
            raise ValueError('curriculum nodes are only used for stages 1 and 2')
        modes = sorted(self.nodes[stage])
        tasks = self.nodes[stage][modes[int(rng.integers(len(modes)))]]
        task_keys = sorted(tasks)
        nodes = tasks[task_keys[int(rng.integers(len(task_keys)))]]
        node = nodes[int(rng.integers(len(nodes)))]
        path = self.directory / node['path']
        if _hash(path) != node['sha256']:
            raise ValueError('selected frozen curriculum node changed')
        state = _load(path)['after']
        item = _validate_checkpoint(state, stage)
        metadata = {'version': VERSION, 'library_sha256': self.manifest['sha256'],
                    'curriculum_start_stage': stage, 'source_path': node['path'],
                    'source_sha256': node['sha256'], 'source_seed': int(state['seed']),
                    'planner_seed': int(new_seed), 'task': copy.deepcopy(item['task']),
                    'instance': item['instance'], 'offset_m': copy.deepcopy(item['offset_m'])}
        state['seed'] = int(new_seed)
        state['continuous'] = bool(continuous)
        return state, copy.deepcopy(item), metadata
