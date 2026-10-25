"""CPU-only recovery tests; import collect with an explicit fake StageEnv module."""
from collections import Counter
from pathlib import Path
import copy
import importlib.util
import json
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import motion311_learning
from motion311_learning.contracts import Observation, StepResult
from motion311_learning.dataset import finish_episode, load_episodes
from motion311_learning.features import PHYSICS_DIM, STATE_DIM
from motion311_learning.supervised import aggregate_stage


def source_path(name):
    for directory in motion311_learning.__path__:
        path = Path(directory) / name
        if path.is_file():
            return path
    raise ImportError(name)


def load_collect():
    fake_environment = types.ModuleType('motion311_learning.stage_env')
    fake_environment.StageEnv = TeacherEnv
    fake_environment.EpisodeFailure = SceneFailure
    spec = importlib.util.spec_from_file_location('_cpu_teacher_resume_collect', source_path('collect.py'))
    module = importlib.util.module_from_spec(spec)
    with patch.dict(sys.modules, {'motion311_learning.stage_env': fake_environment}):
        spec.loader.exec_module(module)
    return module


class SceneFailure(Exception):
    pass


class TeacherEnv:
    caps = (4, 4, 4)
    events = []
    reset_count = 0

    def __init__(self, timed=True, physical=True):
        if not timed or not physical:
            raise AssertionError('teacher should preserve timed physical setup')
        self.path = []

    def observe(self):
        stage = len(self.path)
        state = np.zeros(STATE_DIM, dtype=np.float64)
        state[stage] = 1.0
        state[5:7] = [1.0, 0.0]
        state[7] = sum((i + 1) * action for i, action in enumerate(self.path)) * .01
        count = 4 if stage != 2 else 2 + (self.path[0] % 3)
        candidates = np.zeros((count, 15), dtype=np.float64)
        candidates[:, 0] = -.5
        candidates[:, 1] = np.arange(count) * .1
        actions = np.zeros((count, 3), dtype=np.float64)
        actions[:, 1] = np.arange(count) * .1
        return Observation(stage, state, candidates, actions, np.zeros((count, PHYSICS_DIM)),
                           np.repeat(np.eye(4)[None], 2, axis=0))

    def reset(self, item, seed):
        TeacherEnv.reset_count += 1
        self.item, self.seed, self.path = copy.deepcopy(item), seed, []
        return self.observe()

    def checkpoint(self):
        return dict(item=self.item, seed=self.seed, path=list(self.path),
                    previous=np.arange(15, dtype=np.float64) + len(self.path),
                    timed_blocks=[dict(frames=np.zeros((2, 18)), label='validated')])

    def restore(self, checkpoint):
        # Check dtype/shape preservation through the gzip JSON representation.
        if checkpoint['previous'].dtype != np.dtype('float64'):
            raise AssertionError('checkpoint array dtype changed')
        if checkpoint['timed_blocks'][0]['frames'].shape != (2, 18):
            raise AssertionError('timed block shape changed')
        self.item, self.seed, self.path = copy.deepcopy(checkpoint['item']), checkpoint['seed'], list(checkpoint['path'])
        return self.observe()

    def step(self, action):
        child = self.path + [int(action)]
        TeacherEnv.events.append(tuple(child))
        self.path = child
        stage = len(child) - 1
        # Most stage-1 alternatives fail early; action 0 expands further.
        terminal = stage == 2 or (stage == 1 and action != 0)
        success = stage == 2 and action % 2 == 0
        info = dict(stage=stage, success=success, cumulative_cost_s=float(stage + 1),
                    class_='success' if success else ('task_failure' if terminal else 'stage_success'))
        return StepResult(None if terminal else self.observe(),
                          (1.0 if success else -1.0) if terminal else -.01,
                          terminal, False, info)


class TeacherResumeTest(unittest.TestCase):
    def setUp(self):
        TeacherEnv.events, TeacherEnv.reset_count = [], 0
        self.collect = load_collect()

    def registry(self, folder):
        path = Path(folder) / 'registry.json'
        path.write_text(json.dumps({'instances': [dict(instance='train-one', split='train', group='train',
                                                       task={'left': 1}, kind='single', top_grasp=False,
                                                       offset_m=[0., 0.])]}))
        return path

    def run_collect(self, registry, output):
        argv = ['collect', '--registry', str(registry), '--output', str(output),
                '--branches', '4', '--max-minutes', '24', '--seed', '311']
        with patch.object(sys, 'argv', argv), patch.object(self.collect.signal, 'signal'), patch('builtins.print'), \
             patch.object(self.collect, 'fingerprint', return_value={'digest': 'cpu-fixture'}), \
             patch.object(self.collect, 'assert_fingerprint'):
            self.collect.main()

    def leaves(self, output):
        values = {}
        for path in sorted((Path(output) / 'episodes').glob('*.json')):
            value = json.loads(path.read_text())
            value.pop('elapsed_s', None)
            values[path.name] = value
        return values

    def compare_resumed(self, hook):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            registry = self.registry(root)
            reference, resumed = root / 'reference', root / 'resumed'
            self.run_collect(registry, reference)
            expected_edges = list(TeacherEnv.events)
            expected_leaves = self.leaves(reference)
            TeacherEnv.events, TeacherEnv.reset_count = [], 0
            with hook(resumed):
                with self.assertRaises(SystemExit) as stopped:
                    self.run_collect(registry, resumed)
            self.assertEqual(stopped.exception.code, 3)
            existing_edges = {tuple(map(int, path.name.removeprefix('edge-').removesuffix('.json.gz').split('-')))
                              for path in (resumed / 'nodes').rglob('edge-*.json.gz')}
            already_executed = list(TeacherEnv.events)
            self.assertTrue(existing_edges)
            self.run_collect(registry, resumed)
            self.assertEqual(TeacherEnv.reset_count, 1, 'root must restore rather than reset')
            counts = Counter(TeacherEnv.events)
            self.assertTrue(all(counts[edge] == 1 for edge in existing_edges))
            self.assertEqual(Counter(TeacherEnv.events), Counter(expected_edges))
            self.assertEqual(self.leaves(resumed), expected_leaves)
            before = len(TeacherEnv.events)
            self.run_collect(registry, resumed)
            self.assertEqual(len(TeacherEnv.events), before)
            self.assertEqual(TeacherEnv.reset_count, 1)
            self.assertFalse(list(resumed.rglob('*.tmp')))
            self.assertEqual(len(already_executed), len(existing_edges))
            return list(expected_leaves.values())

    def test_resume_after_durable_edge_never_repeats_step_and_keeps_path_probabilities(self):
        original = self.collect.save_node
        def hook(output):
            counter = [0]
            def save(path, data):
                original(path, data)
                if path.name.startswith('edge-'):
                    counter[0] += 1
                    if counter[0] == 5:
                        raise InterruptedError('simulated exit after atomic edge write')
            return patch.object(self.collect, 'save_node', side_effect=save)
        leaves = self.compare_resumed(hook)
        root_mass = {}
        for leaf in leaves:
            rows = leaf['transitions']
            for index, row in enumerate(rows):
                expected = np.prod([future['selection_probability'] for future in rows[index + 1:]])
                self.assertAlmostEqual(row['suffix_weight'], expected)
                self.assertTrue(0 < row['selection_probability'] <= 1)
            root = rows[0]['action_index']
            root_mass[root] = root_mass.get(root, 0.) + rows[0]['suffix_weight']
        self.assertEqual(set(root_mass), {0, 1, 2, 3})
        for mass in root_mass.values():
            self.assertAlmostEqual(mass, 1.0)

    def test_resume_after_leaf_edge_before_episode_write_recovers_leaf_once(self):
        original = self.collect.write_episode
        counter = [0]
        def hook(output):
            def write(*args, **kwargs):
                counter[0] += 1
                if counter[0] == 2:
                    raise InterruptedError('simulated exit before durable leaf record')
                return original(*args, **kwargs)
            return patch.object(self.collect, 'write_episode', side_effect=write)
        self.compare_resumed(hook)

    def test_resume_after_node_done_preserves_completed_subtree(self):
        original = Path.write_text
        fired = [False]
        def hook(output):
            def write(path, *args, **kwargs):
                result = original(path, *args, **kwargs)
                if path.suffix == '.done' and not fired[0]:
                    fired[0] = True
                    raise InterruptedError('simulated exit after node completion')
                return result
            return patch.object(Path, 'write_text', new=write)
        self.compare_resumed(hook)

    def test_resume_after_root_completion_only_rebuilds_summary(self):
        original = Path.write_text
        fired = [False]
        def hook(output):
            def write(path, *args, **kwargs):
                result = original(path, *args, **kwargs)
                if path.name == 'root.done' and not fired[0]:
                    fired[0] = True
                    raise InterruptedError('simulated exit before scene done marker')
                return result
            return patch.object(Path, 'write_text', new=write)
        self.compare_resumed(hook)

    def test_weighted_supervised_prefix_is_one_quarter_not_four_sevenths(self):
        # Four next choices: three terminate in failure, one expands into four
        # successful leaves. Root-action value must weight choices, not files.
        episodes = []
        for choice in range(4):
            for last in (range(4) if choice == 0 else range(1)):
                length = 3 if choice == 0 else 2
                transitions = []
                for stage in range(length):
                    state = np.zeros(STATE_DIM)
                    state[stage] = 1.
                    state[5] = 1.
                    action = np.zeros(15)
                    if stage == 1:
                        action[1] = choice
                    if stage == 2:
                        action[1] = last
                    terminal = stage == length - 1
                    success = choice == 0 and terminal
                    transitions.append(dict(stage=stage, state=state.tolist(), action_q=action.tolist(),
                                            action_physics=np.zeros(PHYSICS_DIM).tolist(),
                                            reward=(1. if success else -1.) if terminal else 0.,
                                            terminated=terminal, truncated=False, selection_probability=.25,
                                            next=None, info=dict(success=success, cumulative_cost_s=3.)))
                finish_episode(transitions)
                episodes.append(dict(split='train', group='train', task={'left': 1}, transitions=transitions))
        grouped = aggregate_stage(episodes, 0)
        self.assertEqual(len(grouped), 1)
        self.assertEqual(grouped[0]['count'], 7)
        self.assertAlmostEqual(grouped[0]['effective_weight'], 1.)
        self.assertAlmostEqual(grouped[0]['success_probability'], .25)
        self.assertAlmostEqual(grouped[0]['value'], -.5)
        self.assertAlmostEqual(grouped[0]['log_success_cost'], np.log(3.))

    def test_weighted_prefix_mean_matches_equal_next_action_policy(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            registry, output = self.registry(root), root / 'out'
            self.run_collect(registry, output)
            leaves = list(load_episodes(output / 'episodes', split='train'))
            # Use direct weighted expectation: terminal stage-1 choices have 1/4
            # mass each; the one expanded choice shares its 1/4 over its leaves.
            for action in range(4):
                selected = [episode for episode in leaves if episode['transitions'][0]['action_index'] == action]
                values = [episode['transitions'][0]['return'] for episode in selected]
                weights = [episode['transitions'][0]['suffix_weight'] for episode in selected]
                last_count = 2 + action % 3
                final_mean = np.mean([1. if choice % 2 == 0 else -1. for choice in range(last_count)])
                expected = -.01 + (-3.0 + (-.01 + final_mean)) / 4.0
                self.assertAlmostEqual(float(np.average(values, weights=weights)), expected)
                self.assertAlmostEqual(sum(weights), 1.0)


if __name__ == '__main__':
    unittest.main()
