from pathlib import Path
import copy
import gzip
import json
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion311_learning.contracts import Observation, StepResult, potential
from motion311_learning.curriculum import FrozenCurriculum, stage_for_episode
from motion311_learning.features import PHYSICS_DIM, STATE_DIM
from motion311_learning import train


NAMES = ['updown', *[f'{side}_joint{i}' for side in ('left', 'right') for i in range(1, 8)]]


def item(index=1, side='left', top=False):
    return dict(instance=f'{side}-{index}', task={side: index}, split='train', group='train',
                kind='single', top_grasp=top, offset_m=[.01, -.01])


def checkpoint(task, stage):
    previous = np.full(15, stage * .05)
    return dict(item=copy.deepcopy(task), boxes=task['task'], seed=311, continuous=False,
                previous=previous, home=np.zeros(15), stage=stage, done=False, cost_s=float(stage * 2),
                report=dict(task=task['task'], joint_names=NAMES, frames=[previous.tolist()], phases=['extract'], payload=[True]),
                timed_blocks=[dict(timed_validation={'success': True}, frames=[[0.] * 15]) for _ in range(stage)],
                targets={}, carry_targets={}, box_centers={})


def encode(value):
    if isinstance(value, np.ndarray):
        return {'__array__': value.tolist(), 'dtype': str(value.dtype)}
    if isinstance(value, dict):
        return {key: encode(data) for key, data in value.items()}
    if isinstance(value, list):
        return [encode(data) for data in value]
    return value


def put_gzip(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(gzip.compress(json.dumps(encode(value), sort_keys=True).encode(), mtime=0))


def build_library(root, tasks=None, stages=(1, 2)):
    root = Path(root)
    root.mkdir()
    (root / 'config.json').write_text('{"source":"fixture","split":"train"}')
    tasks = tasks or [item()]
    for task in tasks:
        name = task['instance'] + '-r0'
        directory = root / 'nodes' / name
        put_gzip(directory / 'root.json.gz', checkpoint(task, 0))
        for stage in stages:
            put_gzip(directory / ('edge-' + '-'.join(['0'] * stage) + '.json.gz'),
                     dict(row=dict(stage=stage - 1, terminated=False, truncated=False), after=checkpoint(task, stage)))
        (directory / 'root.done').write_text('complete\n')
        (root / 'done').mkdir(exist_ok=True)
        (root / 'done' / (name + '.json')).write_text(json.dumps(dict(instance=task['instance'], repeat=0)))
    return root


class FakeEpisodeFailure(Exception):
    pass


class CurriculumEnv:
    seen_starts = []
    steps = []

    def __init__(self, timed=True, physical=False):
        self.timed, self.physical = timed, physical

    def observation(self):
        state = np.zeros(STATE_DIM)
        state[self.stage] = 1
        state[5:7] = [float('left' in self.item['task']), float('right' in self.item['task'])]
        state[-1] = potential(self.cost_s)
        candidates = np.empty((0, 15)) if self.continuous else np.zeros((2, 15))
        return Observation(self.stage, state, candidates, np.zeros((len(candidates), 3)),
                           np.zeros((len(candidates), PHYSICS_DIM)), np.repeat(np.eye(4)[None], 2, axis=0))

    def restore(self, state):
        self.state = copy.deepcopy(state)
        self.item, self.stage, self.cost_s = state['item'], state['stage'], state['cost_s']
        self.seed, self.continuous = state['seed'], state['continuous']
        CurriculumEnv.seen_starts.append((self.stage, self.seed, self.continuous, self.cost_s))
        return self.observation()

    def reset(self, task, seed, continuous):
        self.item, self.stage, self.seed, self.continuous, self.cost_s = task, 0, seed, continuous, 0.
        CurriculumEnv.seen_starts.append((0, seed, continuous, 0.))
        return self.observation()

    def step(self, action):
        stage = self.stage
        if self.continuous:
            inactive = 2 if 'left' in self.item['task'] else 1
            if action[inactive] != 0.:
                raise AssertionError('inactive psi must remain zero')
        CurriculumEnv.steps.append((stage, self.seed))
        self.cost_s += 1
        terminal = stage == 2
        self.stage += 1
        info = dict(success=terminal, cumulative_cost_s=self.cost_s, seed=self.seed)
        return StepResult(None if terminal else self.observation(), 1. if terminal else -.01, terminal, False, info)


class LearningCurriculumTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old_threads = torch.get_num_threads()
        torch.set_num_threads(4)

    @classmethod
    def tearDownClass(cls):
        torch.set_num_threads(cls.old_threads)

    def setUp(self):
        CurriculumEnv.seen_starts, CurriculumEnv.steps = [], []

    def test_stage_budget_is_twenty_twenty_sixty_and_rounding_is_explicit(self):
        self.assertEqual([stage_for_episode(i, 10) for i in range(10)], [2, 2, 1, 1, 0, 0, 0, 0, 0, 0])
        self.assertEqual([stage_for_episode(i, 7) for i in range(7)], [2, 1, 0, 0, 0, 0, 0])
        with self.assertRaises(ValueError):
            stage_for_episode(0, 4)
        with self.assertRaises(ValueError):
            stage_for_episode(5, 5)

    def test_restore_overwrites_seed_and_continuous_before_observe_preserves_cost(self):
        with tempfile.TemporaryDirectory() as folder:
            library = FrozenCurriculum(build_library(Path(folder) / 'library'))
            env = CurriculumEnv()
            observation, chosen, metadata = train.curriculum_start(env, library, [item()], 0, 10, 19, True,
                                                                    np.random.default_rng(19))
            self.assertEqual(observation.stage, 2)
            self.assertEqual(len(observation.candidates), 0)
            self.assertEqual(env.seed, 19 * 1000003)
            self.assertNotEqual(env.seed, 311)
            self.assertTrue(env.continuous)
            self.assertEqual(env.cost_s, 4.0)
            self.assertEqual(len(env.state['timed_blocks']), 2)
            self.assertEqual(metadata['source_seed'], 311)
            self.assertEqual(metadata['planner_seed'], env.seed)
            self.assertEqual(metadata['curriculum_start_stage'], 2)
            self.assertTrue(metadata['source_sha256'])
            state, _, _ = library.sample(2, np.random.default_rng(19), 99, False)
            self.assertFalse(state['continuous'])
            self.assertEqual(state['seed'], 99)

    def test_missing_stage_incomplete_tree_and_nontrain_nodes_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            missing = build_library(root / 'missing', stages=(1,))
            with self.assertRaisesRegex(ValueError, 'both stage'):
                FrozenCurriculum(missing)
            incomplete = build_library(root / 'incomplete')
            next((incomplete / 'nodes').rglob('root.done')).unlink()
            with self.assertRaisesRegex(ValueError, 'incomplete'):
                FrozenCurriculum(incomplete)
            bad = item(); bad['group'] = 'seen_test'
            nontrain = build_library(root / 'nontrain', tasks=[bad])
            with self.assertRaisesRegex(ValueError, 'group=train'):
                FrozenCurriculum(nontrain)

    def test_changed_source_or_added_node_changes_frozen_library(self):
        with tempfile.TemporaryDirectory() as folder:
            root = build_library(Path(folder) / 'library')
            library = FrozenCurriculum(root)
            source = next((root / 'nodes').rglob('edge-0-0.json.gz'))
            source.write_bytes(source.read_bytes() + b'\n')
            with self.assertRaisesRegex(ValueError, 'changed'):
                library.assert_unchanged()
            with self.assertRaisesRegex(ValueError, 'changed'):
                library.sample(2, np.random.default_rng(1), 77, True)

    def test_sampling_is_reproducible_and_uniform_by_mode_before_task(self):
        with tempfile.TemporaryDirectory() as folder:
            tasks = [item(i, side='left') for i in range(1, 8)] + [item(20, side='right', top=True)]
            library = FrozenCurriculum(build_library(Path(folder) / 'library', tasks=tasks))
            a, b = np.random.default_rng(7), np.random.default_rng(7)
            counts = {'left': 0, 'right': 0}
            for _ in range(300):
                _, chosen, meta = library.sample(1, a, 7, False)
                _, other, meta2 = library.sample(1, b, 7, False)
                self.assertEqual(meta, meta2)
                self.assertEqual(chosen, other)
                counts[next(iter(chosen['task']))] += 1
            self.assertTrue(110 < counts['right'] < 190, counts)

    def run_training(self, root, route, crash=False):
        library = build_library(root / 'library')
        registry = root / 'registry.json'
        registry.write_text(json.dumps({'instances': [item()]}))
        output = root / 'run'
        argv = ['--registry', str(registry), '--output', str(output), '--route', route, '--curriculum-dir', str(library),
                '--episodes', '5', '--seed', '9', '--threads', '4', '--updates', '1']
        environment = types.ModuleType('motion311_learning.stage_env')
        environment.StageEnv, environment.EpisodeFailure = CurriculumEnv, FakeEpisodeFailure
        previous_module = sys.modules.get('motion311_learning.stage_env')
        sys.modules['motion311_learning.stage_env'] = environment
        self.addCleanup(lambda: sys.modules.pop('motion311_learning.stage_env', None)
                        if previous_module is None else sys.modules.__setitem__('motion311_learning.stage_env', previous_module))
        with patch.object(train.signal, 'signal'), patch('builtins.print'):
            if crash:
                original = train.commit_pending
                fired = [False]
                def interrupt(*args, **kwargs):
                    if not fired[0]:
                        fired[0] = True
                        raise InterruptedError('durable curriculum suffix, checkpoint not committed')
                    return original(*args, **kwargs)
                with patch.object(train, 'commit_pending', side_effect=interrupt):
                    with self.assertRaises(InterruptedError):
                        train.main(argv)
                self.assertEqual(len(CurriculumEnv.steps), 1)
            self.assertEqual(train.main(argv), 0)
        records = [json.loads(path.read_text()) for path in sorted((output / 'episodes').glob('*.json'))]
        return records, argv, output, library

    def test_suffix_only_training_and_pending_recovery_do_not_repeat_prefix(self):
        for route in ('r3', 'r4'):
            with self.subTest(route=route), tempfile.TemporaryDirectory() as folder:
                CurriculumEnv.seen_starts, CurriculumEnv.steps = [], []
                records, argv, output, library = self.run_training(Path(folder), route, crash=True)
                self.assertEqual([row['curriculum_start_stage'] for row in records], [2, 1, 0, 0, 0])
                self.assertEqual([[step['stage'] for step in row['transitions']] for row in records],
                                 [[2], [1, 2], [0, 1, 2], [0, 1, 2], [0, 1, 2]])
                self.assertEqual(len(CurriculumEnv.steps), 12)
                self.assertEqual([start[0] for start in CurriculumEnv.seen_starts], [2, 1, 0, 0, 0])
                self.assertEqual([start[1] for start in CurriculumEnv.seen_starts], [9 * 1000003 + i for i in range(5)])
                self.assertEqual(records[0]['curriculum']['source_seed'], 311)
                self.assertEqual(records[0]['transitions'][0]['return'], 1.0)
                config = json.loads((output / 'config.json').read_text())
                self.assertEqual(config['curriculum']['sha256'], records[0]['curriculum']['library_sha256'])
                # Changing even nonselected source metadata makes resume fail closed.
                (library / 'config.json').write_text('{"changed": true}')
                with patch.object(train.signal, 'signal'), self.assertRaisesRegex(RuntimeError, 'configuration'):
                    train.main(argv)


if __name__ == '__main__':
    unittest.main()
