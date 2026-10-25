from pathlib import Path
import copy
import json
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion311_learning.contracts import SCHEMA, potential, reward
from motion311_learning.features import PHYSICS_DIM, STATE_DIM
from motion311_learning.supervised import (
    TARGET_VERSION, SupervisedPolicy, aggregate_stage, fit_episodes, main,
    split_task_groups, teacher_targets, validate_train_episodes,
)


def make_episodes(tasks=6):
    episodes = []
    good = np.zeros(15); good[0], good[1] = -0.5, 0.5
    bad = good.copy(); bad[1] = -0.5
    candidates = np.stack([good, bad])
    physics = np.zeros((2, PHYSICS_DIM))
    for task in range(tasks):
        states = []
        for stage in range(3):
            state = np.zeros(STATE_DIM)
            state[stage] = 1.0
            state[3:7] = [0, 0, 1, 0]
            state[7] = task * 0.05
            state[-1] = potential(float(stage))
            states.append(state)
        for choice in (0, 1):
            rows = []
            success = choice == 0
            for stage in range(3):
                future = None if stage == 2 else dict(stage=stage + 1, state=states[stage + 1].tolist(),
                                                     candidates=candidates.tolist(), physics=physics.tolist())
                rows.append(dict(stage=stage, state=states[stage].tolist(), action_q=candidates[choice].tolist(),
                                 action_physics=physics[choice].tolist(), reward=reward(stage, stage + 1, success if stage == 2 else None),
                                 terminated=stage == 2, truncated=False, trainable=True, episode_success=success,
                                 next=future, info=dict(cumulative_cost_s=float(stage + 1), success=success if stage == 2 else False)))
            suffix = 0.0
            for row in reversed(rows):
                suffix += row['reward']; row['return'] = suffix
            episodes.append(dict(split='train', group='train', task={'left': task + 1}, instance=f'task-{task}-{choice}',
                                 transitions=rows, offset_m=[0.0, 0.0]))
    return episodes


class SupervisedLearningTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.old_threads = torch.get_num_threads()
        torch.set_num_threads(4)
        cls.episodes = make_episodes()
        cls.policy, cls.report = fit_episodes(cls.episodes, seed=19, widths=(8,), epochs=18,
                                             patience=6, threads=4)

    @classmethod
    def tearDownClass(cls):
        torch.set_num_threads(cls.old_threads)

    def test_aggregate_uses_observed_mean_not_luckiest_leaf(self):
        one = copy.deepcopy(self.episodes[0])
        two = copy.deepcopy(one)
        for row in two['transitions']:
            row['return'] = -1.0
            row['episode_success'] = False
        records = aggregate_stage([one, two], 0)
        self.assertEqual(len(records), 1)
        self.assertEqual(records[0]['count'], 2)
        self.assertAlmostEqual(records[0]['value'], (one['transitions'][0]['return'] - 1.0) / 2.0)
        self.assertEqual(records[0]['success_probability'], 0.5)
        self.assertAlmostEqual(records[0]['log_success_cost'], np.log(3.0))

    def test_branch_suffix_probability_corrects_leaf_count_bias(self):
        # Three immediate failures vs one next action with four successful descendants:
        # policy success is 1/4, not the unweighted-leaf rate 4/7.
        leaves = []
        for success in (False, False, False, True, True, True, True):
            episode = copy.deepcopy(self.episodes[0])
            for row in episode['transitions']:
                row['return'] = 1. if success else -1.
                row['episode_success'] = success
                row['suffix_weight'] = 1. / 16 if success else 1. / 4
            leaves.append(episode)
        group = aggregate_stage(leaves, 0)[0]
        self.assertAlmostEqual(group['success_probability'], .25)
        self.assertAlmostEqual(group['value'], -.5)

    def test_task_groups_keep_all_offsets_and_repeats_together(self):
        repeated = copy.deepcopy(self.episodes[0])
        repeated['instance'] = 'same-task-another-offset'
        repeated['offset_m'] = [0.02, -0.02]
        episodes = self.episodes + [repeated]
        train, validation, split = split_task_groups(episodes, seed=7)
        train_tasks = {tuple(sorted(row['task'].items())) for row in train}
        validation_tasks = {tuple(sorted(row['task'].items())) for row in validation}
        self.assertFalse(train_tasks & validation_tasks)
        self.assertEqual(split, split_task_groups(list(reversed(episodes)), seed=7)[2])
        self.assertEqual(len(train) + len(validation), len(episodes))
        with self.assertRaisesRegex(ValueError, 'at least two'):
            split_task_groups(self.episodes[:2])

    def test_external_dev_test_and_truncations_rejected(self):
        for split in ('dev', 'test'):
            episodes = copy.deepcopy(self.episodes)
            episodes[0]['split'] = split
            with self.assertRaisesRegex(ValueError, 'train only'):
                fit_episodes(episodes, epochs=1)
            with self.assertRaisesRegex(ValueError, 'train only'):
                aggregate_stage(episodes, 0)
        episodes = copy.deepcopy(self.episodes)
        episodes[0]['transitions'][0]['truncated'] = True
        with self.assertRaisesRegex(ValueError, 'truncated'):
            validate_train_episodes(episodes)

    def test_teacher_reads_next_stage_and_mixes_with_empirical_mean(self):
        records = aggregate_stage([self.episodes[0]], 0)
        class FrozenTeacher:
            physical = False
            calls = []
            def predict_values(self, state, candidates, stage):
                self.calls.append((stage, len(candidates)))
                return np.array([0.2, 0.8])
        teacher = FrozenTeacher()
        targets = teacher_targets(records, 0, teacher, mc_weight=0.5)
        expected = 0.5 * records[0]['value'] + 0.5 * (records[0]['rows'][0]['reward'] + 0.8)
        self.assertAlmostEqual(float(targets[0]), expected, places=6)
        self.assertEqual(teacher.calls, [(1, 2)])
        terminal = aggregate_stage([self.episodes[0]], 2)
        self.assertAlmostEqual(float(teacher_targets(terminal, 2, teacher)[0]), terminal[0]['value'], places=6)
        self.assertEqual(teacher.calls, [(1, 2)])

    def test_teacher_and_fit_never_receive_internal_holdout_rows(self):
        seen = []
        original = teacher_targets
        def inspect(records, stage, policy, mc_weight):
            heldout = set(self.report['config']['split']['holdout_tasks'])
            for row in records:
                task = int(round(float(row['state'][7]) / 0.05)) + 1
                self.assertNotIn('{"left":%d}' % task, heldout)
            if stage < 2:
                self.assertTrue(policy.models[stage + 1])
                self.assertTrue(all(not parameter.requires_grad for model in policy.models[stage + 1]
                                    for parameter in model.parameters()))
            seen.append(stage)
            return original(records, stage, policy, mc_weight)
        with patch('motion311_learning.supervised.teacher_targets', side_effect=inspect):
            fit_episodes(self.episodes, seed=19, widths=(8,), epochs=1, threads=4)
        self.assertEqual(seen, [2, 1, 0])

    def test_small_fit_is_finite_and_records_value_ranking_selection(self):
        self.assertEqual(self.policy.config['target_version'], TARGET_VERSION)
        self.assertEqual(self.policy.config['seeds'], [19, 20, 21])
        self.assertFalse(self.report['external_dev_used'])
        for stage in range(3):
            summary = self.report['stages'][str(stage)]
            self.assertGreater(summary['ranking_pairs'], 0)
            self.assertTrue(np.isfinite(summary['selection_loss']))
            for member in summary['trials'][0]['members']:
                self.assertGreaterEqual(member['best_epoch'], 0)
                self.assertTrue(all(np.isfinite(row['train_loss']) and np.isfinite(row['value_huber'])
                                    for row in member['curve']))
            self.assertTrue(all(parameter.device.type == 'cpu' for model in self.policy.models[stage]
                                for parameter in model.parameters()))

    def test_inference_is_current_only_and_order_equivariant(self):
        row = self.episodes[0]['transitions'][0]
        candidates = np.array([row['action_q'], self.episodes[1]['transitions'][0]['action_q']])
        first = self.policy.predict_values(row['state'], candidates, 0)
        second = self.policy.predict_values(row['state'], candidates[::-1].copy(), 0)
        np.testing.assert_allclose(second, first[::-1], rtol=1e-6, atol=1e-6)
        selected = self.policy.select(row['state'], candidates, 0)
        reversed_choice = self.policy.select(row['state'], candidates[::-1].copy(), 0)
        self.assertEqual(selected, 1 - reversed_choice)
        with self.assertRaises(TypeError):
            self.policy.select(row['state'], candidates, 0, next_candidates=candidates)
        with self.assertRaisesRegex(ValueError, 'empty'):
            self.policy.select(row['state'], np.empty((0, 15)), 0)

    def test_checkpoint_restores_predictions_and_curves(self):
        row = self.episodes[0]['transitions'][0]
        candidates = np.array([row['action_q'], self.episodes[1]['transitions'][0]['action_q']])
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'checkpoint.pt'
            self.policy.save(path, self.report)
            restored = SupervisedPolicy.load(path)
            self.assertEqual(restored.config, self.policy.config)
            np.testing.assert_array_equal(restored.predict(row['state'], candidates, 0),
                                          self.policy.predict(row['state'], candidates, 0))
            checkpoint = torch.load(path, weights_only=True, map_location='cpu')
            self.assertEqual(checkpoint['report'], self.report)

    def test_physical_dimension_is_schema_driven(self):
        policy, report = fit_episodes(self.episodes, physical=True, widths=(8,), epochs=2, seed=4, threads=4)
        row = self.episodes[0]['transitions'][1]
        candidates = np.concatenate([np.asarray([row['action_q']]), np.zeros((1, PHYSICS_DIM))], axis=1)
        output = policy.predict(row['state'], candidates, 1)
        self.assertEqual(output.shape, (1, 3))
        self.assertTrue(np.isfinite(output).all())
        self.assertEqual(policy.action_dim, 15 + PHYSICS_DIM)
        with self.assertRaisesRegex(ValueError, 'shape'):
            policy.predict(row['state'], candidates[:, :-1], 1)

    def test_cli_trains_default_grid_and_ignores_external_dev(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            source = root / 'source'
            source.mkdir()
            for index, episode in enumerate(self.episodes):
                item = dict(episode, schema=SCHEMA)
                (source / f'{index}.json').write_text(json.dumps(item))
            # Deliberately malformed dev labels must never be opened for fitting.
            (source / 'dev.json').write_text(json.dumps(dict(schema=SCHEMA, split='dev', transitions='invalid')))
            output = root / 'fit'
            with patch('builtins.print'):
                main(['--data', str(source), '--output', str(output), '--epochs', '1', '--threads', '4'])
            policy = SupervisedPolicy.load(output / 'checkpoint.pt')
            report = json.loads((output / 'training_report.json').read_text())
            self.assertEqual(len(policy.models), 3)
            for stage in range(3):
                self.assertEqual([trial['width'] for trial in report['stages'][str(stage)]['trials']], [32, 64])
                self.assertEqual(len(policy.models[stage]), 3)
            self.assertFalse(report['external_dev_used'])
            with self.assertRaises(FileExistsError):
                main(['--data', str(source), '--output', str(output), '--epochs', '1'])

    def test_fixed_seed_fit_is_reproducible(self):
        first, one = fit_episodes(self.episodes, seed=28, widths=(8,), epochs=2, threads=4)
        second, two = fit_episodes(self.episodes, seed=28, widths=(8,), epochs=2, threads=4)
        self.assertEqual(one, two)
        for a, b in zip(first.models, second.models):
            for model_a, model_b in zip(a, b):
                for parameter_a, parameter_b in zip(model_a.parameters(), model_b.parameters()):
                    torch.testing.assert_close(parameter_a, parameter_b, atol=0, rtol=0)


if __name__ == '__main__':
    unittest.main()
