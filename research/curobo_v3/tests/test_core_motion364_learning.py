from pathlib import Path
import json
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion311_learning.contracts import StepResult
from motion311_learning.dataset import normalized_actions, normalized_state
from motion311_learning.features import PHYSICS_DIM, STATE_DIM
from motion311_learning.networks import DQNAgent
from motion306_learning import train
from motion306_learning.agent import ACTION_IN, STATE_IN, LookaheadAgent, candidate_inputs, feasible, state_input
from motion306_learning.env import EXTRA_DIM, FAR_S, LookaheadObservation, carry_features, contact_features, place_features


def observation(stage=0, count=5, seed=0, valid=None):
    rng = np.random.default_rng(seed)
    state = np.zeros(STATE_DIM)
    state[stage] = 1
    state[7:37] = rng.normal(size=30) * .3
    rows = rng.normal(size=(count, 15)) * .5
    return LookaheadObservation(stage, state, rows, np.zeros((count, 3)), np.zeros((count, PHYSICS_DIM)),
                                np.repeat(np.eye(4)[None], 2, axis=0), extra=rng.uniform(0, 3, size=(count, EXTRA_DIM)),
                                valid=np.ones(count, bool) if valid is None else valid, dims=[.3, .4, .4])


class FeatureTest(unittest.TestCase):
    def test_contact_lookahead_uses_endpoints_and_shared_carry_pool(self):
        velocity = np.ones(15)
        rows = np.zeros((3, 15))
        rows[1, 0] = .5
        endpoints = [rows[0] + .1, None, rows[2] + .2]
        goals = np.stack([np.full(15, .3), np.full(15, .35), np.full(15, 2.)])
        features = contact_features(rows, endpoints, np.zeros(15), goals, velocity)
        np.testing.assert_allclose(features[:, 0], [0, .5, 0])
        np.testing.assert_allclose(features[:, 1], [.1, 0, .2])
        np.testing.assert_allclose(features[:, 2], [1, 0, 1])
        np.testing.assert_allclose(features[0, 3:], [.2, (.2 + .25 + 1.9) / 3, 2 / 3])
        np.testing.assert_allclose(features[1, 3:5], [FAR_S, FAR_S])
        empty = contact_features(rows, endpoints, np.zeros(15), np.empty((0, 15)), velocity)
        np.testing.assert_allclose(empty[:, 3:5], FAR_S)

    def test_carry_and_place_features(self):
        velocity = np.full(15, 2.)
        rows = np.zeros((2, 15))
        rows[0, 0], rows[1, 3] = -.4, 1.
        carry = carry_features(rows, np.zeros(15), np.full(15, .2), [None, 'carry_collision'], velocity)
        np.testing.assert_allclose(carry[:, :4], [[.2, 1, .3, .4], [.5, 0, .4, 0]])
        place = place_features(rows, np.zeros(15), np.zeros(15), velocity)
        np.testing.assert_allclose(place[:, :3], [[.2, .2, .4], [.5, .5, 1.]])

    def test_gated_pool_keeps_the_311_capping_rule_on_collision_free_rows(self):
        from motion306_learning.env import cap_ordered
        from motion311_learning.redundancy import RedundancyDecoder
        rows = np.arange(100 * 15, dtype=float).reshape(100, 15)
        actions = np.arange(300, dtype=float).reshape(100, 3)
        chosen, chosen_actions = cap_ordered(rows, actions, 64)
        self.assertEqual(len(chosen), 64)
        np.testing.assert_array_equal(chosen[:48], rows[:48])
        np.testing.assert_array_equal(chosen[48:], rows[48 + np.linspace(0, 51, 16, dtype=int)])
        np.testing.assert_array_equal(chosen_actions[:, 0] * 5, chosen[:, 0])
        small, _ = cap_ordered(rows[:10], actions[:10], 64)
        np.testing.assert_array_equal(small, rows[:10])
        # The decoder's own 64-row pool equals cap_ordered applied to its uncapped (score-ordered) pool.
        from types import SimpleNamespace
        decoder = RedundancyDecoder.__new__(RedundancyDecoder)
        decoder.lower, decoder.upper = np.full(15, -10.), np.full(15, 10.)
        decoder.carriage = SimpleNamespace(inverse=lambda u: np.eye(4)[None])
        solve = lambda index, pose, psi, seed: np.asarray([np.asarray(seed) + np.sin(psi * (index + 2)) * np.arange(1, 8) * .1])
        decoder.planner = SimpleNamespace(analytic=SimpleNamespace(solve=solve))
        targets, previous = {'left': np.eye(4), 'right': np.eye(4)}, np.zeros(15)
        capped = decoder.candidate_set(targets, previous, 64, 0)
        full = decoder.candidate_set(targets, previous, 10 ** 9, 0)
        self.assertEqual(len(full[0]), 21 * 36 * 36)
        for expected, actual in zip(capped, cap_ordered(*full, 64)):
            np.testing.assert_array_equal(expected, actual)

    def test_observation_validates_shapes(self):
        with self.assertRaises(ValueError):
            LookaheadObservation(0, np.zeros(STATE_DIM), np.zeros((2, 15)), np.zeros((2, 3)), np.zeros((2, PHYSICS_DIM)),
                                 np.repeat(np.eye(4)[None], 2, axis=0), extra=np.full((2, EXTRA_DIM), np.nan),
                                 valid=[True, True], dims=[.3, .4, .4])


class AgentTest(unittest.TestCase):
    def test_widening_311_preserves_q_and_order(self):
        old = DQNAgent(STATE_DIM, 15, seed=4)
        for stage in range(3):
            obs = observation(stage, 7, stage)
            expected = old.online[stage](torch.as_tensor(normalized_state(obs.state)).expand(7, -1),
                                         torch.as_tensor(normalized_actions(obs.candidates))).detach().numpy()
            agent = LookaheadAgent.from_311(old)
            q, probability = agent.scores(obs)
            np.testing.assert_allclose(q, expected, rtol=1e-5, atol=1e-6)
            self.assertTrue(((probability > .5) & (probability < 1)).all())
            self.assertEqual(agent.order(obs)[0], int(np.argmax(expected)))

    def test_residual_prior_starts_at_the_analytic_rankings(self):
        from motion306_learning.agent import prior_values
        from motion306_learning.rankers import heuristic_order, lookahead_heuristic_order
        velocity = np.linspace(1., 2., 15)
        old = DQNAgent(STATE_DIM, 15, seed=5)
        for kind, reference in (('heuristic', lambda obs: heuristic_order(obs, velocity)), ('lookahead', lookahead_heuristic_order)):
            agent = LookaheadAgent.from_311(old, residual=True, prior=kind, prior_weight=.01)
            for stage in range(3):
                base = observation(stage, 9, stage + 3)
                previous, home = base.state[7:22], base.state[22:37]
                rows = base.candidates
                if stage == 0:
                    rng = np.random.default_rng(stage)
                    endpoints = [row + rng.normal(size=15) * .1 for row in rows]
                    extra = contact_features(rows, endpoints, previous, rng.normal(size=(20, 15)), velocity)  # previous = Home at contact
                elif stage == 1:
                    extra = carry_features(rows, previous, home, [None] * len(rows), velocity)
                else:
                    extra = place_features(rows, previous, home, velocity)
                obs = LookaheadObservation(stage, base.state, rows, base.actions, base.physics, base.targets,
                                           extra=extra, valid=base.valid, dims=base.dims)
                np.testing.assert_array_equal(agent.order(obs), reference(obs))
                expected = -.01 * extra[:, list({'heuristic': ((0,), (0,), (0, 1)), 'lookahead': ((0, 1, 3), (0,), (0, 1))}[kind][stage])].sum(1)
                np.testing.assert_allclose(prior_values(stage, candidate_inputs(obs), kind, .01).numpy(), expected, rtol=1e-5, atol=1e-6)

    def test_veto_selection_keeps_the_prior_order_unless_a_candidate_is_clearly_riskier(self):
        old = DQNAgent(STATE_DIM, 15, seed=6)
        agent = LookaheadAgent.from_311(old, residual=True, prior='heuristic', prior_weight=.004, selection='veto', veto_margin=.2)
        base = observation(2, 6, 1)
        extra = np.zeros((6, EXTRA_DIM))
        extra[:, 0] = [3., 1., 2., 5., 4., 0.5]
        obs = LookaheadObservation(2, base.state, base.candidates, base.actions, base.physics, base.targets,
                                   extra=extra, valid=base.valid, dims=base.dims)
        self.assertEqual(agent.order(obs).tolist(), [5, 1, 2, 0, 4, 3])
        self.assertEqual(len(set(np.round(agent.scores(obs)[1], 9))), 1)  # residual init: constant success prediction
        with patch.object(LookaheadAgent, 'scores', return_value=(np.zeros(6), np.array([.9, .9, .9, .9, .9, .5]))):
            self.assertEqual(agent.order(obs).tolist(), [1, 2, 0, 4, 3, 5])
        with self.assertRaises(ValueError):
            LookaheadAgent(selection='veto')

    def test_episode_success_target_labels_every_transition(self):
        rows = [dict(stage=s, state=[0.] * STATE_IN, action=[0.] * ACTION_IN, reward=0., terminated=s == 1, truncated=False,
                     feasible=1., info=dict(cumulative_cost_s=1., success=False, class_='task_failure' if s else 'stage_success'),
                     next_state=None, next_candidates=None) for s in (0, 1)]
        record = dict(transitions=rows)
        self.assertEqual([row['feasible'] for row in train.learnable(record, success_target='stage')], [1., 1.])
        self.assertEqual([row['feasible'] for row in train.learnable(record, success_target='episode')], [0., 0.])

    def test_inputs_and_feasibility_labels(self):
        obs = observation(1, 4, valid=np.array([True, False, True, False]))
        self.assertEqual(state_input(obs).shape, (STATE_IN,))
        self.assertEqual(candidate_inputs(obs).shape, (4, ACTION_IN))
        self.assertEqual(candidate_inputs(obs, obs.valid).shape, (2, ACTION_IN))
        self.assertEqual([feasible(dict(class_=name)) for name in ('stage_success', 'success', 'task_failure')], [1., 1., 0.])

    def test_update_targets_only_next_gate_passing_pool_and_learns_feasibility(self):
        agent = LookaheadAgent(seed=1, lr=1e-2)
        rng = np.random.default_rng(0)
        row = lambda stage, ok: dict(stage=stage, state=rng.normal(size=STATE_IN).astype(np.float32),
                                     action=rng.normal(size=ACTION_IN).astype(np.float32), reward=-.01 if ok else -1.,
                                     terminated=not ok, feasible=float(ok),
                                     next_state=rng.normal(size=STATE_IN).astype(np.float32) if ok else None,
                                     next_candidates=rng.normal(size=(3, ACTION_IN)).astype(np.float32) if ok else None)
        batch = [row(0, True), row(0, False), row(1, True)]
        targets = agent.compute_targets(batch)
        self.assertAlmostEqual(float(targets[1]), -1.)
        dead_end = dict(batch[0], next_candidates=np.empty((0, ACTION_IN), np.float32))
        self.assertAlmostEqual(float(agent.compute_targets([dead_end])[0]), -1.01, places=6)
        inputs = torch.cat((torch.as_tensor(batch[0]['next_state']).expand(3, -1), torch.as_tensor(batch[0]['next_candidates'])), 1)
        q = agent.online[1](inputs)[:, 0]
        expected = -.01 + float(agent.target[1](inputs[int(q.argmax()):int(q.argmax()) + 1])[0, 0])
        self.assertAlmostEqual(float(targets[0]), expected, places=5)
        before = agent.online[0](torch.as_tensor(np.concatenate([batch[1]['state'], batch[1]['action']]))[None])[0, 1].item()
        for _ in range(30):
            agent.update(batch)
        after = agent.online[0](torch.as_tensor(np.concatenate([batch[1]['state'], batch[1]['action']]))[None])[0, 1].item()
        self.assertLess(after, before)

    def test_checkpoint_round_trip_with_training_state(self):
        agent = LookaheadAgent(seed=2)
        obs = observation(2)
        with tempfile.TemporaryDirectory() as folder:
            agent.save(Path(folder) / 'agent.pt', dict(last_committed_episode=3))
            loaded, state = LookaheadAgent.load(Path(folder) / 'agent.pt', with_training_state=True)
        np.testing.assert_array_equal(agent.scores(obs)[0], loaded.scores(obs)[0])
        self.assertEqual(state['last_committed_episode'], 3)


class StubLookaheadEnv:
    """Two-pool episode: contact 0 fails its gate, carry gate rejects index 0, placement succeeds."""

    def __init__(self, timed=True, physical=False, contact_pool='capped', place_gate=False):
        self.velocity = np.ones(15)
        self.resets = []

    def reset(self, item, seed):
        self.resets.append((item['instance'], seed))
        self.stage = 0
        self.observation = observation(0, 4, seed % 7)
        return self.observation

    def contact_checks(self, row):
        from motion311_learning.stage_env import EpisodeFailure
        if np.allclose(row, self.observation.candidates[0]):
            raise EpisodeFailure('contact_collision')
        return None, None

    def step(self, index):
        stage = self.stage
        info = dict(stage=stage, class_='stage_success', cumulative_cost_s=float(stage + 1), wall_ms=1.)
        if stage == 2:
            info.update(class_='success', success=True)
            return StepResult(None, .98, True, False, info)
        self.stage += 1
        self.observation = observation(self.stage, 3, index, valid=np.array([False, True, True]))
        self.gate_reasons = ['carry_collision', None, None] if self.stage == 1 else None
        return StepResult(self.observation, -.01, False, False, info)


def registry(folder):
    instances = []
    for level, split in (('S30x40x40', 'seen'), ('S35x38x42', 'heldout')):
        for task, top in (({'left': 1, 'right': 2}, False), ({'right': 3}, True)):
            for group in ('train', 'dev'):
                instances.append(dict(instance=f'{level}-{group}-{len(task)}', group=group, split=group, task=task,
                                      top_grasp=top, kind='dual' if len(task) == 2 else 'single', offset_m=[0, 0],
                                      box_level=level, box_dims=[.3, .4, .4], size_split=split))
    path = Path(folder) / 'registry.json'
    path.write_text(json.dumps(dict(size_levels={'S30x40x40': [.3, .4, .4], 'S35x38x42': [.35, .38, .42]},
                                    heldout_levels=['S35x38x42'], instances=instances)))
    return path


class TrainTest(unittest.TestCase):
    def test_training_items_refuse_heldout_levels_and_balance_modes(self):
        with tempfile.TemporaryDirectory() as folder:
            document = json.loads(registry(folder).read_text())
        groups = train.train_items(document, ['S30x40x40'])
        self.assertEqual(len(groups), 2)
        self.assertTrue(all(item['group'] == 'train' and item['box_level'] == 'S30x40x40'
                            for tasks in groups for instances in tasks for item in instances))
        self.assertEqual(train.sample_item(groups, np.random.default_rng(0))['box_level'], 'S30x40x40')
        with self.assertRaises(ValueError):
            train.train_items(document, ['S30x40x40', 'S35x38x42'])
        self.assertEqual(len(train.train_items(document, ['S30x40x40'], {'S30x40x40:single-right-top'})), 1)
        self.assertAlmostEqual(train.epsilon_at(0, 11, .3, .05), .3)
        self.assertAlmostEqual(train.epsilon_at(10, 11, .3, .05), .05)

    def test_shaped_rewards_match_environment_at_contract_weight(self):
        from motion311_learning.contracts import COST_WEIGHT, reward
        rows = [dict(terminated=False, info=dict(cumulative_cost_s=10.)), dict(terminated=False, info=dict(cumulative_cost_s=25.)),
                dict(terminated=True, info=dict(cumulative_cost_s=40., success=True))]
        expected = [reward(0., 10.), reward(10., 25.), reward(25., 40., True)]
        np.testing.assert_allclose(train.shaped_rewards(rows, COST_WEIGHT), expected)
        failed = rows[:1] + [dict(terminated=True, info=dict(cumulative_cost_s=10., success=False))]
        np.testing.assert_allclose(train.shaped_rewards(failed, 1.), [-(10 / 50), -1.])

    def test_episode_screens_records_valid_next_pool_and_resumes_without_rerun(self):
        with tempfile.TemporaryDirectory() as folder, patch('motion306_learning.env.LookaheadEnv', StubLookaheadEnv):
            path, output = registry(folder), Path(folder) / 'run'
            arguments = ['--registry', str(path), '--output', str(output), '--episodes', '13', '--updates', '1',
                         '--epsilon-start', '0', '--epsilon-end', '0']
            self.assertEqual(train.main(arguments), 0)
            record = json.loads((output / 'episodes' / 'e000001.json').read_text())
            first = record['transitions'][0]
            self.assertNotEqual(first['action_index'], 0)  # contact 0 fails its gate
            self.assertEqual(len(first['next_candidates']), 2)
            self.assertEqual([row['feasible'] for row in record['transitions']], [1., 1., 1.])
            self.assertEqual(record['transitions'][1]['screening']['rejected'], {'carry_collision': 1})
            agent, state = LookaheadAgent.load(output / 'agent.pt', with_training_state=True)
            self.assertEqual(state['last_committed_episode'], 12)
            self.assertGreater(state['gradient_updates'], 0)
            # Crash after episodes 1-2 were durable but before their updates were committed.
            (output / 'agent.pt').write_bytes((output / 'checkpoint-000000.pt').read_bytes())
            with patch('motion306_learning.env.LookaheadEnv', side_effect=AssertionError('must not rerun')):
                self.assertEqual(train.main(arguments), 0)
            resumed, resumed_state = LookaheadAgent.load(output / 'agent.pt', with_training_state=True)
            self.assertEqual(resumed_state, state)
            probe = observation(1, 6, 9)
            np.testing.assert_array_equal(resumed.scores(probe)[0], agent.scores(probe)[0])
            with self.assertRaises(RuntimeError):
                train.main(arguments[:5] + ['14'] + arguments[6:])

if __name__ == '__main__':
    unittest.main()
