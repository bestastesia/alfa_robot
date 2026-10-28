from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion311_learning.contracts import Observation, StepResult
from motion311_learning.evaluate import choose_action
from motion311_learning.features import PHYSICS_DIM, STATE_DIM
from motion311_learning.networks import DQNAgent
from motion311_learning.stage_env import EpisodeFailure
from motion306_learning.evaluate import build_report, evaluate_instance, run_evaluation
from motion306_learning.rankers import dqn_order, heuristic_order
from motion306_learning.screening import screened_choice


def item(identity='a', level='S30x40x40', size_split='seen'):
    return dict(instance=identity, split='dev', group='dev', task={'left': 1, 'right': 2}, top_grasp=False, kind='dual',
                offset_m=[0., 0.], box_level=level, box_dims=[.3, .4, .4], size_split=size_split)


def observation(stage, count=4):
    state = np.zeros(STATE_DIM)
    state[stage] = 1
    candidates = np.zeros((count, 15))
    candidates[:, 1] = np.arange(count) * .1
    return Observation(stage, state, candidates, np.zeros((count, 3)), np.zeros((count, PHYSICS_DIM)),
                       np.repeat(np.eye(4)[None], 2, axis=0))


class ScreenEnv:
    """Stage gates by candidate index: contact rows whose q1 matches `bad_contact`, carry reasons by index."""

    def __init__(self, bad_contact=(), bad_carry=(), fail_stage=None):
        self.bad_contact, self.bad_carry, self.fail_stage = set(bad_contact), set(bad_carry), fail_stage
        self.velocity, self.generation_ms, self.feature_ms = np.ones(15), 1., 0.
        self.steps, self.contact_calls = [], 0

    def reset(self, item, seed):
        self.stage, self.cost_s = 0, 0.
        self.report = dict(success=False, frames=[[0.] * 15], phases=['home'], payload=[False])
        self.observation = observation(0)
        return self.observation

    def contact_checks(self, row):
        self.contact_calls += 1
        if int(round(row[1] * 10)) in self.bad_contact:
            raise EpisodeFailure('extraction_invalid:right固定ψ解析无解@32.0cm')
        return np.zeros((2, 15)), None

    def carry_checks(self, rows):
        return ['carry_collision' if index in self.bad_carry else None for index in range(len(rows))]

    def step(self, index):
        stage = self.stage
        self.steps.append((stage, index))
        self.cost_s += 1
        info = dict(stage=stage, success=False, cumulative_cost_s=self.cost_s)
        gate = (stage == 0 and index in self.bad_contact) or (stage == 1 and index in self.bad_carry)
        if gate or self.fail_stage == stage:
            info.update(class_='task_failure', reason='gate' if gate else 'rrt')
            return StepResult(None, -1., True, False, info)
        if stage == 2:
            self.report.update(success=True, timed_validation={'success': True})
            info.update(success=True, class_='success')
            return StepResult(None, 1., True, False, info)
        self.stage += 1
        self.observation = observation(self.stage)
        return StepResult(self.observation, -.01, False, False, info)


class ScreeningTest(unittest.TestCase):
    def test_contact_screening_is_lazy_in_rank_order(self):
        env = ScreenEnv(bad_contact={2, 0})
        env.reset(item(), 0)
        chosen, record = screened_choice(env, np.array([2, 0, 3, 1]))
        self.assertEqual(chosen, 3)
        self.assertEqual(record['checks'], 3)
        self.assertEqual(record['rejected'], {'extraction_invalid': 2})
        env = ScreenEnv(bad_contact={0, 1, 2, 3})
        env.reset(item(), 0)
        self.assertIsNone(screened_choice(env, np.arange(4))[0])

    def test_batched_collision_pass_skips_per_row_checks(self):
        env = ScreenEnv(bad_contact={2})
        env.reset(item(), 0)
        env.contact_collision_free = lambda rows: np.array([False, True, True, True])
        chosen, record = screened_choice(env, np.array([0, 2, 1]))
        self.assertEqual((chosen, record['checks'], env.contact_calls), (1, 3, 2))
        self.assertEqual(record['rejected'], {'contact_collision': 1, 'extraction_invalid': 1})

    def test_carry_screening_batches_and_counts_valid(self):
        env = ScreenEnv(bad_carry={1, 3})
        env.reset(item(), 0)
        env.stage, env.observation = 1, observation(1)
        chosen, record = screened_choice(env, np.array([3, 1, 2, 0]))
        self.assertEqual((chosen, record['valid'], record['checks']), (2, 2, 4))
        self.assertEqual(record['rejected'], {'carry_collision': 2})

    def test_place_stage_uses_gate_reasons_only_when_exposed(self):
        env = ScreenEnv()
        env.reset(item(), 0)
        env.stage, env.observation = 2, observation(2)
        chosen, record = screened_choice(env, np.array([1, 0, 2, 3]))
        self.assertEqual((chosen, record['checks']), (1, 0))
        env.gate_reasons = ['place_collision', 'place_collision', None, None]
        chosen, record = screened_choice(env, np.array([1, 0, 3, 2]))
        self.assertEqual((chosen, record['valid'], record['rejected']), (3, 2, {'place_collision': 2}))

    def test_orderings_keep_the_311_single_choice_at_the_head(self):
        obs = observation(2, 6)
        state = obs.state.copy()
        state[7:37] = np.random.default_rng(1).normal(size=30)
        rows = np.random.default_rng(2).normal(size=(6, 15))
        obs = Observation(2, state, rows, np.zeros((6, 3)), np.zeros((6, PHYSICS_DIM)), obs.targets)
        velocity = np.linspace(1, 2, 15)
        expected, _ = choose_action('heuristic', None, obs, item(), velocity, np.random.default_rng(0))
        self.assertEqual(heuristic_order(obs, velocity)[0], expected)
        agent = DQNAgent(STATE_DIM, 15, seed=3)
        from motion311_learning.dataset import normalized_actions, normalized_state
        self.assertEqual(dqn_order(agent, obs)[0], agent.select(normalized_state(obs.state), normalized_actions(rows), 2))

    def test_lookahead_heuristic_adds_extraction_and_carry_at_contact_only(self):
        from motion306_learning.env import LookaheadObservation
        from motion306_learning.rankers import lookahead_heuristic_order
        base = observation(0, 3)
        extra = np.array([[1., 0., 1., 5., 0., 0.], [2., 0., 1., 1., 0., 0.], [1.5, 2., 1., 1., 0., 0.]])
        make = lambda stage: LookaheadObservation(stage, base.state, base.candidates, base.actions, base.physics, base.targets,
                                                  extra=extra, valid=np.ones(3, bool), dims=[.3, .4, .4])
        self.assertEqual(lookahead_heuristic_order(make(0)).tolist(), [1, 2, 0])
        self.assertEqual(lookahead_heuristic_order(make(1)).tolist(), [0, 2, 1])
        self.assertEqual(lookahead_heuristic_order(make(2)).tolist(), [0, 1, 2])

    def test_screened_episode_executes_one_valid_action_per_stage(self):
        env = ScreenEnv(bad_contact={0}, bad_carry={0, 1})
        row = evaluate_instance(env, item(), 'heuristic', seed=4)
        self.assertTrue(row['success'])
        self.assertEqual(env.steps, [(0, 1), (1, 2), (2, 0)])
        self.assertEqual([o['action_rank'] for o in row['outcomes']], [1, 2, 0])
        self.assertEqual(row['outcomes'][0]['screening']['rejected'], {'extraction_invalid': 1})
        self.assertGreaterEqual(row['selection_overhead_ms'], 3.)

    def test_exhausted_screening_executes_top_choice_and_fails_once(self):
        env = ScreenEnv(bad_contact={0, 1, 2, 3})
        row = evaluate_instance(env, item(), 'heuristic')
        self.assertEqual(env.steps, [(0, 0)])
        self.assertTrue(row['outcomes'][0]['screening_exhausted'])
        self.assertEqual((row['status'], row['failure_stage']), ('task_failure', 0))

    def test_unscreened_route_executes_the_head(self):
        env = ScreenEnv(bad_contact={0})
        row = evaluate_instance(env, item(), 'heuristic', screen=False)
        self.assertEqual(env.steps, [(0, 0)])
        self.assertEqual(env.contact_calls, 0)
        self.assertFalse(row['success'])

    def test_report_separates_sizes_routes_and_pairs_instances(self):
        rows = [evaluate_instance(ScreenEnv(bad_contact={0}), item('a'), 'heuristic'),
                evaluate_instance(ScreenEnv(bad_contact={0}), item('a'), 'heuristic', screen=False),
                evaluate_instance(ScreenEnv(fail_stage=1), item('b', 'S35x38x42', 'heldout'), 'heuristic')]
        report = build_report(rows)
        summaries = report['summaries']['dev']
        self.assertEqual(summaries['size_split/seen']['heuristic+screen']['success']['successes'], 1)
        self.assertEqual(summaries['size_split/heldout']['heuristic+screen']['success']['successes'], 0)
        self.assertEqual(summaries['all']['heuristic']['success']['successes'], 0)
        pair = report['paired']['dev']['all']['heuristic__heuristic+screen']
        self.assertEqual((pair['n_paired'], pair['b_only_success']), (1, 1))
        with self.assertRaises(ValueError):
            build_report(rows + rows[:1])

    def test_interleaved_routes_rotate_per_instance_and_report_by_label(self):
        import json as jsonlib
        from motion306_learning import interleave
        with tempfile.TemporaryDirectory() as folder:
            registry = Path(folder) / 'registry.json'
            registry.write_text(jsonlib.dumps(dict(instances=[item(name) for name in ('a', 'b', 'c')])))
            calls = []

            class Env(ScreenEnv):
                def __init__(self, **kwargs):
                    super().__init__()

                def reset(self, item_, seed):
                    calls.append(item_['instance'])
                    return super().reset(item_, seed)

            specs = ['h=heuristic', 'u=uniform']
            self.assertTrue(interleave.run(registry, Path(folder) / 'out', specs, env_factory=lambda **kwargs: Env()))
            self.assertEqual(calls, ['a', 'a', 'b', 'b', 'c', 'c'])
            orders = {}
            for path in (Path(folder) / 'out').glob('*/*/instances/*.json'):
                row = jsonlib.loads(path.read_text())
                orders[(row['instance'], row['label'])] = row['interleave_position']
            self.assertEqual([orders[('a', 'h')], orders[('b', 'u')], orders[('b', 'h')]], [0, 0, 1])
            report = jsonlib.loads((Path(folder) / 'out' / 'report.json').read_text())
            self.assertEqual(sorted(report['summaries']['dev']['all']), ['h', 'u'])
            self.assertTrue(interleave.run(registry, Path(folder) / 'out', specs, env_factory=lambda **kwargs: Env()))
            self.assertEqual(len(calls), 6)  # resumed: nothing reruns
            with self.assertRaises(ValueError):
                interleave.parse_routes(['h=heuristic', 'h=uniform'])

    def test_test_split_stays_sealed(self):
        with tempfile.TemporaryDirectory() as folder:
            with self.assertRaises(ValueError):
                run_evaluation(Path(folder) / 'missing.json', folder, 'heuristic', group='test')


if __name__ == '__main__':
    unittest.main()
