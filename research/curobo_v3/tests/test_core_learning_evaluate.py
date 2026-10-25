from pathlib import Path
import copy
import hashlib
import json
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion311_learning.contracts import Observation, StepResult
from motion311_learning.evaluate import (
    build_report, choose_action, compact_report, evaluate_instance,
    instance_seed, run_evaluation, wilson,
)
from motion311_learning.features import PHYSICS_DIM, STATE_DIM


def item(identity='a', split='dev', side='left'):
    return dict(instance=identity, split=split, task={side: 1}, top_grasp=False,
                kind='single', offset_m=[0.0, 0.0])


def observation(stage=0, continuous=False):
    state = np.zeros(STATE_DIM)
    state[stage] = 1
    state[3:7] = [0, 0, 1, 0]
    candidates = np.empty((0, 15)) if continuous else np.zeros((3, 15))
    if not continuous:
        candidates[:, 1] = [.1, .2, .3]
    return Observation(stage, state, candidates, np.zeros((len(candidates), 3)),
                       np.zeros((len(candidates), PHYSICS_DIM)), np.repeat(np.eye(4)[None], 2, axis=0))


class FakeFailure(Exception):
    reason = 'scene_home_collision'


class StubEnv:
    def __init__(self, timed=True, physical=False, fail_stage=None, infrastructure=False,
                 empty_initial=False, empty_stage2=False, scene_failure=False, accept_timing=True):
        self.timed, self.physical = timed, physical
        self.fail_stage, self.infrastructure = fail_stage, infrastructure
        self.empty_initial, self.empty_stage2 = empty_initial, empty_stage2
        self.scene_failure, self.accept_timing = scene_failure, accept_timing
        self.velocity = np.ones(15)
        self.calls = []
        self.reset_calls = 0
        self.generation_ms, self.feature_ms = 2.0, 3.0

    def reset(self, item, seed, continuous):
        self.reset_calls += 1
        self.continuous, self.stage, self.cost_s = continuous, 0, 0.0
        self.report = dict(success=False, frames=[[0.0] * 15], joint_names=[f'q{i}' for i in range(15)],
                           phases=['home'], payload=[False])
        if self.scene_failure:
            raise FakeFailure()
        return observation(0, continuous or self.empty_initial)

    def step(self, action):
        self.calls.append(action.copy() if isinstance(action, np.ndarray) else action)
        stage = self.stage
        self.cost_s += 1
        self.report['frames'].append([float(stage + 1)] * 15)
        self.report['phases'].append(('extract', 'transport', 'return_home')[stage])
        self.report['payload'].append(stage != 2)
        info = dict(stage=stage, success=False, cumulative_cost_s=self.cost_s,
                    feature_ms=3.0, decode_ms=1.0 if self.continuous else 0.0)
        if self.fail_stage == stage:
            if self.infrastructure:
                info.update(class_='infrastructure', reason='cuda_oom')
                return StepResult(None, 0.0, False, True, info)
            info.update(class_='task_failure', reason='collision')
            return StepResult(None, -1.0, True, False, info)
        if self.empty_stage2 and stage == 1:
            info['class_'] = 'empty_next_candidates'
            return StepResult(None, -1.0, True, False, info)
        if stage == 2:
            self.report.update(success=True, timed_validation={'success': self.accept_timing}, motion_duration_s=5.0,
                               timed_segments=[dict(name='transport', positions=[[0.0] * 15] * 10,
                                                    velocities=[[0.0] * 15] * 10, duration_s=5.0)])
            info.update(success=True, class_='success', executed_t_path_s=2.5, motion_duration_s=5.0)
            return StepResult(None, 1.0, True, False, info)
        self.stage += 1
        return StepResult(observation(self.stage, self.continuous), -0.01, False, False, info)


class StubSAC:
    def __init__(self):
        self.calls = []
    def select(self, state, stage, deterministic=False):
        self.calls.append((stage, deterministic, state.copy()))
        return np.array([.2, .3, .4])
    def update(self, batch):
        raise AssertionError('evaluation must not update')


class StrictEvaluationTest(unittest.TestCase):
    def test_heuristic_current_movement_and_last_home_cost(self):
        obs = observation(2)
        state = obs.state.copy()
        state[7:22] = 0
        state[7] = .8
        state[22:37] = 0
        candidates = np.zeros((2, 15))
        candidates[0, :2] = [.8, .8]
        candidates[1, :2] = [.3, .0]
        obs = Observation(2, state, candidates, np.zeros((2, 3)), np.zeros((2, PHYSICS_DIM)), obs.targets)
        index, timing = choose_action('heuristic', None, obs, item(), np.ones(15), np.random.default_rng(0))
        self.assertEqual(index, 1)
        self.assertGreaterEqual(timing['inference_ms'], 0)

    def test_strict_success_preserves_costs_rewards_hash_and_no_large_segments(self):
        env = StubEnv()
        result = evaluate_instance(env, item(), 'heuristic', seed=3)
        self.assertTrue(result['success'])
        self.assertEqual(result['status'], 'success')
        self.assertEqual(len(env.calls), 3)
        self.assertEqual(result['raw_t_path_s'], 3.0)
        self.assertEqual(result['executed_t_path_s'], 2.5)
        self.assertEqual(result['motion_duration_s'], 5.0)
        self.assertAlmostEqual(result['episode_return'], .98)
        self.assertTrue(result['third_stage_reached'])
        self.assertFalse(result['third_stage_failed'])
        expected = hashlib.sha256(np.asarray(env.report['frames'], dtype='<f8').tobytes()).hexdigest()
        self.assertEqual(result['report']['frames_sha256'], expected)
        self.assertNotIn('timed_segments', result['report'])
        self.assertNotIn('positions', result['report']['segment_summary'][0])
        self.assertEqual(result['outcomes'][0]['feature_ms'], 3.0)

    def test_failure_never_retries_action_or_later_stage(self):
        env = StubEnv(fail_stage=1)
        result = evaluate_instance(env, item(), 'heuristic')
        self.assertFalse(result['success'])
        self.assertEqual(len(env.calls), 2)
        self.assertEqual(result['failure_stage'], 1)
        self.assertEqual(result['status'], 'task_failure')
        self.assertFalse(result['third_stage_reached'])

    def test_timing_gate_required_for_claimed_success(self):
        result = evaluate_instance(StubEnv(accept_timing=False), item(), 'heuristic')
        self.assertFalse(result['success'])
        self.assertTrue(result['third_stage_failed'])
        self.assertEqual(result['failure_stage'], 2)

    def test_empty_initial_and_scene_failures_are_not_discarded(self):
        empty = evaluate_instance(StubEnv(empty_initial=True), item('empty'), 'heuristic')
        scene = evaluate_instance(StubEnv(scene_failure=True), item('scene'), 'heuristic')
        self.assertEqual(empty['status'], 'empty_initial_candidates')
        self.assertEqual(empty['rewards'], [-1.0])
        self.assertEqual(scene['status'], 'scene_infeasible')
        self.assertEqual(scene['rewards'], [-1.0])
        self.assertEqual(len(empty['outcomes']), 0)
        self.assertEqual(len(scene['outcomes']), 0)
        summary = build_report([empty, scene])['summaries']['dev']['all']['heuristic']
        self.assertEqual(summary['success']['total'], 2)
        self.assertEqual(summary['statuses']['scene_infeasible'], 1)

    def test_continuous_empty_pool_is_not_terminal_and_idle_is_fixed(self):
        for side, inactive in (('left', 2), ('right', 1)):
            env, policy = StubEnv(), StubSAC()
            result = evaluate_instance(env, item(side=side), 'r4', policy)
            self.assertTrue(result['success'])
            self.assertEqual(len(env.calls), 3)
            self.assertTrue(all(action[inactive] == 0 for action in env.calls))
            self.assertTrue(all(call[1] for call in policy.calls))
            self.assertTrue(all(row['candidate_count'] == 0 for row in result['outcomes']))

    def test_uniform_samples_once_and_reproduces_independent_of_order(self):
        first, second = StubEnv(fail_stage=0), StubEnv(fail_stage=0)
        a = evaluate_instance(first, item(), 'uniform', seed=5)
        b = evaluate_instance(second, item(), 'uniform', seed=5)
        self.assertEqual(len(first.calls), 1)
        np.testing.assert_array_equal(first.calls[0], second.calls[0])
        self.assertTrue((np.abs(first.calls[0]) <= 1).all())
        self.assertEqual(a['planner_seed'], b['planner_seed'])
        self.assertNotEqual(instance_seed(item('other'), 5), instance_seed(item(), 5))

    def test_infrastructure_preserved_and_third_stage_denominators_explicit(self):
        rows = [evaluate_instance(StubEnv(fail_stage=0), item('early'), 'heuristic'),
                evaluate_instance(StubEnv(fail_stage=2), item('last'), 'heuristic'),
                evaluate_instance(StubEnv(empty_stage2=True), item('empty'), 'heuristic'),
                evaluate_instance(StubEnv(fail_stage=2, infrastructure=True), item('oom'), 'heuristic'),
                evaluate_instance(StubEnv(), item('success'), 'heuristic')]
        summary = build_report(rows)['summaries']['dev']['all']['heuristic']
        self.assertEqual(summary['success']['total'], 5)
        self.assertEqual(summary['success']['successes'], 1)
        self.assertEqual(summary['third_stage_reached'], 4)
        self.assertEqual(summary['third_stage_failed'], 2)
        self.assertEqual(summary['third_stage_failure_given_reached']['rate'], .5)
        self.assertEqual(summary['third_stage_failure_per_episode']['rate'], .4)
        self.assertEqual(summary['third_stage_infrastructure'], 1)
        self.assertEqual(summary['statuses']['infrastructure'], 1)

    def test_paired_differences_use_common_instances_and_success_only_costs(self):
        left_a = evaluate_instance(StubEnv(), item('a'), 'heuristic')
        left_b = evaluate_instance(StubEnv(fail_stage=1), item('b'), 'heuristic')
        right_a = evaluate_instance(StubEnv(), item('a'), 'uniform')
        right_a['executed_t_path_s'] = 1.5
        right_b = evaluate_instance(StubEnv(), item('b'), 'uniform')
        right_c = evaluate_instance(StubEnv(), item('c'), 'uniform')
        report = build_report([left_a, left_b, right_a, right_b, right_c])
        pair = report['paired']['dev']['all']['heuristic__uniform']
        self.assertEqual(pair['n_paired'], 2)
        self.assertEqual(pair['n_b_only'], 1)
        self.assertEqual(pair['success_delta_b_minus_a'], .5)
        self.assertEqual(pair['paired_success_costs']['executed_t_path_s']['n_both_success'], 1)
        self.assertEqual(pair['paired_success_costs']['executed_t_path_s']['mean_b_minus_a'], -1.0)
        with self.assertRaisesRegex(ValueError, 'duplicate'):
            build_report([left_a, copy.deepcopy(left_a)])

    def test_wilson_zero_and_known_interval(self):
        self.assertIsNone(wilson(0, 0)['rate'])
        interval = wilson(5, 10)
        self.assertAlmostEqual(interval['low'], .236593, places=5)
        self.assertAlmostEqual(interval['high'], .763407, places=5)
        self.assertGreater(wilson(0, 10)['high'], 0)

    def test_resume_and_test_gate_do_not_construct_or_rerun_environment(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            registry, output = root / 'registry.json', root / 'results'
            registry.write_text(json.dumps({'instances': [item('a'), item('b'), item('test', 'test')]}))
            constructed = []
            def factory(**kwargs):
                env = StubEnv(**kwargs)
                constructed.append(env)
                return env
            with self.assertRaisesRegex(ValueError, '--allow-test'):
                run_evaluation(registry, output, 'uniform', group='test', env_factory=factory)
            self.assertEqual(constructed, [])
            with patch('builtins.print'):
                self.assertTrue(run_evaluation(registry, output, 'uniform', env_factory=factory))
                self.assertEqual(len(constructed), 1)
                self.assertEqual(constructed[0].reset_calls, 2)
                self.assertTrue(run_evaluation(registry, output, 'uniform', env_factory=factory))
            self.assertEqual(len(constructed), 1)
            paths = list(output.glob('*/*/instances/*.json'))
            self.assertEqual(len(paths), 2)
            self.assertTrue((output / 'report.json').exists())
            self.assertFalse(list(output.rglob('*.tmp')))
            registry.write_text(json.dumps({'instances': [item('a'), item('b'), item('extra')]}))
            with self.assertRaisesRegex(ValueError, 'resume configuration'):
                run_evaluation(registry, output, 'uniform', env_factory=factory)

    def test_environment_initialization_failures_are_persisted_without_retries(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            registry, output = root / 'registry.json', root / 'results'
            registry.write_text(json.dumps({'instances': [item('a'), item('b')]}))
            with patch('builtins.print'), patch(__name__ + '.StubEnv', side_effect=RuntimeError('missing runtime')) as factory:
                self.assertTrue(run_evaluation(registry, output, 'uniform', env_factory=factory))
                self.assertEqual(factory.call_count, 1)
            records = [json.loads(path.read_text()) for path in output.glob('*/*/instances/*.json')]
            self.assertEqual(len(records), 2)
            self.assertTrue(all(row['status'] == 'infrastructure' for row in records))
            self.assertTrue(all(row['provenance']['source_sha256'] for row in records))

    def test_report_rejects_pooling_changed_checkpoints(self):
        first = evaluate_instance(StubEnv(), item('a'), 'heuristic')
        second = evaluate_instance(StubEnv(), item('b'), 'heuristic')
        first['provenance'] = dict(registry_sha256='registry', checkpoint_sha256='one')
        second['provenance'] = dict(registry_sha256='registry', checkpoint_sha256='two')
        with self.assertRaisesRegex(ValueError, 'provenance'):
            build_report([first, second])

    def test_compact_report_hash_changes_with_frame_and_drops_large_arrays(self):
        report = dict(frames=[[0, 1]], phases=['home'], payload=[False], velocities=[[0, 0]],
                      accelerations=[[0, 0]], jerks=[[0, 0]], timed_segments=[])
        before = compact_report(report)
        report['frames'][0][1] = 2
        after = compact_report(report)
        self.assertNotEqual(before['frames_sha256'], after['frames_sha256'])
        self.assertNotEqual(before['full_report_sha256'], after['full_report_sha256'])
        self.assertNotIn('frames', before)
        self.assertNotIn('velocities', before)


if __name__ == '__main__':
    unittest.main()
