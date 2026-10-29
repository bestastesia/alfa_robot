from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion311_learning.contracts import StepResult
from motion311_learning.features import PHYSICS_DIM, STATE_DIM
from motion311_learning.stage_env import EpisodeFailure
from motion306_learning.env import LookaheadObservation
from motion306_learning.oracle import (best_leaf, branches, fixed_choice, follow, heuristic_choice, passing_rows,
                                       path_summary, run_policy, search, search_instance)


def item():
    return dict(instance='a', split='dev', group='dev', task={'left': 1, 'right': 2}, top_grasp=False, kind='dual',
                offset_m=[0., 0.], box_level='S30x40x40', size_split='seen')


def observation(stage, count=4, lookahead_first=None):
    """Heuristic order is ascending index (row[1] = 0.1 i from a zero state); the lookahead order is descending
    unless `lookahead_first` names its head."""
    state = np.zeros(STATE_DIM)
    state[stage] = 1
    candidates = np.zeros((count, 15))
    candidates[:, 1] = np.arange(count) * .1
    extra = np.zeros((count, 6))
    extra[:, 0] = np.arange(count)[::-1] + 1.
    if lookahead_first is not None:
        extra[lookahead_first, 0] = 0.
    return LookaheadObservation(stage, state, candidates, np.zeros((count, 3)), np.zeros((count, PHYSICS_DIM)),
                                np.repeat(np.eye(4)[None], 2, axis=0), extra=extra, valid=np.ones(count, bool),
                                dims=[.3, .4, .4])


class TreeEnv:
    """Deterministic three-stage tree: a step's outcome and duration depend on its path only."""

    def __init__(self, count=4, fail=(), collide=(), bad_contact=(), carry_reasons=None, duration=None):
        self.count, self.fail, self.collide, self.bad_contact = count, set(fail), set(collide), set(bad_contact)
        self.carry_reasons = carry_reasons
        self.duration = duration or (lambda path: 1. + path[-1])
        self.velocity, self.timed, self.seeds = np.ones(15), True, []

    def _observe(self):
        self.observation = observation(self.stage, self.count)
        self.gate_reasons = None if self.stage == 0 else (self.carry_reasons if self.stage == 1 and self.carry_reasons
                                                         else [None] * self.count)
        return self.observation

    def reset(self, item, seed):
        self.seeds.append(seed)
        self.stage, self.path, self.timed_blocks, self.cost = 0, [], [], 0.
        return self._observe()

    def checkpoint(self):
        return dict(stage=self.stage, path=list(self.path), blocks=list(self.timed_blocks), cost=self.cost)

    def restore(self, state):
        self.stage, self.path, self.timed_blocks, self.cost = state['stage'], list(state['path']), list(state['blocks']), state['cost']
        return self._observe()

    def contact_collision_free(self, rows):
        return np.array([index not in self.collide for index in range(len(rows))])

    def contact_checks(self, row):
        if int(round(row[1] * 10)) in self.bad_contact:
            raise EpisodeFailure('extraction_invalid:x')
        return np.zeros((2, 15)), None

    def step(self, index):
        path = self.path + [int(index)]
        info = dict(stage=self.stage, success=False, wall_ms=1.)
        if tuple(path) in self.fail:
            info.update(class_='task_failure', reason='rrt', cumulative_cost_s=self.cost)
            return StepResult(None, -1., True, False, info)
        duration = self.duration(path)
        self.timed_blocks.append(dict(motion_duration_s=duration))
        self.cost += duration
        if self.stage == 2:
            info.update(success=True, class_='success', stage_cost_s=duration, cumulative_cost_s=self.cost,
                        motion_duration_s=sum(block['motion_duration_s'] for block in self.timed_blocks))
            return StepResult(None, 1., True, False, info)
        self.stage, self.path = self.stage + 1, path
        info.update(class_='stage_success', stage_cost_s=duration, cumulative_cost_s=self.cost)
        return StepResult(self._observe(), 0., False, False, info)


def contact_dependent(path):
    """Stage 2 is expensive after contact 0, so the heuristic's local choices are not the best triple."""
    if len(path) < 3:
        return 1. + path[-1]
    return (10. if path[0] == 0 else 1.) + .1 * path[-1]


class OracleTest(unittest.TestCase):
    def test_branches_put_heuristic_and_lookahead_first_then_local_then_random(self):
        obs = observation(0, 8)
        picks = branches(obs, list(range(8)), np.ones(15), 5, np.random.default_rng(0))
        self.assertEqual(picks[:4], [(0, ['heuristic']), (7, ['lookahead']), (1, ['local']), (2, ['local'])])
        self.assertEqual(len(picks), 5)
        self.assertEqual(picks[4][1], ['random'])
        self.assertIn(picks[4][0], {3, 4, 5, 6})
        self.assertEqual(picks, branches(obs, list(range(8)), np.ones(15), 5, np.random.default_rng(0)))
        shared = branches(observation(0, 8, lookahead_first=1), [1, 2, 3, 5], np.ones(15), 3, np.random.default_rng(0))
        self.assertEqual(shared[0], (1, ['heuristic', 'lookahead']))
        self.assertTrue(all(index in {1, 2, 3, 5} for index, _ in shared))
        self.assertEqual(branches(obs, [], np.ones(15), 3, np.random.default_rng(0)), [])
        self.assertEqual(branches(obs, list(range(8)), np.ones(15), 1, np.random.default_rng(0)), [(0, ['heuristic'])])

    def test_passing_rows_use_the_contact_and_carry_gates(self):
        env = TreeEnv(count=5, collide={1}, bad_contact={2}, carry_reasons=[None, 'carry_collision', None, None, 'carry_envelope'])
        self.assertEqual(passing_rows(env, env.reset(item(), 0)), [0, 3, 4])
        env.stage = 1
        self.assertEqual(passing_rows(env, env._observe()), [0, 2, 3])

    def test_search_keeps_the_heuristic_path_and_finds_the_best_leaf(self):
        env = TreeEnv(fail={(0, 3)}, duration=contact_dependent)
        env.reset(item(), 7)
        edges, nodes = search(env, env.checkpoint(), (2, 2, 2), env.velocity, 7)
        self.assertEqual(len(edges), 2 + 4 + 6)
        self.assertEqual(len(nodes), 1 + 2 + 3)
        self.assertEqual(follow(edges, 'heuristic')['path'], [0, 0, 0])
        self.assertEqual(follow(edges, 'lookahead')['path'], [3, 3, 3])
        best = best_leaf(edges)
        self.assertEqual((best['path'], best['motion_duration_s']), ([3, 0, 0], 6.))
        summary = path_summary(edges, best)
        np.testing.assert_allclose([row[1] for row in summary['rows']], [.3, 0., 0.])
        self.assertEqual(summary['roles'], [['lookahead'], ['heuristic'], ['heuristic']])
        heuristic = path_summary(edges, follow(edges, 'heuristic'))
        self.assertEqual((heuristic['success'], heuristic['motion_duration_s']), (True, 12.))

    def test_a_failed_heuristic_step_ends_its_path_but_not_the_search(self):
        env = TreeEnv(fail={(0, 0)}, duration=contact_dependent)
        env.reset(item(), 7)
        edges, _ = search(env, env.checkpoint(), (2, 2, 2), env.velocity, 7)
        heuristic = path_summary(edges, follow(edges, 'heuristic'))
        self.assertEqual((heuristic['success'], heuristic['stage'], heuristic['reason']), (False, 1, 'rrt'))
        self.assertEqual(best_leaf(edges)['path'], [3, 0, 0])
        self.assertTrue(all(edge['path'][:2] != [0, 0] for edge in edges if edge['stage'] == 2))

    def test_policies_replay_heuristic_and_fixed_paths_without_remapping(self):
        env = TreeEnv(duration=contact_dependent)
        outcome = run_policy(env, item(), 11, heuristic_choice)
        self.assertEqual((outcome['path'], outcome['success'], outcome['motion_duration_s']), ([0, 0, 0], True, 12.))
        summary = dict(path=[3, 0, 0], rows=[observation(stage).candidates[index].tolist() for stage, index in enumerate([3, 0, 0])])
        self.assertTrue(run_policy(env, item(), 12, fixed_choice(summary))['success'])
        summary['rows'][1] = (np.asarray(summary['rows'][1]) + 1.).tolist()
        outcome = run_policy(env, item(), 13, fixed_choice(summary))
        self.assertEqual((outcome['success'], outcome['stage'], outcome['reason']), (False, 1, 'no_choice'))
        self.assertEqual(env.seeds, [11, 12, 13])

    def test_search_instance_records_tree_summaries_and_replicates(self):
        env = TreeEnv(fail={(0, 3)}, duration=contact_dependent)
        record = search_instance(env, item(), 101, (2, 2, 2), [102, 103])
        self.assertEqual(record['status'], 'searched')
        self.assertEqual((record['steps'], record['leaves']), (12, 6))
        self.assertEqual((record['heuristic']['path'], record['best']['path']), ([0, 0, 0], [3, 0, 0]))
        self.assertEqual([entry['seed'] for entry in record['replicates']], [102, 103])
        self.assertTrue(all(entry['best']['success'] and entry['heuristic']['success'] for entry in record['replicates']))
        self.assertEqual(len(set(env.seeds)), 3)


if __name__ == '__main__':
    unittest.main()
