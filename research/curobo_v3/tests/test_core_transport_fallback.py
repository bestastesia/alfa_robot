import sys
from pathlib import Path
import unittest
from unittest import mock

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import v3_cartesian_transport_planner as planner_module
from v3_cartesian_transport_planner import TRANSPORT_BUDGET_S, CartesianTransportPlanner


def row(branch_signs):
    """A 15-joint row whose right arm has the wrist branch sign(q2, q4, q6) given."""
    values = np.zeros(15)
    values[9], values[11], values[13] = [.5 * sign for sign in branch_signs]
    return values


def stub_planner(**options):
    planner = object.__new__(CartesianTransportPlanner)
    planner.transport_search_mode = 'cartesian'
    planner.transport_timing = 'toppra'
    planner.transport_seed = 7
    planner.cartesian_options = options
    planner.selected_task = {'right': 20}
    planner.transport_joint_path_used = False
    planner.joint_calls = []

    def single_path(start, goals, checker, rotation=False, strict=None, budget=2., seed=20261015):
        planner.joint_calls.append(dict(budget=budget, seed=seed))
        return np.zeros((3, 15)), dict(iterations=1)

    planner.single_path = single_path
    return planner


class TransportFallbackTest(unittest.TestCase):
    def setUp(self):
        self.start = row((1, -1, 1))
        self.same = np.stack([row((1, -1, 1)), row((-1, -1, 1))])
        self.other = self.same[::-1]

    def cartesian(self, path):
        return mock.patch.object(planner_module, 'plan_cartesian_transport',
                                 return_value=(path, dict(failure=None if path is not None else 'no_valid_path')))

    def test_defaults_keep_the_v2_behaviour(self):
        planner = stub_planner()
        with self.cartesian(None) as search:
            path, stats = planner.transport_path(self.start, self.other, None, self.other)
        self.assertIsNone(path)
        self.assertEqual(search.call_count, 1)
        self.assertEqual(planner.joint_calls, [])
        self.assertFalse(planner.transport_joint_path_used)
        self.assertEqual(stats['failure'], 'no_valid_path')

    def test_fallback_gets_the_rest_of_the_budget(self):
        planner = stub_planner(fallback_joint_rrt=True)
        with self.cartesian(None):
            path, stats = planner.transport_path(self.start, self.same, None, self.same)
        self.assertIsNotNone(path)
        self.assertEqual(len(planner.joint_calls), 1)
        self.assertLess(planner.joint_calls[0]['budget'], TRANSPORT_BUDGET_S)
        self.assertGreater(planner.joint_calls[0]['budget'], TRANSPORT_BUDGET_S - .5)
        self.assertEqual(planner.joint_calls[0]['seed'], 7)
        self.assertEqual(stats['tier'], 'joint_rrt_fallback')
        self.assertEqual(stats['cartesian_failure'], 'no_valid_path')
        self.assertIsNone(stats['failure'])
        self.assertTrue(planner.transport_joint_path_used)

    def test_fallback_is_not_used_when_the_cartesian_search_succeeds(self):
        planner = stub_planner(fallback_joint_rrt=True, route_branch_change=True)
        with self.cartesian(np.ones((4, 15))):
            path, _ = planner.transport_path(self.start, self.same, None, self.same)
        self.assertEqual(path.shape, (4, 15))
        self.assertEqual(planner.joint_calls, [])
        self.assertFalse(planner.transport_joint_path_used)

    def test_branch_change_routes_to_the_joint_rrt(self):
        planner = stub_planner(route_branch_change=True)
        with self.cartesian(np.ones((4, 15))) as search:
            path, stats = planner.transport_path(self.start, self.other, None, self.other)
        self.assertEqual(search.call_count, 0)
        self.assertEqual(planner.joint_calls, [dict(budget=2., seed=7)])
        self.assertEqual(stats['tier'], 'joint_rrt_routed')
        self.assertTrue(planner.transport_joint_path_used)

    def test_joint_path_is_shortcut_like_the_baseline(self):
        planner = stub_planner()
        rows = np.zeros((5, 15))
        with mock.patch.object(planner_module.TimedSingleArmPlanner, '_shortcut_phase',
                               return_value=(rows[[0, -1]], {})) as baseline:
            kept, info = planner._shortcut_phase('transport', rows, None, None)
            self.assertEqual(len(kept), 5)
            self.assertIn('shortcut_skipped', info)
            planner.transport_joint_path_used = True
            kept, _ = planner._shortcut_phase('transport', rows, None, None)
        self.assertEqual(len(kept), 2)
        self.assertEqual(baseline.call_count, 1)


class WristFlipRoutingTest(unittest.TestCase):
    def setUp(self):
        self.start = row((1, -1, 1))
        self.wrist = np.stack([row((1, -1, -1))])      # only the wrist branch differs
        self.elbow = np.stack([row((1, 1, -1))])       # elbow and wrist differ

    def test_wrist_only_change_tries_the_wrist_flip_first(self):
        planner = stub_planner(route_branch_change=True, wrist_flip=True)
        flip = (np.ones((6, 15)), dict(swing_indices=[2, 4], search_ms=12.))
        with mock.patch.object(planner_module, 'plan_wrist_flip', return_value=flip) as search, \
                mock.patch.object(planner_module.cartesian, 'BatchAnalyticIk'):
            path, stats = planner.transport_path(self.start, self.wrist, None, self.wrist)
        self.assertEqual(search.call_count, 1)
        self.assertEqual(path.shape, (6, 15))
        self.assertEqual(stats['tier'], 'wrist_flip')
        self.assertEqual(planner.last_wrist_flip, dict(rows=6, swing=[2, 4]))
        self.assertEqual(planner.joint_calls, [])
        self.assertFalse(planner.transport_joint_path_used)

    def test_failed_wrist_flip_falls_back_to_the_joint_rrt_within_the_budget(self):
        planner = stub_planner(route_branch_change=True, wrist_flip=True)
        with mock.patch.object(planner_module, 'plan_wrist_flip', return_value=(None, dict(failure='no_wrist_flip_path'))), \
                mock.patch.object(planner_module.cartesian, 'BatchAnalyticIk'):
            path, stats = planner.transport_path(self.start, self.wrist, None, self.wrist)
        self.assertIsNotNone(path)
        self.assertEqual(stats['tier'], 'joint_rrt_routed')
        self.assertEqual(stats['wrist_flip']['failure'], 'no_wrist_flip_path')
        self.assertLess(planner.joint_calls[0]['budget'], TRANSPORT_BUDGET_S)
        self.assertIsNone(planner.last_wrist_flip)
        self.assertTrue(planner.transport_joint_path_used)

    def test_wrist_flip_search_is_capped_and_leaves_the_rest_to_the_joint_rrt(self):
        planner = stub_planner(route_branch_change=True, wrist_flip=True, wrist_flip_budget_s=.4)
        deadlines = []

        def search(planner_, start, candidates, checker, ik, options, deadline):
            deadlines.append((deadline - planner_module.time.perf_counter(), options['wrist_flip_budget_s']))
            return None, dict(failure='wrist_flip_deadline')

        with mock.patch.object(planner_module, 'plan_wrist_flip', side_effect=search), \
                mock.patch.object(planner_module.cartesian, 'BatchAnalyticIk'):
            planner.transport_path(self.start, self.wrist, None, self.wrist)
        self.assertLessEqual(deadlines[0][0], .4)
        self.assertEqual(deadlines[0][1], .4)
        self.assertGreater(planner.joint_calls[0]['budget'], TRANSPORT_BUDGET_S - .4 - .05)

    def test_other_branch_changes_are_routed_without_a_wrist_flip(self):
        planner = stub_planner(route_branch_change=True, wrist_flip=True)
        with mock.patch.object(planner_module, 'plan_wrist_flip') as search:
            _, stats = planner.transport_path(self.start, self.elbow, None, self.elbow)
        self.assertEqual(search.call_count, 0)
        self.assertEqual(stats['tier'], 'joint_rrt_routed')
        self.assertEqual(planner.joint_calls, [dict(budget=2., seed=7)])


class ToppraPiecesTest(unittest.TestCase):
    def test_pieces_stop_at_their_joins_and_add_up(self):
        from v3_path_timing import time_parameterize_toppra, time_parameterize_toppra_pieces

        rows = np.column_stack([np.linspace(0., 1., 31), np.r_[np.linspace(0., .5, 16), np.linspace(.5, .5, 16)[1:]]])
        limits = (np.full(2, 1.), np.full(2, 2.), np.full(2, 20.))
        pieces = [rows[:16], rows[15:]]
        joined, validation = time_parameterize_toppra_pieces(pieces, ['a', 'b'], *limits)
        parts = [time_parameterize_toppra(piece, ['a', 'b'], *limits)[0] for piece in pieces]
        self.assertAlmostEqual(joined['duration_s'], sum(part['duration_s'] for part in parts))
        times = np.asarray(joined['time_from_start_s'])
        self.assertTrue(np.all(np.diff(times) > 0))
        self.assertEqual(len(joined['positions']), len(rows))
        join = 15
        self.assertLess(np.abs(np.asarray(joined['velocities'])[join]).max(), 1e-6)
        sampled = np.asarray(joined['sampled_trajectory']['time_from_start_s'])
        self.assertTrue(np.all(np.diff(sampled) > 0))
        self.assertEqual(len(validation), len(sampled))


if __name__ == '__main__':
    unittest.main()
