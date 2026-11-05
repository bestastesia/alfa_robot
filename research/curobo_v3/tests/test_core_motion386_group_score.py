from pathlib import Path
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from motion306_learning.group_score import GroupScoreConfig, group_score_terms, score_groups
from motion306_learning.group_selector import (CandidatePool, ContactPool, GroupBeamConfig, PlacePool,
                                                select_group)

ROOT = Path(__file__).resolve().parents[1]


def redundancy(updown=(0., .1, .2), left=(0., 10., 20.), right=(0., -10., -20.)):
    return np.array([updown, left, right], dtype=float).T


class GroupScoreTest(unittest.TestCase):
    def setUp(self):
        self.velocity = np.ones(15)
        self.lower = np.full(15, -2.)
        self.upper = np.full(15, 2.)
        self.config = GroupScoreConfig.load(ROOT / "config/motion386_b2_group_smooth.json")

    def test_config_is_seconds_and_frozen_weights_load(self):
        self.assertEqual((self.config.motion_weight, self.config.turn_penalty_s,
                          self.config.reversal_penalty_s, self.config.redundancy_penalty_s),
                         (1., .75, 1., .5))
        self.assertEqual((self.config.collision_penalty_s, self.config.joint_limit_penalty_s,
                          self.config.score_temperature_s), (2., 1., 5.))

    def test_consistent_direction_beats_reversal(self):
        smooth = np.zeros((3, 15))
        smooth[:, 1] = [0., .5, 1.]
        reverse = smooth.copy()
        reverse[:, 1] = [0., .5, 0.]
        points = np.stack([smooth, reverse])
        redundant = np.stack([redundancy(), redundancy()])
        costs = group_score_terms(points, self.velocity, self.lower, self.upper, redundant, np.zeros((2, 2)), self.config)
        self.assertEqual(costs["turn_fraction"][0], 0.)
        self.assertEqual(costs["reversal_fraction"][0], 0.)
        self.assertGreater(costs["turn_fraction"][1], 0.)
        self.assertGreater(costs["reversal_fraction"][1], 0.)
        self.assertLess(costs["total_s"][0], costs["total_s"][1])

    def test_redundancy_curvature_and_collision_are_monotonic(self):
        points = np.zeros((2, 3, 15))
        points[:, :, 2] = [0., .2, .4]
        straight = redundancy()
        bent = redundancy(updown=(0., .1, 0.), left=(0., 30., 0.), right=(0., -30., 0.))
        costs = group_score_terms(points, self.velocity, self.lower, self.upper,
                                  np.stack([straight, bent]), [[0., 0.], [0., .5]], self.config)
        self.assertEqual(costs["redundancy_curvature"][0], 0.)
        self.assertGreater(costs["redundancy_curvature"][1], 0.)
        self.assertEqual(costs["terms"]["collision_s"][1], 1.)
        self.assertGreater(costs["total_s"][1], costs["total_s"][0])

    def test_limit_is_soft_inside_margin_and_hard_outside(self):
        groups = np.zeros((3, 3, 15))
        groups[1, 1, 0] = 1.9  # 2.5% normalized margin: half of the soft term.
        groups[2, 1, 0] = 2.01
        red = np.stack([redundancy()] * 3)
        result = group_score_terms(groups, self.velocity, self.lower, self.upper, red, np.zeros((3, 2)), self.config)
        self.assertEqual(result["terms"]["joint_limit_s"][0], 0.)
        self.assertAlmostEqual(result["terms"]["joint_limit_s"][1], .5)
        self.assertTrue(np.isinf(result["total_s"][2]))
        self.assertFalse(result["valid"][2])

    def test_relative_score_is_100_for_best_and_decreases_with_cost(self):
        groups = np.zeros((2, 3, 15))
        groups[0, :, 1] = [0., .2, .4]
        groups[1, :, 1] = [0., .5, 0.]
        red = np.stack([redundancy()] * 2)
        result = score_groups(groups, self.velocity, self.lower, self.upper, red, np.zeros((2, 2)), self.config)
        self.assertAlmostEqual(result["score"].max(), 100.)
        self.assertGreater(result["score"][0], result["score"][1])
        expected = 100. * np.exp(-(result["total_s"][1] - result["total_s"][0]) / 5.)
        self.assertAlmostEqual(result["score"][1], expected)

    def test_missing_redundancy_or_risk_is_rejected(self):
        points = np.zeros((3, 15))
        with self.assertRaises(ValueError):
            group_score_terms(points, self.velocity, self.lower, self.upper, None, np.zeros(2), self.config)
        with self.assertRaises(ValueError):
            group_score_terms(points, self.velocity, self.lower, self.upper, redundancy(), [0.], self.config)


class FakeGroupBackend:
    home = np.zeros(15)
    velocity = np.ones(15)
    lower = np.full(15, -2.)
    upper = np.full(15, 2.)

    def __init__(self, empty=False):
        self.empty = empty
        self.carry_calls = []
        self.place_calls = []

    def contact_pool(self):
        rows = np.zeros((1, 15)); rows[0, 1] = .2
        extraction = np.zeros((1, 15)); extraction[0, 1] = .4
        return ContactPool(rows, [[0., 0., 0.]], [0.], [5], [not self.empty], extraction)

    def carry_pool(self, contact_index, extraction):
        self.carry_calls.append(contact_index)
        rows = np.zeros((2, 15)); rows[:, 1] = [.6, -.6]
        return CandidatePool(rows, [[.1, 10., -10.], [.1, -30., 30.]], [0., 0.], [7, 8], [True, True])

    def place_pool(self, contact_index, carry_index, carry_row):
        self.place_calls.append((contact_index, carry_index))
        rows = np.zeros((2, 15))
        if carry_index == 7:
            rows[:, 1] = [.8, .4]
            redundant = [[.2, 20., -20.], [0., 0., 0.]]
        else:
            rows[:, 1] = [-.8, -.4]
            redundant = [[.2, -40., 40.], [0., 0., 0.]]
        return PlacePool(rows, redundant, [0., 0.], [9, 10], [True, True], [0., 0.])


class GroupSelectorTest(unittest.TestCase):
    def test_beam_config_loads_from_same_frozen_document(self):
        config = GroupBeamConfig.load(ROOT / "config/motion386_b2_group_smooth.json")
        self.assertEqual((config.contact_beam, config.pair_beam, config.place_cap,
                          config.full_group_limit), (4, 16, 64, 1024))

    def test_time_ablation_removes_only_endpoint_smoothness_terms(self):
        baseline = GroupScoreConfig.load(ROOT / "config/motion386_b2_group_smooth.json")
        time_only = GroupScoreConfig.load(ROOT / "config/motion386_b2_time.json")
        self.assertEqual((time_only.turn_penalty_s, time_only.reversal_penalty_s,
                          time_only.redundancy_penalty_s), (0., 0., 0.))
        self.assertEqual((time_only.motion_weight, time_only.collision_penalty_s,
                          time_only.joint_limit_penalty_s, time_only.edge_collision_samples),
                         (baseline.motion_weight, baseline.collision_penalty_s,
                          baseline.joint_limit_penalty_s, baseline.edge_collision_samples))
        beam = GroupBeamConfig.load(ROOT / "config/motion386_b2_time.json")
        self.assertEqual(beam, GroupBeamConfig.load(ROOT / "config/motion386_b2_group_smooth.json"))

    def test_time_beam_ablations_change_only_pair_capacity(self):
        base_score = GroupScoreConfig.load(ROOT / "config/motion386_b2_time.json")
        base_beam = GroupBeamConfig.load(ROOT / "config/motion386_b2_time.json")
        for name, pair_beam in (("motion386_b2_time_beam12.json", 12),
                                ("motion386_b2_time_beam8.json", 8)):
            self.assertEqual(GroupScoreConfig.load(ROOT / "config" / name), base_score)
            beam = GroupBeamConfig.load(ROOT / "config" / name)
            self.assertEqual((beam.contact_beam, beam.place_cap, beam.selection_rule),
                             (base_beam.contact_beam, base_beam.place_cap, base_beam.selection_rule))
            self.assertEqual((beam.pair_beam, beam.full_group_limit), (pair_beam, pair_beam * 64))

    def test_selector_returns_one_smooth_group_with_auditable_indices(self):
        backend = FakeGroupBackend()
        score = GroupScoreConfig(motion_weight=0., collision_penalty_s=0., joint_limit_penalty_s=0.)
        choice = select_group(backend, score, GroupBeamConfig(contact_beam=1, pair_beam=2,
                                                              place_cap=2, full_group_limit=4))
        self.assertIsNotNone(choice)
        self.assertEqual((choice.contact_index, choice.carry_index, choice.place_index), (5, 7, 9))
        self.assertEqual(choice.rows.shape, (6, 15))
        self.assertEqual(choice.redundancy.shape, (3, 3))
        self.assertEqual(choice.audit["full_groups"], 4)
        self.assertEqual(backend.carry_calls, [5])
        self.assertEqual(backend.place_calls, [(5, 7), (5, 8)])

    def test_no_hard_valid_contact_returns_no_group(self):
        self.assertIsNone(select_group(FakeGroupBackend(empty=True)))

    def test_near_optimal_rule_prefers_smooth_group_inside_primary_slack(self):
        backend = FakeGroupBackend()
        choice = select_group(backend, GroupScoreConfig(collision_penalty_s=0., joint_limit_penalty_s=0.),
                              GroupBeamConfig(contact_beam=1, pair_beam=2, place_cap=2, full_group_limit=4,
                                              selection_rule="near_optimal_smooth", primary_cost_slack_s=.5))
        self.assertEqual((choice.contact_index, choice.carry_index, choice.place_index), (5, 7, 9))
        self.assertEqual(choice.audit["selection_rule"], "near_optimal_smooth")
        self.assertGreater(choice.audit["selected_weighted_rank"], 0)


if __name__ == "__main__":
    unittest.main()
