from pathlib import Path
from types import SimpleNamespace
import sys
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
from motion311_learning.contracts import StepResult
from motion311_learning.features import PHYSICS_DIM, STATE_DIM
from motion306_learning.env import EXTRA_DIM, LookaheadObservation
from motion306_learning.evaluate import evaluate_instance
from motion306_learning.rankers import B1CostConfig, b1_cost_terms, b1_order

ROOT = Path(__file__).resolve().parents[1]


def observation(stage, extra, valid=None):
    extra = np.asarray(extra, dtype=float).reshape(-1, EXTRA_DIM)
    count = len(extra)
    state = np.zeros(STATE_DIM)
    state[stage] = 1.
    rows = np.zeros((count, 15))
    rows[:, 1] = np.arange(count) * .1
    return LookaheadObservation(stage, state, rows, np.zeros((count, 3)), np.zeros((count, PHYSICS_DIM)),
                                np.repeat(np.eye(4)[None], 2, axis=0), extra=extra,
                                valid=np.ones(count, bool) if valid is None else valid, dims=[.3, .4, .4])


class B1CostTest(unittest.TestCase):
    def test_committed_config_preserves_validated_nonzero_terms(self):
        config = B1CostConfig.load(ROOT / "config/motion365_b1_v1.json")
        self.assertEqual(config.selection_budget_ms, 2000.)
        self.assertEqual((config.contact_approach_weight, config.contact_extraction_weight,
                          config.contact_nearest_carry_weight), (1., 1., 1.))
        self.assertEqual((config.contact_carry_mean_excess_weight, config.contact_scarcity_penalty_s,
                          config.carry_home_proxy_weight, config.joint_limit_penalty_s), (0., 0., 0., 0.))
        v2 = B1CostConfig.load(ROOT / "config/motion365_b1_v2.json")
        self.assertEqual((v2.direct_path_samples, v2.approach_direct_risk_penalty_s,
                          v2.place_direct_risk_penalty_s, v2.home_direct_risk_penalty_s), (9, 2., 2., 2.))
        v3 = B1CostConfig.load(ROOT / "config/motion365_b1_v3_carry.json")
        self.assertEqual((v3.approach_direct_risk_penalty_s, v3.carry_direct_risk_penalty_s,
                          v3.place_direct_risk_penalty_s, v3.home_direct_risk_penalty_s), (0., 2., 2., 2.))

    def test_stage_costs_are_seconds_and_invalid_rows_sort_last(self):
        config = B1CostConfig()
        lower, upper, velocity = np.full(15, -1.), np.full(15, 1.), np.ones(15)
        contact = observation(0, [[1., 2., 1., 3., 8., .2], [.5, .2, 0., .1, .1, 1.]])
        costs = b1_cost_terms(contact, velocity, lower, upper, config)
        self.assertEqual(costs["total_s"][0], 6.)
        self.assertTrue(np.isinf(costs["total_s"][1]))
        self.assertEqual(b1_order(contact, velocity, lower, upper, config).tolist(), [0, 1])
        contact.candidates.setflags(write=True)
        contact.candidates[0, 0] = 1.01
        contact.candidates.setflags(write=False)
        self.assertTrue(np.isinf(b1_cost_terms(contact, velocity, lower, upper, config)["total_s"][0]))

        carry = observation(1, [[2., 1., 9., .4, 0., 0.], [1., 0., 0., 0., 0., 0.]], [True, False])
        costs = b1_cost_terms(carry, velocity, lower, upper, config)
        self.assertEqual(costs["total_s"][0], 2.)
        self.assertTrue(np.isinf(costs["total_s"][1]))

        place = observation(2, [[2., 3., 5., 0., 0., 0.], [1., 5., 6., 0., 0., 0.]])
        np.testing.assert_array_equal(b1_cost_terms(place, velocity, lower, upper, config)["total_s"], [5., 6.])

    def test_optional_soft_terms_and_stable_tie_break(self):
        config = B1CostConfig(contact_carry_mean_excess_weight=.5, contact_scarcity_penalty_s=2.,
                              joint_limit_penalty_s=1.)
        obs = observation(0, [[1., 1., 1., 1., 3., .25], [1., 1., 1., 1., 1., 1.]])
        obs.candidates.setflags(write=True)
        obs.candidates[0, 0] = .99
        obs.candidates.setflags(write=False)
        costs = b1_cost_terms(obs, np.ones(15), np.full(15, -1.), np.full(15, 1.), config)
        self.assertGreater(costs["total_s"][0], costs["total_s"][1])
        tie = observation(2, np.ones((3, EXTRA_DIM)))
        self.assertEqual(b1_order(tie, np.ones(15), np.full(15, -1.), np.full(15, 1.), B1CostConfig()).tolist(), [0, 1, 2])

    def test_direct_risk_is_soft_seconds_and_requires_explicit_measurement(self):
        obs = observation(0, [[1., 1., 1., 1., 1., 1.], [1., 1., 1., 1., 1., 1.]])
        config = B1CostConfig(approach_direct_risk_penalty_s=2.)
        limits = (np.full(15, -1.), np.full(15, 1.))
        with self.assertRaises(ValueError):
            b1_cost_terms(obs, np.ones(15), *limits, config)
        costs = b1_cost_terms(obs, np.ones(15), *limits, config, {"approach": np.array([.75, .25])})
        np.testing.assert_allclose(costs["terms"]["approach_direct_risk_s"], [1.5, .5])
        self.assertEqual(b1_order(obs, np.ones(15), *limits, config, {"approach": np.array([.75, .25])}).tolist(), [1, 0])
        with self.assertRaises(ValueError):
            B1CostConfig(direct_path_samples=1)


class B1Env:
    def __init__(self):
        self.velocity = np.ones(15)
        self.decoder = SimpleNamespace(lower=np.full(15, -2.), upper=np.full(15, 2.))
        self.generation_ms, self.feature_ms, self.lookahead_ms = 1., 2., 3.
        self.steps = []

    def _observation(self, stage):
        extra = np.zeros((3, EXTRA_DIM))
        if stage == 0:
            extra[:, [0, 1, 2, 3]] = [[1., 1., 1., 3.], [1., 1., 1., 1.], [1., 1., 1., 2.]]
        elif stage == 1:
            extra[:, 0] = [2., 1., 3.]
        else:
            extra[:, 0:2] = [[2., 2.], [1., 1.], [3., 3.]]
        return observation(stage, extra)

    def reset(self, item, seed):
        self.stage, self.cost_s = 0, 0.
        self.report = dict(success=False, frames=[[0.] * 15], phases=["home"], payload=[False])
        self.observation = self._observation(0)
        return self.observation

    def contact_collision_free(self, rows):
        return np.ones(len(rows), dtype=bool)

    def contact_checks(self, row):
        return np.zeros((2, 15)), None

    def carry_checks(self, rows):
        return [None] * len(rows)

    def b1_direct_risk(self, observation, samples):
        self.risk_samples = samples
        return ({"approach": np.array([1., 0., .5])} if observation.stage == 0 else
                {"place": np.zeros(3), "home": np.zeros(3)} if observation.stage == 2 else
                {"carry": np.zeros(3)})

    def step(self, index):
        self.steps.append((self.stage, index))
        self.cost_s += 1.
        info = dict(stage=self.stage, success=False, class_="stage_success", cumulative_cost_s=self.cost_s)
        if self.stage == 2:
            self.report.update(success=True, timed_validation={"success": True})
            info.update(success=True, class_="success")
            return StepResult(None, 1., True, False, info)
        self.stage += 1
        self.observation = self._observation(self.stage)
        return StepResult(self.observation, -.01, False, False, info)


class B1AuditTest(unittest.TestCase):
    def test_strict_episode_records_cost_and_complete_timing_contract(self):
        item = dict(instance="b1", split="dev", group="dev", task={"left": 1, "right": 2}, top_grasp=False,
                    kind="dual", offset_m=[0., 0.], box_level="S30x40x40", box_dims=[.3, .4, .4], size_split="seen")
        env, config = B1Env(), B1CostConfig(selection_budget_ms=100.)
        row = evaluate_instance(env, item, "b1", config, seed=4)
        self.assertTrue(row["success"])
        self.assertEqual(env.steps, [(0, 1), (1, 1), (2, 1)])
        self.assertTrue(all(outcome["b1_cost"]["units"] == "seconds" for outcome in row["outcomes"]))
        self.assertEqual(row["outcomes"][0]["b1_cost"]["selected_terms_s"]["nearest_carry_s"], 1.)
        self.assertGreaterEqual(row["selection_total_ms"], 18.)
        self.assertEqual(row["selection_budget_ms"], 100.)
        self.assertFalse(row["selection_budget_exceeded"])
        self.assertGreaterEqual(row["planning_wall_ms"], 0.)

    def test_risk_time_and_raw_fraction_are_audited(self):
        item = dict(instance="b1-risk", split="dev", group="dev", task={"left": 1, "right": 2}, top_grasp=False,
                    kind="dual", offset_m=[0., 0.], box_level="S30x40x40", box_dims=[.3, .4, .4], size_split="seen")
        env = B1Env()
        row = evaluate_instance(env, item, "b1", B1CostConfig(approach_direct_risk_penalty_s=2.))
        self.assertTrue(row["success"])
        self.assertEqual(env.risk_samples, 9)
        self.assertIn("risk_ms", row["outcomes"][0])
        self.assertEqual(row["outcomes"][0]["b1_cost"]["selected_direct_risk"]["approach"], 0.)
        self.assertEqual(row["outcomes"][0]["b1_cost"]["selected_terms_s"]["approach_direct_risk_s"], 0.)


if __name__ == "__main__":
    unittest.main()
