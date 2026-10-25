from pathlib import Path
import json
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import v3_candidate_ranker as ranker

URDF = Path(__file__).resolve().parents[1] / "generated/v3_analytic_071cb95/model/robot_meter_meshes.urdf"
CONTEXT = dict(kind="dual", sides=["left", "right"], top_grasp=False, home=np.zeros(15),
               box_centers={"left": [.9, -.4, 1.4], "right": [.9, .4, 1.4]})


def tiny_model(path, inputs, names):
    rng = np.random.default_rng(0)
    arrays = dict(mean=np.zeros(inputs), scale=np.ones(inputs))
    for member in range(2):
        arrays[f"m{member}_w0"] = rng.normal(size=(inputs, 4))
        arrays[f"m{member}_b0"] = rng.normal(size=4)
        arrays[f"m{member}_w1"] = rng.normal(size=(4, 2))
        arrays[f"m{member}_b1"] = rng.normal(size=2)
    meta = dict(members=2, features=list(names), feature_hash=ranker.feature_hash(names), config={})
    np.savez(path, meta=np.array(json.dumps(meta)), **arrays)


class FeatureTest(unittest.TestCase):
    def setUp(self):
        self.kinematics = ranker.Kinematics(URDF)
        self.contacts = np.random.default_rng(1).uniform(-1, 0, (64, 15))

    def test_names_match_columns(self):
        self.assertEqual(ranker.s1_t0(self.contacts, CONTEXT, self.kinematics).shape, (64, len(ranker.S1_T0_NAMES)))
        self.assertEqual(ranker.s2_t0(self.contacts[0], self.contacts, CONTEXT).shape, (64, len(ranker.S2_T0_NAMES)))

    def test_set_features_follow_the_candidates(self):
        features = ranker.s1_t0(self.contacts, CONTEXT, self.kinematics)
        permutation = np.random.default_rng(2).permutation(64)
        np.testing.assert_allclose(ranker.s1_t0(self.contacts[permutation], CONTEXT, self.kinematics), features[permutation])

    def test_contact_features_reuse_swivels_without_changing_values(self):
        from types import SimpleNamespace
        from v3_triple_ranker import contact_features, time_bound

        class Analytic:
            calls = 0

            def swivel(self, side, row):
                self.calls += 1
                return float(row.sum() + side)

            def solve(self, side, target, swivel, seed):
                return np.asarray([seed + swivel * .001])

        class Tree:
            def query(self, rows, k):
                return np.zeros((len(rows), k)), np.zeros((len(rows), k), dtype=int)

        analytic = Analytic()
        self.kinematics.analytic = analytic
        self.kinematics.carriage = SimpleNamespace(inverse=lambda u: np.repeat(np.eye(4)[None], len(u), 0))
        self.kinematics.trees = {"side": {"full": Tree()}}
        context = dict(CONTEXT, contact_poses={side + "_tool0": dict(position=[1., 0., 1.], quaternion=[1., 0., 0., 0.])
                                             for side in ("left", "right")})
        ends = ranker.endpoints(self.contacts, context, self.kinematics)
        extra, eligible = ranker.s1_t1(self.contacts, context, self.kinematics, ends)
        times = np.column_stack([time_bound(context["home"], self.contacts, self.kinematics.velocity),
                                 time_bound(self.contacts, ends, self.kinematics.velocity)])
        expected = np.concatenate([ranker.s1_t0(self.contacts, context, self.kinematics), extra, times], axis=1)
        analytic.calls = 0
        actual, actual_ends, actual_eligible = contact_features(self.contacts, context, self.kinematics, self.kinematics.velocity)
        np.testing.assert_array_equal(expected, actual)
        np.testing.assert_array_equal(ends, actual_ends)
        np.testing.assert_array_equal(eligible, actual_eligible)
        self.assertEqual(analytic.calls, len(self.contacts) * 2)

    def test_limits_from_urdf(self):
        self.assertEqual(self.kinematics.lower[0], -1.)
        self.assertTrue(np.all(self.kinematics.velocity > 0))


class PolicyTest(unittest.TestCase):
    def test_order_and_abstain(self):
        probability = np.array([.9, .9, .2, .95])
        cost = np.array([10., 12., 5., 11.])
        self.assertEqual(ranker.attempt_order(probability, cost), [0, 3, 1])
        self.assertEqual(ranker.attempt_order(probability, cost, eligible=[False, True, True, True]), [3, 1])
        self.assertEqual(ranker.attempt_order(probability, cost, theta=.1), [0, 2, 3, 1])
        self.assertIsNone(ranker.attempt_order(np.full(4, .3), cost))
        self.assertIsNone(ranker.attempt_order(probability, cost, max_z=np.array([1, 7, 1, 1])))


class EnsembleTest(unittest.TestCase):
    def test_load_checks_hash_and_features(self):
        names = [f"x{index}" for index in range(5)]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "model.npz"
            tiny_model(path, 5, names)
            model = ranker.Ensemble.load(path, feature_names=names)
            probability, cost, max_z = model.predict(np.ones((3, 5)))
            self.assertEqual(probability.shape, (3,))
            self.assertTrue(np.all((probability > 0) & (probability < 1)) and np.all(cost > 0))
            with self.assertRaises(ValueError):
                ranker.Ensemble.load(path, sha256="0" * 64)
            with self.assertRaises(ValueError):
                ranker.Ensemble.load(path, feature_names=names[::-1])

    def test_inference_uses_no_random_state(self):
        names = [f"x{index}" for index in range(5)]
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "model.npz"
            tiny_model(path, 5, names)
            model = ranker.Ensemble.load(path)
            state = np.random.get_state()[1].copy()
            first = model.predict(np.ones((2, 5)))
            np.testing.assert_array_equal(state, np.random.get_state()[1])
            np.testing.assert_array_equal(first[0], model.predict(np.ones((2, 5)))[0])


if __name__ == "__main__":
    unittest.main()
