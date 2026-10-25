from pathlib import Path
import json
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tools"))
import v3_candidate_ranker as ranker
import v3_triple_ranker as triple

CONTEXT = dict(kind="dual", sides=["left", "right"], top_grasp=True, home=np.zeros(15),
               box_centers={"left": [.9, -.4, 1.4], "right": [.9, .4, 1.4]})
VELOCITY = np.array([5.] + [.2618] * 4 + [.5236] * 3 + [.2618] * 4 + [.5236] * 3)


def tiny_model(path, names):
    rng = np.random.default_rng(0)
    arrays = dict(mean=np.zeros(len(names)), scale=np.ones(len(names)) * 10)
    for member in range(2):
        arrays[f"m{member}_w0"] = rng.normal(size=(len(names), 4))
        arrays[f"m{member}_b0"] = rng.normal(size=4)
        arrays[f"m{member}_w1"] = rng.normal(size=(4, 2)) * .1
        arrays[f"m{member}_b1"] = rng.normal(size=2) * .1
    meta = dict(members=2, features=list(names), feature_hash=ranker.feature_hash(names), config={})
    np.savez(path, meta=np.array(json.dumps(meta)), **arrays)
    return ranker.Ensemble.load(path, feature_names=names)


class TripleRankerTest(unittest.TestCase):
    def setUp(self):
        rng = np.random.default_rng(3)
        self.home = np.zeros(15)
        self.carries = rng.uniform(-1, 1, (40, 15))
        self.places = rng.uniform(-1, 1, (100, 15))
        self.folder = tempfile.TemporaryDirectory()
        self.n3 = tiny_model(Path(self.folder.name) / "n3.npz", triple.PLACE_NAMES)
        self.n2 = tiny_model(Path(self.folder.name) / "n2.npz", triple.TRANSPORT_NAMES)

    def tearDown(self):
        self.folder.cleanup()

    def test_place_features_broadcast_the_carry(self):
        single = triple.place_features(self.carries[0], self.places, self.home, VELOCITY, CONTEXT)
        rowwise = triple.place_features(np.repeat(self.carries[:1], len(self.places), 0), self.places, self.home, VELOCITY, CONTEXT)
        self.assertEqual(single.shape, (100, len(triple.PLACE_NAMES)))
        np.testing.assert_array_equal(single, rowwise)

    def test_downstream_values_match_brute_force(self):
        for model, keep in [(None, 1000), (None, 7), (self.n3, 1000), (self.n3, 7)]:
            values, choice = triple.downstream_values(self.carries, self.places, self.home, VELOCITY, CONTEXT, model, keep=keep, chunk=16)
            for row, carry in enumerate(self.carries):
                bound = triple.time_bound(carry, self.places, VELOCITY) + triple.time_bound(self.places, self.home, VELOCITY)
                near = np.argsort(bound)[:keep]
                q, _ = triple.place_values(carry, self.places[near], self.home, VELOCITY, CONTEXT, model)
                self.assertAlmostEqual(values[row], q.min(), places=10)
                self.assertEqual(choice[row], near[np.argmin(q)])

    def test_contact_downstream_matches_brute_force(self):
        downstream, _ = triple.downstream_values(self.carries, self.places, self.home, VELOCITY, CONTEXT)
        ends = np.random.default_rng(4).uniform(-1, 1, (5, 15))
        ends[2] = np.nan
        for model, pool, keep in [(None, 10, 32), (None, 1000, 32), (self.n2, 10, 4)]:
            output = triple.contact_downstream(ends, self.carries, downstream, VELOCITY, CONTEXT, model, pool=pool, keep=keep)
            self.assertTrue(np.isinf(output[2]))
            for index in (0, 1, 3, 4):
                distance = np.linalg.norm((self.carries - ends[index]) * ranker.WEIGHTS, axis=1)
                near = np.argsort(distance)[:pool]
                values = triple.time_bound(ends[index], self.carries[near], VELOCITY) + downstream[near]
                if model is not None:
                    best = near[np.argsort(values, kind="stable")[:keep]]
                    values, _ = triple.carry_values(ends[index], self.carries[best], downstream[best], VELOCITY, CONTEXT, model)
                self.assertAlmostEqual(output[index], values.min(), places=10)

    def test_contact_pruning_preserves_exhaustive_values(self):
        class RiskModel:
            def __init__(self, probability):
                self.probability, self.rows_seen = probability, 0

            def predict(self, features):
                self.rows_seen += len(features)
                p = np.full(len(features), self.probability)
                return p, np.ones(len(p)), np.zeros(len(p))

        ends = np.zeros((3, 15))
        rows = np.repeat(np.arange(40.)[:, None], 15, axis=1)
        downstream = np.zeros(40)
        for probability in (0., .5, 1.):
            model = RiskModel(probability)
            actual = triple.contact_downstream(ends, rows, downstream, np.ones(15), CONTEXT, model)
            np.testing.assert_array_equal(actual, np.full(3, (1. - probability) * triple.C_FAIL_S))
            self.assertLess(model.rows_seen, 3 * 32)

        # The analytic winner can have high risk: do not prune a genuinely better runner-up.
        class VariableRisk(RiskModel):
            def predict(self, features):
                self.rows_seen += len(features)
                p = np.where(features[:, 15] == 0., 0., 1.)
                return p, np.ones(len(p)), np.zeros(len(p))

        model = VariableRisk(0.)
        actual = triple.contact_downstream(ends, rows, downstream, np.ones(15), CONTEXT, model)
        np.testing.assert_array_equal(actual, np.ones(3))

    def test_row_table_finds_exact_rows_only(self):
        rows = np.random.default_rng(5).uniform(-3, 3, (500, 15)).astype(np.float32)
        table = triple.RowTable(rows, np.arange(500.))
        queries = rows[[7, 499, 0]].copy()
        queries[2, 4] = np.nextafter(queries[2, 4], np.float32(10))
        np.testing.assert_array_equal(table.lookup(queries), [7, 499, -1])
        np.testing.assert_array_equal(table.lookup(rows.astype(np.float64)), np.arange(500))

    def test_ranked_keeps_only_confident_candidates(self):
        self.assertEqual(triple.ranked([3., 1., 2.], [.9, .4, .6]), [2, 0])
        self.assertEqual(triple.ranked([3., 1.], [.1, .2]), [])


if __name__ == "__main__":
    unittest.main()
