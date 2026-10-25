import ast
from pathlib import Path
from types import SimpleNamespace
import unittest

import numpy as np

SOURCE = Path(__file__).resolve().parents[1] / "tools/v3_triple_selection_planner.py"


def hook(name, namespace):
    """Exercise the real hook body without constructing a GPU planner."""
    tree = ast.parse(SOURCE.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "TripleSelectionPlanner")
    method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == name)
    module = ast.Module(body=[method], type_ignores=[])
    exec(compile(ast.fix_missing_locations(module), str(SOURCE), "exec"), namespace)
    return namespace[name]


class PlaceResolutionTest(unittest.TestCase):
    def test_skipped_first_place_is_recorded(self):
        rows = np.repeat(np.arange(3.)[:, None], 15, axis=1)
        planner = SimpleNamespace(selected_task={"left": 23}, _place_sets={"mode": (rows, np.zeros((3, 3)))},
                                  mode=lambda task: "mode", _velocity=np.ones(15), home_values=np.zeros(15),
                                  place_keep=3, place_attempts=2, theta=.5, model=lambda name: None,
                                  context=lambda task: {})
        values = SimpleNamespace(time_bound=lambda first, second, velocity: np.zeros(3),
                                 place_values=lambda *args: (np.arange(3.), np.ones(3)),
                                 ranked=lambda *args: [0, 1, 2])
        function = hook("place_options", dict(np=np, triple=values,
                        resolve=lambda planner, targets, row, params: None if row[0] == 0 else row))
        attempts, record = function(planner, np.zeros(15), {"left": np.eye(4)}, None)
        self.assertEqual(len(attempts), 2)
        self.assertEqual(record["attempt_ranks"], [1, 2])
        self.assertEqual([row["success"] for row in record["resolution"]], [False, True, True])


class SelectionTimingTest(unittest.TestCase):
    def test_single_hook_includes_baseline_ordering(self):
        clock = [0.]

        def baseline(start, candidates):
            clock[0] += .004
            return np.array([1, 0])

        planner = SimpleNamespace(selection_active=lambda: True, selection_report={"carry": []})

        def choose(root, rows, index):
            clock[0] += .005
            planner.selection_report["carry"].append(dict(ms=5.))
            return 1

        planner.carry_choice = choose
        function = hook("order_transport_goals", dict(np=np, time=SimpleNamespace(perf_counter=lambda: clock[0]),
                        super=lambda: SimpleNamespace(order_transport_goals=baseline)))
        result = function(planner, np.zeros(15), np.zeros((2, 15)))
        np.testing.assert_array_equal(result, [0])
        record = planner.selection_report["carry"][0]
        self.assertAlmostEqual(record["ms"], 9.)
        self.assertAlmostEqual(record["baseline_score_ms"], 4.)
        self.assertAlmostEqual(record["value_ms"], 5.)

    def test_dual_hook_includes_baseline_and_host_transfer(self):
        clock = [0.]

        class Tensor:
            def __init__(self, rows):
                self.rows = rows

            def cpu(self):
                clock[0] += .002
                return self

            def numpy(self):
                return self.rows

        def baseline():
            clock[0] += .003
            return [0., 1.]

        planner = SimpleNamespace(selection_active=lambda: True, selection_report={"carry": []})

        def choose(root, rows, index):
            clock[0] += .005
            planner.selection_report["carry"].append(dict(ms=5.))
            return 1

        planner.carry_choice = choose
        function = hook("placement_goal_order", dict(np=np, time=SimpleNamespace(perf_counter=lambda: clock[0]),
                        torch=SimpleNamespace(argmin=lambda values: SimpleNamespace(item=lambda: 0))))
        result = function(planner, Tensor(np.zeros(15)), Tensor(np.zeros((2, 15))), None, baseline)
        self.assertEqual(result, [1])
        record = planner.selection_report["carry"][0]
        self.assertAlmostEqual(record["ms"], 12.)  # 3 baseline + 4 transfers + 5 selection
        self.assertAlmostEqual(record["baseline_score_ms"], 3.)
        self.assertAlmostEqual(record["value_ms"], 5.)


if __name__ == "__main__":
    unittest.main()
