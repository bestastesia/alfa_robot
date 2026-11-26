"""CPU check for the actual loaded search gravity filter; native GPU calls are replaced."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from curobo_core.box_pick import BoxPick, layout_model_key


class StaticLoadTest(unittest.TestCase):
    def test_chunked_payload_rnea_rejects_overload_and_nonfinite_without_limiting_lift(self):
        path = ROOT/'tools/v3_box_layout_planner.py'
        cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'BoxLayoutPlanner')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'static_load_mask')
        policy = dict(arm_continuous_torque_nm=[294, 294, 178, 178, 87, 52, 10], planning_limit_scale=.995)
        namespace = dict(torch=torch, layout_model_key=layout_model_key, POLICY=policy, JointState=lambda **kw:NS(**kw))
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
        picks = {s:BoxPick('40x40x30', (1.1680520510673524, y, 1.), s, carry_joint_home=True)
                 for s, y in [('left', .5), ('right', -.5)]}
        attachments = {s:p.tool_to_box() for s, p in picks.items()}
        batches = []; calls = []
        def rnea(state):
            torch.testing.assert_close(state.velocity, torch.zeros_like(state.position))
            torch.testing.assert_close(state.acceleration, torch.zeros_like(state.position))
            torque = torch.zeros_like(state.position)
            torque[:, 0] = 1e20  # inherited planning policy leaves the shared lift effort unrestricted
            torque[:, 1] = 294*state.position[:, 1]
            return torque
        dynamics = NS(setup_batch_size=batches.append, compute_inverse_dynamics=rnea)
        def model(sides, actual):
            calls.append(sides)
            self.assertIs(actual, attachments)
            return dynamics
        planner = NS(picks=picks, layout_native=(layout_model_key(picks), NS(dynamics=NS(_model=model))),
                     static_load_filter_statistics=dict(rows=0, rejected=0))
        values = torch.zeros((515, 15));values[:, 1] = .5
        values[510, 1] = -.996;values[512, 1] = .996;values[514, 1] = float('nan')
        mask = namespace['static_load_mask'](planner, NS(task_tool_to_box=attachments), values)
        expected = np.ones(515, dtype=bool);expected[[510, 512, 514]] = False
        np.testing.assert_array_equal(mask.numpy(), expected)
        self.assertEqual(batches, [512, 3]);self.assertEqual(calls, [('left', 'right')])
        self.assertEqual(planner.static_load_filter_statistics, dict(rows=515, rejected=3))


if __name__ == '__main__':unittest.main()
