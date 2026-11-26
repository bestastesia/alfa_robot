"""Robot ground margin and physical carton support are separate checks."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))


class PayloadFloorTest(unittest.TestCase):
    def test_native_margin_keeps_robot_clearance_and_only_exempts_registered_payload_suffix(self):
        path = ROOT/'tools/v3_two_box_pick.py'
        cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'TwoBoxPickPlanner')
        node = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'native_margin_mask')
        env = dict(torch=torch)
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), env)
        spheres = torch.tensor([[[0., 0., .05, .02], [0., 0., .2-1e-8, .2]]])
        state = NS(robot_spheres=spheres[None])
        checker = NS(base_cuboid_layer=NS(cost=lambda state:torch.zeros((1, 1))))
        mask = env['native_margin_mask'](None, checker, torch.zeros((1, 15)), state=state)
        self.assertFalse(bool(mask.all()))
        checker.payload_ground_exempt_sphere_indices = (1,)
        mask = env['native_margin_mask'](None, checker, torch.zeros((1, 15)), state=state)
        self.assertTrue(bool(mask.all()))
        spheres[0, 0, 2] = .021
        self.assertFalse(bool(env['native_margin_mask'](None, checker, torch.zeros((1, 15)), state=state).all()))

    def test_actual_full_box_ground_audit_still_rejects_penetration(self):
        from test_payload_safety import cpu_audit, PayloadSafetyTest
        checker, _ = PayloadSafetyTest().checker()
        rows = np.array([[1., 0., .19, 5., 5., 5.]], dtype=np.float32)
        attachments = {'left': np.eye(4)}
        dimensions = (.3, .4, .4)
        robot = {'kinematics': {'collision_spheres': {'left_link7': [{}], 'right_link7': [{}]}}}
        result = cpu_audit()(checker, rows, attachments, dimensions, robot, check_ground=True)
        self.assertFalse(result['valid'])
        self.assertTrue(any(f['kind'] == 'box_ground' for f in result['failures']))


if __name__ == '__main__':unittest.main()
