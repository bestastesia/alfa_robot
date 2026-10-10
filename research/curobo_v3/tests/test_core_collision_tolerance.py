"""A numerical joint-bound tolerance must not weaken collision rejection."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

try:
    import torch
except ImportError:
    torch = None


@unittest.skipIf(torch is None, "requires torch")
class CollisionToleranceTest(unittest.TestCase):
    def test_bound_roundoff_is_independent_of_collision(self):
        # Load the real method without importing CUDA planning dependencies.
        source = Path(__file__).resolve().parents[1] / "tools/v3_batched_loaded_search.py"
        cls = next(n for n in ast.parse(source.read_text()).body
                   if isinstance(n, ast.ClassDef) and n.name == "GpuValidity")
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == "evaluate")
        namespace = {"torch": torch, "JointState": NS(from_position=lambda q, **kw: q)}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)
        state = NS(robot_spheres=torch.tensor([[[[0., 0., 1., .1]]]]))
        checker = NS(
            setup_batch_tensors=lambda *args: None,
            kinematics=NS(compute_kinematics=lambda q: state),
            collision_constraint=NS(update_num_spheres=lambda *args, **kw: None,
                                    forward=lambda state: torch.zeros(1, 1, 1)),
        )
        validity = NS(checker=checker, joint_names=["joint"], check_ground=False,
                      _stability_cost=lambda state, horizon: torch.zeros(horizon),
                      minimum_box_up_z=0., states_checked=0)
        for collision, bound, expected in ((0., 0., True), (0., 1e-14, True),
                                           (0., 1e-7, False), (1e-10, 1e-14, False)):
            with self.subTest(collision=collision, bound=bound):
                checker.get_self_collision = lambda spheres: torch.full((1, 1, 1), collision)
                checker.get_bound = lambda q: torch.full((1, 1, 1), bound)
                result, _ = namespace["evaluate"](validity, torch.zeros(1, 1))
                self.assertEqual(result.item(), expected)


if __name__ == "__main__":
    unittest.main()
