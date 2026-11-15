from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from curobo_core.new_warp_reference import PolarCurve, polar_plan
from curobo_core.new_warp_prior import nearest_chain, swivel_schedule


class OriginalCurve:
    center_tool = np.zeros(3)
    angle = 0.
    timing_ratio = 1.

    def __init__(self, sign):
        self.endpoints = np.array([[.55, sign * .25, 1.], [.45, sign * .35, 1.1]])

    def evaluate(self, parameter):
        points = self.endpoints[0] + np.asarray(parameter)[:, None] * np.diff(self.endpoints, axis=0)
        return dict(positions=points, centers=points,
                    rotations=Rotation.from_quat(np.tile([0., 0., 0., 1.], (len(points), 1))))


def components(blocked=False):
    originals = {side: OriginalCurve(sign) for side, sign in [('left', 1), ('right', -1)]}
    context = {side: dict(shoulders=np.array([[0., 0., 1.], [0., 0., 1.1]]), preferred_radius=.693333967) for side in originals}
    models = {side: dict(centers_tool=[[0., 0., 0.]], radii=[.02]) for side in originals}
    objects = [dict(name='solid', position=[0., 0., 1.], dimensions=[10., 10., 10.], quaternion_wxyz=[1., 0., 0., 0.])] if blocked else []
    start, goal = np.zeros(15), np.zeros(15)
    start[0], goal[0] = -.3, -.2
    return dict(start=start, goal=goal, event=dict(review_scene_objects=objects)), context, originals, models, None, (-1., 0.)


class NewWarpContracts(unittest.TestCase):
    def test_colliding_reference_remains_guidance_but_never_claims_safe(self):
        scene = components(True)
        strict = polar_plan(None, 'case', 'transport', components=scene)
        soft = polar_plan(None, 'case', 'transport', components=scene, allow_colliding_reference=True)
        self.assertFalse(strict['guidance_available'])
        self.assertFalse(soft['accepted'])
        self.assertTrue(soft['guidance_available'])
        self.assertTrue(soft['used_colliding_reference'])
        for side, path in soft['paths'].items():
            self.assertTrue(path['reference_only'])
            np.testing.assert_allclose(np.asarray(path['positions'])[[0, -1]], scene[2][side].endpoints, atol=1e-8)

    def test_clear_reference_is_unchanged_by_allowing_soft_fallback(self):
        scene = components()
        strict = polar_plan(None, 'case', 'approach', components=scene)
        soft = polar_plan(None, 'case', 'approach', components=scene, allow_colliding_reference=True)
        self.assertTrue(strict['accepted'] and soft['accepted'])
        self.assertFalse(soft['used_colliding_reference'])
        for side in scene[1]:
            np.testing.assert_array_equal(strict['paths'][side]['positions'], soft['paths'][side]['positions'])

    def test_shared_lift_is_monotone_and_endpoint_geometry_is_fixed(self):
        scene = components()
        parameters = np.linspace(0., 1., 128)
        phases = []
        for side in scene[1]:
            states = PolarCurve(scene[2][side], scene[1][side], 1., .2, .75).evaluate(parameters)
            phases.append(states['lift_phase'])
            self.assertTrue((np.diff(states['lift_phase']) >= 0).all())
            np.testing.assert_allclose(states['positions'][[0, -1]], scene[2][side].endpoints, atol=1e-8)
        np.testing.assert_array_equal(*phases)

    def test_missing_analytic_layers_are_initialization_only(self):
        start, goal = np.zeros(15), np.ones(15)
        chain, legal_layers = nearest_chain(np.empty((0, 15)), np.empty(0, int), np.empty(0, bool), start, goal)
        self.assertFalse(legal_layers.any())
        self.assertTrue(np.isfinite(chain).all())
        np.testing.assert_array_equal(chain[[0, -1]], [start, goal])

    def test_swivel_schedule_uses_shortest_circle_without_changing_joint_endpoints(self):
        class Analytic:
            def swivel(self, arm, values):
                return values[0]
        start, goal = np.zeros(15), np.zeros(15)
        start[[1, 8]], goal[[1, 8]] = np.deg2rad(179.), np.deg2rad(-179.)
        angles, ends, delta = swivel_schedule(Analytic(), start, goal, 128)
        np.testing.assert_allclose(np.rad2deg(delta), [2., 2.])
        np.testing.assert_allclose(np.exp(1j * angles[-1]), np.exp(1j * ends[-1]))


if __name__ == '__main__':
    unittest.main()
