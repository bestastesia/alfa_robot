"""CPU contracts for the five-column door latency work: identical decisions, bounded resources."""
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from curobo_core.conveyor import conveyor_pair
from curobo_core.door_sequential_validity import SequentialValidity, box_mesh_obb, material_self_overlap
from curobo_core.resource_stats import record_build, resource_delta, resource_snapshot
from curobo_core.scene import MeshObject, Pose


class NativeLikeChecker:
    """float32 mirror of cuRobo's self-collision kernel: skips negative radii, max (R^2-d^2)."""
    def __init__(self, pairs, padding):
        self.pairs, self.padding = pairs, padding.float()

    def get_self_collision(self, spheres):
        rows = spheres.reshape(-1, spheres.shape[-2], 4).float()
        first, second = rows[:, self.pairs[:, 0]], rows[:, self.pairs[:, 1]]
        radius = (first[..., 3]+self.padding[self.pairs[:, 0]]), (second[..., 3]+self.padding[self.pairs[:, 1]])
        valid = (radius[0] >= 0) & (radius[1] >= 0)
        value = ((radius[0]+radius[1])**2-((first[..., :3]-second[..., :3])**2).sum(-1))*valid
        return (0.5*value.max(-1).values.clamp_min(0.)).reshape(*spheres.shape[:-2], 1)


def bracketed_validity(spheres_count, payload_indices, pairs, padding):
    validity = SequentialValidity.__new__(SequentialValidity)
    validity.checker = NativeLikeChecker(pairs, padding)
    validity.payload_indices = list(payload_indices)
    payload = torch.zeros(spheres_count, dtype=torch.bool)
    payload[validity.payload_indices] = True
    validity.self_pairs = pairs
    validity.self_padding = padding.double()
    validity.robot_self_pairs = ~(payload[pairs[:, 0]] | payload[pairs[:, 1]])
    validity.self_margin_m = 1e-5
    validity.self_band_rows = 0
    return validity


class LoadedSelfCollisionTests(unittest.TestCase):
    def test_bracketed_native_decision_equals_float64_reference_at_the_boundary(self):
        generator = torch.Generator().manual_seed(7)
        count, payload_indices = 12, (10, 11)
        pairs = torch.tensor([(i, j) for i in range(count) for j in range(i+1, count) if j-i > 1])
        padding = torch.rand(count, generator=generator)*.01
        validity = bracketed_validity(count, payload_indices, pairs, padding)
        rows = torch.rand((4000, count, 4), generator=generator)
        rows[..., :3] = rows[..., :3]*.6
        rows[..., 3] = .02+rows[..., 3]*.08
        rows[:200, 3] = -100.  # disabled spheres never collide
        rows[200:400, 5, 3] = 0.  # zero radius is not material, as in the float64 reference
        collision = validity.exact_robot_self_collision(rows)
        hit_rows, free_rows = rows[collision][:300], rows[~collision][:300]
        count_pairs = min(len(hit_rows), len(free_rows))
        low, high = torch.zeros(count_pairs, dtype=torch.float64), torch.ones(count_pairs, dtype=torch.float64)
        first, second = hit_rows[:count_pairs].double(), free_rows[:count_pairs].double()

        def at(t):
            values = first.clone()
            values[..., :3] = first[..., :3]+t[:, None, None]*(second[..., :3]-first[..., :3])
            return values.float()

        for _ in range(48):
            middle = (low+high)/2
            collide = validity.exact_robot_self_collision(at(middle))
            low, high = torch.where(collide, middle, low), torch.where(collide, high, middle)
        boundary = torch.cat([at(((low+high)/2+offset).clamp(0, 1))
                              for offset in (0., 1e-9, -1e-9, 1e-6, -1e-6, 1e-4, -1e-4)])
        for values in (rows, boundary):
            decided = validity._self_collision_cost(values[None]).reshape(-1) > 0
            np.testing.assert_array_equal(decided.numpy(), validity.exact_robot_self_collision(values).numpy())
        self.assertGreater(validity.self_band_rows, 0)
        self.assertLess(validity.self_band_rows, len(rows)+len(boundary))

    def test_payload_pairs_are_left_to_the_exact_obb_checks(self):
        pairs = torch.tensor([(0, 1), (0, 2)])
        padding = torch.zeros(3)
        validity = bracketed_validity(3, (2,), pairs, padding)
        rows = torch.tensor([[[0., 0., 0., .1], [1., 0., 0., .1], [.05, 0., 0., .1]]])  # only robot/payload overlap
        self.assertFalse(bool(validity._self_collision_cost(rows[None]).reshape(-1)[0] > 0))
        self.assertTrue(bool(material_self_overlap(rows, pairs, padding.double(), torch.tensor([False, False]), 0.)[0]))


class PayloadWorldMeshTests(unittest.TestCase):
    def test_conveyor_box_meshes_have_exact_obbs(self):
        parts = conveyor_pair(pose=Pose((-2.3, 0., 0.))).world_parts()
        self.assertEqual(len(parts), 2)
        for item in parts:
            center, rotation, half = box_mesh_obb(item)
            vertices = np.asarray(item.vertices_m)
            np.testing.assert_allclose(half, (vertices.max(0)-vertices.min(0))/2)
            np.testing.assert_allclose(center, np.asarray(item.pose.position)+(vertices.max(0)+vertices.min(0))/2)
            np.testing.assert_allclose(rotation, np.eye(3), atol=1e-12)

    def test_non_box_mesh_has_no_obb(self):
        tetrahedron = MeshObject('tetra', ((0., 0., 0.), (1., 0., 0.), (0., 1., 0.), (0., 0., 1.)),
                                ((0, 1, 2), (0, 1, 3), (0, 2, 3), (1, 2, 3)), Pose())
        self.assertIsNone(box_mesh_obb(tetrahedron))


class ResourceStatsTests(unittest.TestCase):
    def test_build_counters_report_per_round_deltas(self):
        import time
        owner = SimpleNamespace()
        before = resource_snapshot(owner)
        if torch.cuda.is_available():
            record_build(owner, 'door_checker', time.perf_counter())
            record_build(owner, 'door_checker', time.perf_counter())
            delta = resource_delta(before, resource_snapshot(owner))
            self.assertEqual(delta['door_checker']['builds'], 2)
            self.assertEqual(resource_delta(resource_snapshot(owner), resource_snapshot(owner)), {})


if __name__ == '__main__':
    unittest.main()
