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
from curobo_core.resource_stats import bounded_get, record_build, resource_delta, resource_snapshot
from curobo_core.urdf_fk import BatchToolFk
from curobo_core.planner import shortcut_waypoints, validated_shortcut
from curobo_core.scene import MeshObject, Pose
from curobo_core.trajectory import time_parameterize_polyline


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


class TwoSidedPolylineTests(unittest.TestCase):
    def test_two_sided_law_shortens_to_the_binding_limit_without_exceeding_it(self):
        names = [f'j{i}' for i in range(3)]
        velocity, acceleration, jerk = np.array([1., 2., 3.]), np.full(3, 10.), np.full(3, 500.)
        # Equal motion on three joints: the initial L2 bound leaves each joint at ~55% speed.
        path = np.linspace([0., 0., 0.], [1., 2., 3.], 25)
        reference, _ = time_parameterize_polyline(path, names, velocity, acceleration, jerk)
        shortened, _ = time_parameterize_polyline(path, names, velocity, acceleration, jerk, two_sided=True)
        self.assertLess(shortened['duration_s'], reference['duration_s'])
        times = np.asarray(shortened['time_from_start_s'])
        ratios = [np.max(np.abs(np.asarray(shortened[key]))/limit)**(1/order)
                  for key, limit, order in (('velocities', velocity, 1), ('accelerations', acceleration, 2), ('jerks', jerk, 3))]
        self.assertLessEqual(max(ratios), 1.0001)
        self.assertGreaterEqual(max(ratios), 1/1.05**2-1e-3)
        np.testing.assert_allclose(np.asarray(shortened['positions'])[[0, -1]], path[[0, -1]], atol=1e-6)
        self.assertEqual(times[0], 0.)

    def test_default_law_is_unchanged(self):
        names = ['a', 'b']
        path = np.array([[0., 0.], [.5, .2], [.7, .9]])
        first, _ = time_parameterize_polyline(path, names, np.ones(2), np.full(2, 10.), np.full(2, 500.))
        second, _ = time_parameterize_polyline(path, names, np.ones(2), np.full(2, 10.), np.full(2, 500.), two_sided=False)
        self.assertEqual(first['duration_s'], second['duration_s'])


class CorridorValidity:
    """2-D free space y <= .2 or x >= .8; `coarse_edges` accepts every edge (a too-coarse check)."""
    def __init__(self, coarse_edges=False):
        self.weights, self.coarse_edges = torch.ones(2), coarse_edges

    def mask(self, values):
        return (values[:, 1] <= .2) | (values[:, 0] >= .8)

    def edges(self, starts, goals, resolution=.01):
        if self.coarse_edges:
            return torch.ones(len(starts), dtype=torch.bool)
        fractions = torch.linspace(0., 1., 201)[None, :, None]
        points = starts[:, None]+fractions*(goals-starts)[:, None]
        return self.mask(points.reshape(-1, 2)).reshape(len(starts), -1).all(1)


class SuffixShortcutTests(unittest.TestCase):
    path = np.array([[0., 0.], [.5, 0.], [1., 0.], [1., .5], [1., 1.]])

    def test_shortcut_keeps_only_corners_with_valid_edges(self):
        np.testing.assert_array_equal(shortcut_waypoints(self.path, CorridorValidity()), self.path[[0, 2, 4]])
        short, dense = validated_shortcut(self.path, CorridorValidity())
        np.testing.assert_array_equal(short, self.path[[0, 2, 4]])
        self.assertTrue(bool(CorridorValidity().mask(torch.tensor(dense, dtype=torch.float32)).all()))

    def test_shortcut_whose_dense_rows_fail_is_rejected(self):
        # The coarse edge check jumps the corner; its dense rows cross the blocked square.
        self.assertEqual(len(shortcut_waypoints(self.path, CorridorValidity(coarse_edges=True))), 2)
        self.assertIsNone(validated_shortcut(self.path, CorridorValidity(coarse_edges=True)))


class BatchUrdfFkTests(unittest.TestCase):
    def test_batched_tool_chain_equals_yourdfpy(self):
        import yourdfpy
        from curobo_core.contracts import ACTIVE_JOINTS, PlannerAssets
        urdf = yourdfpy.URDF.load(PlannerAssets().urdf, load_meshes=False, build_scene_graph=True)
        frames = ('left_tool0', 'right_tool0', 'arm_carriage')
        fk = BatchToolFk(urdf, frames, ACTIVE_JOINTS)
        rows = np.random.default_rng(5).uniform(-2.5, 2.5, (64, 15))
        rows[:, 0] = np.random.default_rng(6).uniform(-.3, .1, 64)
        batch = {frame: fk.transforms(rows, frame) for frame in frames}
        for index, row in enumerate(rows):
            mapping = dict(zip(ACTIVE_JOINTS, row))
            urdf.update_cfg({name: mapping.get(name, 0.) for name in urdf.actuated_joint_names})
            for frame in frames:
                np.testing.assert_allclose(batch[frame][index], urdf.get_transform(frame, urdf.base_link), atol=1e-12, rtol=0)

    def test_yourdfpy_reads_depend_on_history_and_batched_fk_does_not(self):
        """trimesh keeps an edge matrix whose replacement is within 1e-8 of the previous read."""
        import yourdfpy
        from curobo_core.contracts import ACTIVE_JOINTS, PlannerAssets
        urdf = yourdfpy.URDF.load(PlannerAssets().urdf, load_meshes=False, build_scene_graph=True)
        fk = BatchToolFk(urdf, ('right_tool0',), ACTIVE_JOINTS)
        read = lambda row: (urdf.update_cfg({name: dict(zip(ACTIVE_JOINTS, row)).get(name, 0.)
                                             for name in urdf.actuated_joint_names}),
                            urdf.get_transform('right_tool0', urdf.base_link).copy())[1]
        row = np.random.default_rng(8).uniform(-1.5, 1.5, 15)
        row[0] = -.1
        moved = row.copy()
        moved[9] += 5e-9
        read(row)
        stale = read(moved)
        exact = fk.transforms(moved[None], 'right_tool0')[0]
        self.assertGreater(np.abs(stale-exact).max(), 1e-10)
        read(row+.1)
        np.testing.assert_allclose(read(moved), exact, atol=1e-12, rtol=0)


class ResourceStatsTests(unittest.TestCase):
    def test_bounded_cache_keeps_the_most_recently_used_entries(self):
        cache, builds = {}, []
        get = lambda key: bounded_get(cache, key, lambda: builds.append(key) or f'resource-{key}', 2)
        self.assertEqual(get('a'), 'resource-a')
        get('b')
        get('a')  # hit: 'a' becomes the most recent entry
        get('c')  # evicts 'b', the least recently used
        self.assertEqual(list(cache), ['a', 'c'])
        get('b')
        self.assertEqual(builds, ['a', 'b', 'c', 'b'])
        self.assertEqual(list(cache), ['c', 'b'])

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
