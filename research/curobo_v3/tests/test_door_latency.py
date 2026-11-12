"""CPU contracts for the five-column door latency work: identical decisions, bounded resources."""
from dataclasses import dataclass
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
from curobo_core.fast_builds import sphere_pairs
from curobo_core.resource_stats import bounded_get, record_build, resource_delta, resource_snapshot
from curobo_core.urdf_fk import BatchToolFk
from curobo_core.planner import shortcut_waypoints, validated_shortcut
from curobo_core.scene import MeshObject, Pose
from curobo_core.trajectory import polyline_time_scaling, time_parameterize_polyline, time_parameterize_polyline_shortest


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

    def test_cruise_scaling_is_a_smooth_monotonic_progress(self):
        u = np.linspace(0., 1., 20001)
        for ramp in (.35, .15, .08):
            s, peak = polyline_time_scaling(u, ramp)
            self.assertAlmostEqual(peak, 1/(1-ramp))
            self.assertEqual((s[0], s[-1]), (0., 1.))
            speed = np.gradient(s, u)
            self.assertGreaterEqual(speed.min(), -1e-12)
            self.assertAlmostEqual(speed.max(), peak, places=3)
            # No jump where the ramps meet the cruise: neighbouring samples differ by at most the
            # analytic slope bound (smoothstep' <= 1.875, |smoothstep''| <= 5.774).
            du = u[1]-u[0]
            acceleration = np.gradient(speed, u)
            self.assertLessEqual(np.abs(np.diff(speed)).max(), 1.01*peak*1.875/ramp*du)
            self.assertLessEqual(np.abs(np.diff(acceleration)).max(), 1.01*peak*5.774/ramp**2*du)
        np.testing.assert_allclose(polyline_time_scaling(u)[0], 10*u**3-15*u**4+6*u**5)

    def test_shortest_scaling_beats_the_quintic_on_a_velocity_bound_path_within_limits(self):
        names = [f'j{i}' for i in range(3)]
        velocity, acceleration, jerk = np.array([1., 2., 3.]), np.full(3, 50.), np.full(3, 5000.)
        path = np.linspace([0., 0., 0.], [1., 2., 3.], 25)
        quintic, _ = time_parameterize_polyline(path, names, velocity, acceleration, jerk, two_sided=True)
        best, _ = time_parameterize_polyline_shortest(path, names, velocity, acceleration, jerk)
        self.assertLess(best['duration_s'], .8*quintic['duration_s'])
        self.assertTrue(best['time_scaling'].startswith('cruise_ramp_'))
        for key, limit in (('velocities', velocity), ('accelerations', acceleration), ('jerks', jerk)):
            self.assertLessEqual(np.max(np.abs(np.asarray(best[key]))/limit), 1.0001)
        np.testing.assert_allclose(np.asarray(best['positions'])[[0, -1]], path[[0, -1]], atol=1e-6)

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


class BoxWorld:
    """cuRobo-like world cost for one axis-aligned box: per-sphere penetration, radius <= 0 disabled."""
    def __init__(self, low, high):
        self.low, self.high = torch.tensor(low), torch.tensor(high)

    def forward(self, state):
        spheres = state.robot_spheres
        distance = torch.linalg.vector_norm(spheres[..., :3]-torch.maximum(torch.minimum(spheres[..., :3], self.high), self.low), dim=-1)
        return ((spheres[..., 3] > 0) & (distance < spheres[..., 3])).float()*(spheres[..., 3]-distance)


def grown_validity(growth, side, exempt=()):
    from curobo_core.door_sequential_validity import CollisionMargins
    validity = SequentialValidity.__new__(SequentialValidity)
    validity.checker = SimpleNamespace(collision_constraint=BoxWorld((1., -1., -1.), (2., 1., 1.)))
    validity.margins = CollisionMargins(exempt_radius_m=.06)
    validity.world_growth_indices = torch.tensor([0, 1])
    validity.world_growth = torch.tensor(growth)
    validity.world_growth_side = torch.tensor(side)
    validity.world_exempt = [None, None]
    for index, points in exempt:
        validity.world_exempt[index] = torch.tensor(points).reshape(-1, 3)
    return validity


@dataclass
class SphereState:
    """The fields of a cuRobo kinematics state the world cost reads."""
    robot_spheres: torch.Tensor
    tool_poses: object


def sphere_state(spheres, tools):
    poses = {name: SimpleNamespace(position=torch.tensor(position)[None]) for name, position in tools.items()}
    return SphereState(torch.tensor(spheres)[None, None], SimpleNamespace(to_dict=lambda: poses))


class GrownWorldCostTests(unittest.TestCase):
    tools = {'left_tool0': (0., 0., 0.), 'right_tool0': (0., .5, 0.)}

    def test_only_grown_free_space_spheres_reach_the_world(self):
        # 1 cm gaps; sphere 2 is not a free-space sphere and keeps its original radius.
        state = sphere_state([[.89, 0., 0., .1], [.89, .5, 0., .1], [.89, -.5, 0., .1]], self.tools)
        cost = grown_validity([.02, .005], [0, 1])._grown_world_cost(state)
        self.assertGreater(float(cost[0, 0, 0]), 0.)
        self.assertEqual(float(cost[0, 0, 1]), 0.)
        self.assertEqual(float(cost[0, 0, 2]), 0.)

    def test_disabled_spheres_stay_disabled(self):
        state = sphere_state([[1.5, 0., 0., -100.], [1.5, .5, 0., 0.]], self.tools)
        self.assertEqual(float(grown_validity([.02, .02], [0, 1])._grown_world_cost(state).abs().sum()), 0.)

    def test_an_arm_near_its_contact_keeps_its_original_spheres(self):
        state = sphere_state([[.89, 0., 0., .1], [.89, .5, 0., .1]], self.tools)
        near = grown_validity([.02, .02], [0, 1], exempt=[(0, [(.05, 0., 0.)])])._grown_world_cost(state)
        self.assertEqual(float(near[0, 0, 0]), 0.)  # left tool 5 cm from its exempt point
        self.assertGreater(float(near[0, 0, 1]), 0.)  # the right arm is not exempt
        far = grown_validity([.02, .02], [0, 1], exempt=[(0, [(.07, 0., 0.)])])._grown_world_cost(state)
        self.assertGreater(float(far[0, 0, 0]), 0.)


class CrossArmMarginTests(unittest.TestCase):
    """A left and a right sphere 25 mm apart, coverage margins 20 mm (left) and 10 mm (right)."""

    def validity(self, growth_sides=(True, True), exempt=()):
        from curobo_core.door_sequential_validity import CollisionMargins
        validity = SequentialValidity.__new__(SequentialValidity)
        validity.margins = CollisionMargins(exempt_radius_m=.06)
        validity.world_growth = torch.zeros(0)
        validity.world_exempt = [None, None]
        for index, points in exempt:
            validity.world_exempt[index] = torch.tensor(points).reshape(-1, 3)
        validity.growth_sides = torch.tensor(growth_sides)
        validity.cross_pairs = torch.tensor([[0, 1]])
        validity.cross_margins = torch.tensor([[.02, .01]])
        validity.cross_block = None
        return validity

    def clash(self, growth_sides=(True, True), exempt=(), radius=.1):
        spheres = torch.tensor([[[0., 0., 0., radius], [.225, 0., 0., .1]]])
        tools = {'left_tool0': SimpleNamespace(position=torch.tensor([[[0., 0., .3]]])),
                 'right_tool0': SimpleNamespace(position=torch.tensor([[[.225, 0., .3]]]))}
        validity = self.validity(growth_sides, exempt)
        pairwise = bool(validity._cross_arm_clash(spheres, tools)[0])
        validity.cross_block = (torch.tensor([0]), torch.tensor([1]), torch.tensor([.02]), torch.tensor([.01]))
        self.assertEqual(bool(validity._cross_arm_clash(spheres, tools)[0]), pairwise)
        return pairwise

    def test_both_free_space_arms_need_the_sum_of_their_margins(self):
        self.assertTrue(self.clash())

    def test_a_static_or_exempt_arm_adds_no_margin(self):
        self.assertFalse(self.clash(growth_sides=(True, False)))  # 25 mm clear of 20 mm
        self.assertFalse(self.clash(growth_sides=(False, True), radius=.11))  # 15 mm clear of 10 mm
        self.assertTrue(self.clash(growth_sides=(False, True), radius=.12))  # 5 mm clear of 10 mm
        self.assertFalse(self.clash(exempt=[(0, [(0., 0., .32)])]))  # left tool0 2 cm from its contact: 10 mm

    def test_disabled_spheres_never_clash(self):
        self.assertFalse(self.clash(radius=-100.))

    def test_sphere_block_equals_the_pairwise_check(self):
        generator = torch.Generator().manual_seed(7)
        left, right = [0, 2, 3, 6], [1, 4, 5]
        left_margin, right_margin = torch.rand(4, generator=generator)*.03, torch.rand(3, generator=generator)*.03
        spheres = torch.rand((256, 7, 4), generator=generator)*torch.tensor([.6, .6, .6, .12])
        spheres[torch.rand((256, 7), generator=generator) < .1, 3] = -100.
        tools = {name: SimpleNamespace(position=torch.rand((1, 256, 3), generator=generator)*.6)
                 for name in ('left_tool0', 'right_tool0')}
        for growth_sides in ((True, True), (True, False), (False, False)):
            validity = self.validity(growth_sides, exempt=[(1, [(.3, .3, .3)])])
            validity.cross_pairs = torch.tensor([[i, j] for i in left for j in right])
            validity.cross_margins = torch.stack([left_margin.repeat_interleave(3), right_margin.repeat(4)], dim=-1)
            pairwise = validity._cross_arm_clash(spheres, tools)
            validity.cross_block = (torch.tensor(left), torch.tensor(right), left_margin, right_margin)
            block = validity._cross_arm_clash(spheres, tools)
            self.assertTrue(torch.equal(block, pairwise), growth_sides)
            self.assertTrue(0 < int(block.sum()) < len(block))


class CoverageMarginTests(unittest.TestCase):
    def test_margins_grow_each_sphere_to_cover_the_points_it_needs_least_growth_for(self):
        from curobo_core.coverage_margins import sphere_margins
        corners = np.array([[x, y, z] for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)])
        spheres = np.array([[-.25, 0., 0., .3], [.25, 0., 0., .3]])
        margins, need = sphere_margins(corners, spheres, .002)
        # Each corner belongs to the sphere on its side: |(.25, .5, .5)| - .3 to reach it.
        expected = np.linalg.norm([.25, .5, .5])-.3
        np.testing.assert_allclose(margins, expected+.002)
        np.testing.assert_allclose(need, expected)
        grown = spheres.copy()
        grown[:, 3] += margins
        reach = np.linalg.norm(corners[:, None]-grown[None, :, :3], axis=-1)-grown[None, :, 3]
        self.assertTrue((reach.min(1) <= 0).all())

    def test_covered_points_need_no_growth(self):
        from curobo_core.coverage_margins import sphere_margins
        margins, need = sphere_margins(np.zeros((4, 3)), np.array([[0., 0., 0., .1]]), 0.)
        np.testing.assert_array_equal(margins, [0.])
        self.assertTrue((need < 0).all())


class FreeSpaceScopeTests(unittest.TestCase):
    """Which margins a door validity gets: none without coverage margins, world growth only in a scope."""

    def task(self, sphere_margins):
        from curobo_core.door_recenter import DoorPlanner
        from curobo_core.door_sequential import _SequentialTask

        def transforms(rows, frame):
            # tool0 x = the side's first arm joint, so each row gives its own exempt point.
            output = np.tile(np.eye(4), (len(rows), 1, 1))
            output[:, 0, 3] = np.asarray(rows)[:, 1 if frame == 'left_tool0' else 8]
            return output
        task = _SequentialTask.__new__(_SequentialTask)
        task.planner = SimpleNamespace(
            sphere_margins=sphere_margins, coverage_margin_links=DoorPlanner.coverage_margin_links,
            coverage_self_pairs=DoorPlanner.coverage_self_pairs, coverage_payload_links=DoorPlanner.coverage_payload_links,
            coverage_cross_pairs=DoorPlanner.coverage_cross_pairs,
            free_space_exempt_radius_m=DoorPlanner.free_space_exempt_radius_m, batch_fk=SimpleNamespace(transforms=transforms))
        task.free_space_sides, task.free_space_exempt = (), ()
        return task

    def test_no_margins_without_coverage_margins(self):
        task = self.task(None)
        with task.free_space(('left', 'right'), np.zeros((1, 15))):
            self.assertIsNone(task.collision_margins())

    def test_world_growth_only_for_the_scoped_arms_and_restored_on_exit(self):
        task = self.task({})
        outside = task.collision_margins()
        self.assertEqual((outside.world, outside.exempt, outside.cross_pairs), ((), (), ()))
        self.assertEqual(outside.self_pairs, task.planner.coverage_self_pairs)
        self.assertEqual(outside.payload, task.planner.coverage_payload_links)
        rows = np.zeros((3, 15))
        rows[:, 1] = (.1, .2, .1)
        with task.free_space(('left',), rows):
            inside = task.collision_margins()
            self.assertEqual(inside.world, tuple(f'left_link{index}' for index in range(1, 8)))
            self.assertEqual(inside.exempt, (('left', ((.1, 0., 0.), (.2, 0., 0.))),))
            self.assertEqual(inside.cross_pairs, task.planner.coverage_cross_pairs)
            with task.free_space(()):
                self.assertEqual(task.collision_margins().world, ())
            self.assertEqual(task.collision_margins(), inside)
            hash(inside)  # part of the validity cache key
        self.assertEqual(task.collision_margins(), outside)


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


def loop_sphere_pairs(distances):
    """cuRobo's create_from_sphere_pair_distances pair loop (curobo/_src/robot/types/self_collision_params.py)."""
    pairs = []
    for i in range(len(distances)):
        if torch.max(distances[i]) == -torch.inf:
            continue
        pairs.extend((i, j) for j in range(i+1, len(distances)) if distances[i, j] != -torch.inf)
    return torch.tensor(pairs, dtype=torch.int16).reshape(-1, 2)


class SpherePairTests(unittest.TestCase):
    def test_tensor_pairs_equal_the_loop_in_order(self):
        generator = torch.Generator().manual_seed(4)
        for count in (1, 2, 7, 60):
            distances = torch.rand((count, count), generator=generator)
            distances[torch.rand((count, count), generator=generator) < .6] = -torch.inf
            distances[count//2] = -torch.inf  # a sphere without any pair
            distances[0, -1] = float('nan')   # the loop keeps NaN distances
            pairs = sphere_pairs(distances)
            self.assertEqual(pairs.dtype, torch.int16)
            self.assertTrue(torch.equal(pairs, loop_sphere_pairs(distances)), count)

    def test_no_pairs(self):
        self.assertEqual(tuple(sphere_pairs(torch.full((3, 3), -torch.inf)).shape), (0, 2))


class DenseEdgeCheckTests(unittest.TestCase):
    """The RRT's per-edge cached dense check decides every path as the whole-path check."""

    def test_cached_edges_decide_as_whole_paths(self):
        from v3_batched_loaded_search import DenseEdgeCheck
        rng = np.random.default_rng(5)

        def tree(count, roots):
            parents = [-1]*roots+[int(rng.integers(0, index)) for index in range(roots, count)]
            return torch.as_tensor(rng.uniform(-1., 1., (count, 15)), dtype=torch.float32), parents

        def walk(node, parents):
            nodes = []
            while node >= 0:
                nodes.append(int(node))
                node = parents[node]
            return nodes
        (start_tree, start_parents), (goal_tree, goal_parents) = tree(40, 1), tree(40, 3)
        # A thin slab of joint 1 (2 cm), blocked only where joint 2 is above 0.2.
        validity = SimpleNamespace(weights=torch.ones(15), mask=lambda rows: ~(
            (rows[:, 1] > .3) & (rows[:, 1] < .32) & (rows[:, 2] > .2)))
        whole, cached = (DenseEdgeCheck(validity, validity.weights, cache) for cache in (False, True))
        decisions = []
        for start_node, goal_node in rng.integers(0, 40, (60, 2)):
            paths = (start_tree, goal_tree, walk(start_node, start_parents), walk(goal_node, goal_parents))
            decisions.append(whole(*paths))
            self.assertEqual(cached(*paths), decisions[-1])
        self.assertTrue(any(decisions) and not all(decisions))
        self.assertLess(cached.states, whole.states)


class CompactEdgeTests(unittest.TestCase):
    """GpuValidity.edges over each edge's own samples returns what the padded grid returns."""

    def validity(self, compact):
        from v3_batched_loaded_search import GpuValidity

        def evaluate(values):
            self.states += len(values)
            stability = (values[:, 3]*7.).sin().abs()
            return ~((values[:, 1] > .3) & (values[:, 1] < .32) & (values[:, 2] > 0.)), stability
        validity = SimpleNamespace(weights=torch.ones(15), minimum_edge_steps=2, compact_edges=compact,
                                   evaluate=evaluate)
        validity._compact_edges = lambda *args: GpuValidity._compact_edges(validity, *args)
        validity.edges = lambda *args, **kwargs: GpuValidity.edges(validity, *args, **kwargs)
        return validity

    def test_compact_edges_match_the_padded_grid(self):
        rng = np.random.default_rng(11)
        starts = torch.as_tensor(rng.uniform(-1., 1., (64, 15)), dtype=torch.float32)
        goals = starts+torch.as_tensor(rng.uniform(-1., 1., (64, 15))*rng.uniform(.01, 1., (64, 1)), dtype=torch.float32)
        counts = {}
        for compact in (False, True):
            self.states = 0
            validity = self.validity(compact)
            valid, stability = validity.edges(starts, goals, return_stability=True)
            counts[compact] = self.states
            if compact:
                self.assertTrue(torch.equal(valid, padded[0]) and torch.equal(stability, padded[1]))
                self.assertTrue(torch.equal(validity.edges(starts, goals), padded[0]))
            padded = valid, stability
        self.assertTrue(padded[0].any() and not padded[0].all())
        self.assertLess(counts[True], counts[False])


if __name__ == '__main__':
    unittest.main()
