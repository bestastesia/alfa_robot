from itertools import combinations
import json
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from curobo_core.fixtures import tasks
from v3_box_geometry import (BOX_GAP_M, CONTAINER_HALF_WIDTH_M, DEFAULT_BOX_DIMS, EXTRACTION_M, SIZE_LEVELS,
                             box_dims, box_sphere_model, extraction_distance_m, extraction_steps, pitch,
                             sphere_model_metrics, wall_center)
import v3_task_scene
import v3_wall_ik_benchmark

FROZEN = json.loads((ROOT / 'generated/v3_suction_v322_6bb184b/frozen_collision_model/'
                     'box_voxel_edge_corner_final.json').read_text())
ALL_TASKS = [pair for _, pair in tasks() + tasks([1, 0])] + [{'left': i} for i in range(25)] + [{'right': i} for i in range(25)]


def legacy_wall_center(front, distance, box_id):
    return np.array([front + distance + 0.15, (box_id % 5 - 2) * 0.41, 0.20 + (box_id // 5) * 0.41])


def legacy_row_obstacles(front, boxes):
    centers = {side: legacy_wall_center(front, 0.9, box_id) for side, box_id in boxes.items()}
    for center in centers.values():
        center[2] += 0.001
    lowest = min(centers.values(), key=lambda center: center[2])
    intervals = [(-1.02, 1.02)]
    for center in centers.values():
        if abs(center[2] - lowest[2]) > 1e-6:
            continue
        remaining = []
        cut_lower, cut_upper = center[1] - 0.2, center[1] + 0.2
        for lower, upper in intervals:
            if cut_upper <= lower or cut_lower >= upper:
                remaining.append((lower, upper))
            else:
                if cut_lower > lower:
                    remaining.append((lower, cut_lower))
                if cut_upper < upper:
                    remaining.append((cut_upper, upper))
        intervals = remaining
    obstacles = [dict(center=[lowest[0], (lower + upper) / 2, lowest[2]],
                      dimensions=[0.3, upper - lower, 0.4]) for lower, upper in intervals if upper - lower > 1e-8]
    bottom_height = lowest[2] - 0.2
    if bottom_height - 0.001 > 1e-8:
        obstacles.append(dict(center=[lowest[0], 0.0, (bottom_height + 0.001) / 2], dimensions=[0.3, 2.04, bottom_height - 0.001]))
    return centers, obstacles


def overlap(center_a, half_a, center_b, half_b):
    return np.minimum(center_a + half_a, center_b + half_b) - np.maximum(center_a - half_a, center_b - half_b)


class DefaultPathIsLegacyTest(unittest.TestCase):
    """Default dims must reproduce the legacy literals bit for bit (L0 unchanged)."""

    def test_wall_centers(self):
        for front in (0.0, 0.2, 0.3712):
            for box_id in range(25):
                reference = legacy_wall_center(front, 0.9, box_id)
                for value in (wall_center(front, 0.9, box_id), v3_task_scene.wall_center(front, 0.9, box_id),
                              v3_wall_ik_benchmark.wall_center(front, 0.9, box_id)):
                    self.assertTrue(np.array_equal(value, reference))

    def test_row_obstacles_for_every_task(self):
        for boxes in ALL_TASKS:
            centers, obstacles = v3_task_scene.row_obstacles(0.2, boxes)
            reference_centers, reference_obstacles = legacy_row_obstacles(0.2, boxes)
            self.assertEqual(obstacles, reference_obstacles)
            for side in boxes:
                self.assertTrue(np.array_equal(centers[side], reference_centers[side]))

    def test_grasp_transform_and_tool_to_box(self):
        side = np.eye(4)
        side[:3, :3] = [[0, 0, 1], [0, -1, 0], [1, 0, 0]]
        side[2, 3] = 0.15
        top = np.eye(4)
        top[:3, :3] = np.diag([1.0, -1.0, -1.0])
        top[2, 3] = 0.2
        for name in ('left', 'right'):
            self.assertTrue(np.array_equal(v3_task_scene.grasp_transform(name, False), side))
            self.assertTrue(np.array_equal(v3_task_scene.grasp_transform(name, True), top))
            self.assertEqual(v3_wall_ik_benchmark.canonical_side_tool_to_box(name)[2, 3], 0.15)

    def test_make_scene(self):
        scene = v3_wall_ik_benchmark.make_scene(0.2, 0.9, {3})
        explicit = v3_wall_ik_benchmark.make_scene(0.2, 0.9, {3}, DEFAULT_BOX_DIMS)
        self.assertEqual([(c.name, list(c.pose), list(c.dims)) for c in scene.cuboid],
                         [(c.name, list(c.pose), list(c.dims)) for c in explicit.cuboid])
        boxes = {c.name: c for c in scene.cuboid if c.name.startswith('wall_box_')}
        self.assertEqual(len(boxes), 24)
        for name, cuboid in boxes.items():
            self.assertEqual(list(cuboid.dims), [0.30, 0.40, 0.40])
            self.assertEqual(list(cuboid.pose[:3]), legacy_wall_center(0.2, 0.9, int(name[-2:])).tolist())
        front = next(c for c in scene.cuboid if c.name == 'front_wall')
        self.assertEqual(front.pose[0], 0.2 + 0.9 + 0.30 + 1e-6 + 0.05)

    def test_extraction(self):
        self.assertEqual(extraction_distance_m(), EXTRACTION_M)
        self.assertTrue(np.array_equal(extraction_steps(), np.linspace(.01, .35, 35)))
        self.assertTrue(np.array_equal(extraction_steps(FROZEN['dimensions_m']), np.linspace(.01, .35, 35)))


class BoxSizeSceneTest(unittest.TestCase):
    def test_levels_are_valid_and_include_default(self):
        self.assertIn(DEFAULT_BOX_DIMS, SIZE_LEVELS.values())
        self.assertEqual(len(set(SIZE_LEVELS.values())), len(SIZE_LEVELS))
        for dims in SIZE_LEVELS.values():
            self.assertEqual(box_dims(dims), dims)

    def test_invalid_dims_are_rejected(self):
        for value in ((0.3, 0.45, 0.4), (0.3, 0.4, 0.48), (0.0, 0.4, 0.4), (0.3, float('nan'), 0.4),
                      (0.3, 0.4), (0.6, 0.4, 0.4)):
            with self.assertRaises(ValueError):
                box_dims(value)

    def test_wall_boxes_disjoint_inside_container_on_ground(self):
        for dims in SIZE_LEVELS.values():
            half = np.asarray(dims) / 2
            scene = v3_wall_ik_benchmark.make_scene(0.2, 0.9, set(), dims)
            boxes = [c for c in scene.cuboid if c.name.startswith('wall_box_')]
            self.assertEqual(len(boxes), 25)
            centers = [np.asarray(c.pose[:3]) for c in boxes]
            for cuboid, center in zip(boxes, centers):
                self.assertEqual(tuple(cuboid.dims), dims)
                self.assertAlmostEqual(center[0] - half[0], 0.2 + 0.9)
                self.assertLessEqual(abs(center[1]) + half[1], CONTAINER_HALF_WIDTH_M + 1e-9)
                self.assertGreaterEqual(center[2] - half[2], -1e-12)
                self.assertLess(center[2] + half[2], 2.40)
            for first, second in combinations(centers, 2):
                gaps = overlap(first, half, second, half)
                self.assertTrue(np.any(gaps <= -BOX_GAP_M + 1e-9))
            wall = next(c for c in scene.cuboid if c.name == 'front_wall')
            self.assertGreaterEqual(wall.pose[0] - wall.dims[0] / 2, 0.2 + 0.9 + dims[0])

    def test_pitch_is_face_plus_gap(self):
        for dims in SIZE_LEVELS.values():
            self.assertAlmostEqual(pitch(dims)[0], dims[1] + BOX_GAP_M, places=12)
            self.assertAlmostEqual(pitch(dims)[1], dims[2] + BOX_GAP_M, places=12)

    def test_fill_never_intersects_targets_and_covers_target_row(self):
        for dims in SIZE_LEVELS.values():
            half = np.asarray(dims) / 2
            for boxes in ALL_TASKS:
                centers, obstacles = v3_task_scene.row_obstacles(0.2, boxes, dims)
                for side, box_id in boxes.items():
                    self.assertTrue(np.allclose(centers[side] - (0, 0, 0.001), wall_center(0.2, 0.9, box_id, dims)))
                lowest = min(center[2] for center in centers.values())
                row_cover = []
                for obstacle in obstacles:
                    center, extent = np.asarray(obstacle['center']), np.asarray(obstacle['dimensions']) / 2
                    self.assertTrue(np.all(extent > 0))
                    self.assertGreaterEqual(center[2] - extent[2], 0)
                    self.assertAlmostEqual(extent[0], half[0])
                    for target in centers.values():
                        self.assertFalse(np.all(overlap(center, extent, target, half) > 1e-8))
                    if abs(center[2] - lowest) < 1e-9:
                        self.assertAlmostEqual(extent[2], half[2])
                        row_cover.append((center[1] - extent[1], center[1] + extent[1]))
                for center in centers.values():
                    if abs(center[2] - lowest) < 1e-9:
                        row_cover.append((center[1] - half[1], center[1] + half[1]))
                row_cover.sort()
                self.assertAlmostEqual(row_cover[0][0], -CONTAINER_HALF_WIDTH_M)
                self.assertAlmostEqual(row_cover[-1][1], CONTAINER_HALF_WIDTH_M)
                for previous, following in zip(row_cover, row_cover[1:]):
                    self.assertLessEqual(following[0], previous[1] + 1e-9)

    def test_grasp_offsets_put_tool_on_the_suction_face(self):
        for dims in SIZE_LEVELS.values():
            for top in (False, True):
                transform = v3_task_scene.grasp_transform('left', top, dims)
                self.assertAlmostEqual(transform[2, 3], dims[2] / 2 if top else dims[0] / 2)
                tool = np.eye(4)
                tool[:3, :3] = np.diag([1, -1, -1]) if top else [[0, 0, 1], [0, -1, 0], [1, 0, 0]]
                box = tool @ transform
                face = box[:3, 3] + (np.array([0, 0, dims[2] / 2]) if top else np.array([-dims[0] / 2, 0, 0]))
                self.assertTrue(np.allclose(face, tool[:3, 3], atol=1e-12))
            self.assertAlmostEqual(v3_wall_ik_benchmark.canonical_side_tool_to_box('left', dims)[2, 3], dims[0] / 2)

    def test_extraction_clears_the_box_depth(self):
        for dims in SIZE_LEVELS.values():
            distance = extraction_distance_m(dims)
            steps = extraction_steps(dims)
            self.assertGreaterEqual(distance, dims[0] + 0.05 - 1e-9)
            self.assertGreaterEqual(distance, EXTRACTION_M)
            if dims[0] <= 0.30:
                self.assertEqual(distance, EXTRACTION_M)
            self.assertAlmostEqual(steps[0], 0.01)
            self.assertEqual(steps[-1], distance)
            self.assertTrue(np.allclose(np.diff(steps), 0.01))


class SphereModelTest(unittest.TestCase):
    def test_default_dims_contain_every_frozen_sphere(self):
        model = box_sphere_model(DEFAULT_BOX_DIMS)
        generated = np.column_stack([model['centers'], model['radii']])
        for center, radius in zip(FROZEN['centers'], FROZEN['radii']):
            distances = np.abs(generated - np.r_[center, radius]).max(axis=1)
            self.assertLess(distances.min(), 2e-6)
        self.assertEqual(model['sphere_groups'].count('edge'), 12)
        self.assertEqual(model['sphere_groups'].count('corner'), 8)

    def test_all_levels_inscribed_deterministic_and_compatible(self):
        for dims in SIZE_LEVELS.values():
            model = box_sphere_model(dims)
            self.assertEqual(model, box_sphere_model(dims))
            self.assertEqual(tuple(model['dimensions_m']), dims)
            self.assertEqual(len(model['centers']), len(model['radii']))
            self.assertEqual(model['actual_spheres'], len(model['radii']))
            self.assertTrue({'name', 'parameters', 'metrics', 'sphere_groups'} <= set(model))
            half = np.asarray(dims) / 2
            centers, radii = np.asarray(model['centers']), np.asarray(model['radii'])
            self.assertTrue(np.all(radii > 0))
            self.assertLessEqual(float(np.max(np.abs(centers) + radii[:, None] - half)), 1e-12)
            self.assertEqual(model['metrics']['protrusion'], 0.0)
            model['centers'][0][0] += 1.
            self.assertNotEqual(model, box_sphere_model(dims))

    def test_coverage_not_worse_than_frozen_model(self):
        frozen = sphere_model_metrics(FROZEN, samples=60000, seed=5)
        self.assertAlmostEqual(frozen['coverage'], FROZEN['metrics']['coverage'], delta=0.01)
        self.assertAlmostEqual(frozen['surface_gap_p95_mm'], FROZEN['metrics']['surface_gap_p95_mm'], delta=2.)
        default = sphere_model_metrics(box_sphere_model(DEFAULT_BOX_DIMS), samples=60000, seed=5)
        self.assertGreaterEqual(default['coverage'], frozen['coverage'])
        for dims in SIZE_LEVELS.values():
            metrics = sphere_model_metrics(box_sphere_model(dims), samples=30000, seed=6)
            self.assertGreaterEqual(metrics['coverage'], frozen['coverage'])
            self.assertLess(metrics['surface_gap_p95_mm'], 1000 * min(dims) * 0.25)


if __name__ == '__main__':
    unittest.main()
