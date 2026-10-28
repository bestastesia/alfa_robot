from dataclasses import dataclass
import json
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from curobo_core.adapter import pose_matrix
from curobo_core.fixtures import tasks
from motion311_learning.stage_env import configure_planner
from v3_box_geometry import DEFAULT_BOX_DIMS, SIZE_LEVELS, SPHERE_MODEL_NAME, wall_center
from v3_conveyor_cycle_planner import ConveyorCyclePlanner
from v3_task_cycle_planner import TaskCyclePlanner
from v3_task_scene import grasp_transform, row_obstacles

FROZEN = json.loads((ROOT / 'generated/v3_suction_v322_6bb184b/frozen_collision_model/'
                     'box_voxel_edge_corner_final.json').read_text())
SIDE_PAIR = tasks()[0][1]
TOP_PAIR = next(pair for _, pair in tasks([1, 0]) if all(box_id // 5 <= 1 for box_id in pair.values()))


@dataclass(frozen=True)
class FakeSnapshot:
    objects: tuple = ()
    revision: int = 0


def fake_planner(box_dims):
    return SimpleNamespace(task_map=dict(tasks() + tasks([1, 0])), front=0.2, box_fit=FROZEN, legacy_box_fit=FROZEN,
                           box_dims=box_dims, original_snapshot=FakeSnapshot())


class ConfigureTaskTest(unittest.TestCase):
    def test_legacy_instance_keeps_frozen_model_and_scene(self):
        planner = fake_planner(None)
        centers, obstacles = TaskCyclePlanner.configure_task(planner, SIDE_PAIR)
        self.assertIs(planner.box_fit, FROZEN)
        legacy_centers, legacy_obstacles = row_obstacles(0.2, SIDE_PAIR)
        self.assertEqual(obstacles, legacy_obstacles)
        objects = {item.object_id: item for item in planner.snapshot.objects}
        box = objects[f"wall_box_{SIDE_PAIR['left']:02d}"]
        self.assertEqual(planner.base_advance, 0.0)
        self.assertEqual(box.dimensions_m, DEFAULT_BOX_DIMS)
        self.assertEqual(box.pose.position, tuple(legacy_centers['left']))

    def test_sized_instance_swaps_model_scene_and_attachments(self):
        for dims in SIZE_LEVELS.values():
            for pair in (SIDE_PAIR, TOP_PAIR):
                planner = fake_planner(dims)
                TaskCyclePlanner.configure_task(planner, pair)
                self.assertEqual(planner.box_fit['name'], SPHERE_MODEL_NAME)
                self.assertEqual(tuple(planner.box_fit['dimensions_m']), dims)
                self.assertIs(planner.legacy_box_fit, FROZEN)
                objects = {item.object_id: item for item in planner.snapshot.objects}
                advance = planner.base_advance
                for box_id in pair.values():
                    box = objects[f'wall_box_{box_id:02d}']
                    self.assertEqual(box.dimensions_m, dims)
                    expected = wall_center(0.2, 0.9, box_id, dims) + (-advance, 0, 0.001)
                    self.assertTrue(np.allclose(box.pose.position, expected))
                front = objects['front_wall']
                self.assertAlmostEqual(front.pose.position[0] - front.dimensions_m[0] / 2,
                                       0.2 + 0.9 + dims[0] + 1e-6 - advance)
                for name, item in objects.items():
                    if name.startswith('task_fill_'):
                        self.assertAlmostEqual(item.dimensions_m[0], dims[0])
                for attachment in TaskCyclePlanner.task_attachments(planner, pair):
                    self.assertEqual(attachment.dimensions_m, dims)
                    offset = pose_matrix(attachment.tool_to_object)[2, 3]
                    self.assertAlmostEqual(offset, dims[2] / 2 if planner.top_grasp else dims[0] / 2)

    def test_reconfiguring_restores_legacy_model(self):
        planner = fake_planner(SIZE_LEVELS['S40x40x45'])
        TaskCyclePlanner.configure_task(planner, SIDE_PAIR)
        planner.box_dims = None
        TaskCyclePlanner.configure_task(planner, SIDE_PAIR)
        self.assertIs(planner.box_fit, FROZEN)


class ConveyorPlaceTargetTest(unittest.TestCase):
    def test_box_rests_at_conveyor_height_for_every_size(self):
        tool = np.eye(4)
        tool[:3, :3] = [[0, 0, 1], [0, -1, 0], [1, 0, 0]]
        tool[:3, 3] = (0.6, 0.425, 1.0)
        for dims in SIZE_LEVELS.values():
            planner = SimpleNamespace(tool_matrices=lambda start: {'left': tool.copy()}, top_grasp=False,
                                      box_fit=dict(dimensions_m=list(dims)), conveyor_push_m=ConveyorCyclePlanner.conveyor_push_m,
                                      conveyor_height_m=ConveyorCyclePlanner.conveyor_height_m,
                                      placement_clearance_m=ConveyorCyclePlanner.placement_clearance_m)
            targets, boxes = ConveyorCyclePlanner.conveyor_place_targets(planner, None)
            bottom = boxes['left'][2, 3] - dims[2] / 2
            self.assertAlmostEqual(bottom, ConveyorCyclePlanner.conveyor_height_m + ConveyorCyclePlanner.placement_clearance_m)
            self.assertTrue(np.allclose(targets['left'] @ grasp_transform('left', False, dims), boxes['left']))
            carried = tool @ grasp_transform('left', False, dims)
            self.assertAlmostEqual(boxes['left'][0, 3] - carried[0, 3], ConveyorCyclePlanner.conveyor_push_m)


class StageEnvThreadingTest(unittest.TestCase):
    def test_instance_box_dims_reach_the_planner_before_configuration(self):
        seen = []
        planner = SimpleNamespace(configure_task=lambda boxes: seen.append((planner.scene_offset, planner.box_dims, boxes)))
        configure_planner(planner, dict(offset_m=[0.01, -0.02], box_dims=[0.25, 0.30, 0.30]), {'right': 3})
        configure_planner(planner, dict(offset_m=[0.0, 0.0]), {'right': 3})
        self.assertEqual(seen, [((0.01, -0.02), (0.25, 0.30, 0.30), {'right': 3}), ((0.0, 0.0), None, {'right': 3})])
        with self.assertRaises(ValueError):
            configure_planner(planner, dict(offset_m=[0.0, 0.0], box_dims=[0.3, 0.5, 0.4]), {'right': 3})


if __name__ == '__main__':
    unittest.main()
