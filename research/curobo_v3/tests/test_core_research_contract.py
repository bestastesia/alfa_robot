import ast
import hashlib
import json
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]


class ResearchContractTest(unittest.TestCase):
    def test_freeze_manifest_matches_every_runtime_asset(self):
        manifest = json.loads((ROOT / 'config/research_asset_manifest.json').read_text())
        for item in manifest['assets']:
            self.assertEqual(hashlib.sha256((ROOT / item['path']).read_bytes()).hexdigest(), item['sha256'])

    def test_placement_and_base_geometry_contracts(self):
        policy = json.loads((ROOT / 'config/placement_task_policy.json').read_text())
        self.assertEqual(policy['side']['position'], {'x': .55, 'y_left': .425, 'y_right': -.425, 'z': 1.4})
        self.assertEqual(policy['side']['quaternion_wxyz'], [0, 0, 0, 1])
        self.assertEqual(policy['top']['position']['z'], 1)
        self.assertFalse(policy['turn_collision_check'])
        balls = json.loads((ROOT / 'config/base_link_collision_frozen.json').read_text())
        self.assertEqual(len(balls['collision_spheres']), 71)
        cuboids = json.loads((ROOT / 'generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json').read_text())
        self.assertEqual(cuboids['frame'], 'base_link')
        self.assertEqual(len(cuboids['cuboids']), 4)

    def test_mixed_planner_does_not_mutate_process_or_global_planner(self):
        path = ROOT / 'tools/v3_mixed_placement_planner.py'
        text = path.read_text()
        ast.parse(text)
        self.assertNotIn('os.chdir', text)
        self.assertNotIn('full_module.prepare_roots=', text)
        self.assertNotIn('/mnt/mydisk/ALFA/alfa_robot_v3_curobo', text)
        self.assertIn('joint_distance_weights(ACTIVE_JOINTS)', text)


if __name__ == '__main__':
    unittest.main()
