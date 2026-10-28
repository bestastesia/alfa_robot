from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'tools'))
from motion306_registry import HELDOUT_LEVELS, PATH, build_size_registry
from v3_box_geometry import DEFAULT_BOX_DIMS, SIZE_LEVELS, box_dims
from v3_scene_variants import build_registry, load_registry


class SizeRegistryTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.base = build_registry()
        cls.registry = build_size_registry()

    def test_reproducible_and_committed_file_is_current(self):
        self.assertEqual(build_size_registry()['sha256'], self.registry['sha256'])
        self.assertEqual(load_registry(PATH)['sha256'], self.registry['sha256'])  # digest covers the whole body
        self.assertEqual(self.registry['base_registry_sha256'], load_registry(ROOT / 'config/motion311_registry.json')['sha256'])

    def test_every_base_instance_at_every_level_with_inherited_fields(self):
        instances = self.registry['instances']
        self.assertEqual(len(instances), len(self.base['instances']) * len(SIZE_LEVELS))
        self.assertEqual(len({item['instance'] for item in instances}), len(instances))
        base = {item['instance']: item for item in self.base['instances']}
        for item in instances:
            parent = base[item['base_instance']]
            self.assertEqual(item['instance'], f"{parent['instance']}-{item['box_level']}")
            for key, value in parent.items():
                if key != 'instance':
                    self.assertEqual(item[key], value)
            self.assertEqual(tuple(item['box_dims']), SIZE_LEVELS[item['box_level']])
            self.assertEqual(box_dims(item['box_dims']), SIZE_LEVELS[item['box_level']])

    def test_heldout_sizes_are_never_seen(self):
        self.assertTrue(set(HELDOUT_LEVELS) < set(SIZE_LEVELS))
        default = next(level for level, dims in SIZE_LEVELS.items() if dims == DEFAULT_BOX_DIMS)
        self.assertNotIn(default, HELDOUT_LEVELS)
        for item in self.registry['instances']:
            self.assertEqual(item['size_split'], 'heldout' if item['box_level'] in HELDOUT_LEVELS else 'seen')
        self.assertEqual(self.registry['heldout_levels'], list(HELDOUT_LEVELS))


    def test_heldout_levels_interpolate_and_recombine_seen_sizes(self):
        seen = [dims for level, dims in SIZE_LEVELS.items() if level not in HELDOUT_LEVELS]
        low, high = np.min(seen, axis=0), np.max(seen, axis=0)
        for level in HELDOUT_LEVELS:
            dims = SIZE_LEVELS[level]
            self.assertTrue(np.all(np.asarray(dims) >= low - 1e-12) and np.all(np.asarray(dims) <= high + 1e-12))
            self.assertNotIn(dims, seen)
        interpolation, combination = (SIZE_LEVELS[level] for level in HELDOUT_LEVELS)
        values = [{dims[axis] for dims in seen} for axis in range(3)]
        self.assertTrue(all(value not in values[axis] for axis, value in enumerate(interpolation)))
        self.assertTrue(all(value in values[axis] for axis, value in enumerate(combination)))


if __name__ == '__main__':
    unittest.main()
