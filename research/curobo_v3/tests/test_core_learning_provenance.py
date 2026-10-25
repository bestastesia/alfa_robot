from pathlib import Path
import copy
import hashlib
import json
import sys
import tempfile
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from motion311_learning.provenance import (
    _default_paths, assert_fingerprint, fingerprint, stream_hash,
)


class LearningProvenanceTest(unittest.TestCase):
    def fixture(self, folder):
        root = Path(folder)
        registry, code, asset, library = [root / name for name in ('registry.json', 'stage.py', 'robot.urdf', 'analytic.so')]
        registry.write_text('{"instances": []}')
        code.write_text('stage = 0\n')
        asset.write_text('<robot name="fixture"/>')
        library.write_bytes(b'fixture-only-binary')
        paths = {'code/stage': code, 'asset/urdf': asset, 'asset/analytic_so': library}
        return registry, paths

    def test_stream_hash_is_exact_and_uses_bounded_chunks(self):
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'large.bin'
            content = bytes(range(256)) * 19000
            path.write_bytes(content)
            with patch.object(Path, 'read_bytes', side_effect=AssertionError('stream, do not read the whole file')):
                digest, size = stream_hash(path, chunk_size=113)
            self.assertEqual(digest, hashlib.sha256(content).hexdigest())
            self.assertEqual(size, len(content))
            with self.assertRaises(ValueError):
                stream_hash(path, chunk_size=0)

    def test_mapping_order_and_repeat_runs_are_stable(self):
        with tempfile.TemporaryDirectory() as folder:
            registry, paths = self.fixture(folder)
            first = fingerprint(registry, paths=paths)
            second = fingerprint(registry, paths=dict(reversed(list(paths.items()))))
            self.assertEqual(first, second)
            self.assertTrue(first['prepared'])
            self.assertIsNone(assert_fingerprint(first, second))
            self.assertEqual(list(first['files']), sorted(first['files']))

    def test_registry_code_urdf_and_shared_library_changes_are_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            registry, paths = self.fixture(folder)
            baseline = fingerprint(registry, paths=paths)
            for path in [registry, *paths.values()]:
                original = path.read_bytes()
                path.write_bytes(original + b'changed')
                current = fingerprint(registry, paths=paths)
                self.assertNotEqual(current['digest'], baseline['digest'])
                with self.assertRaisesRegex(ValueError, 'changed'):
                    assert_fingerprint(baseline, current)
                path.write_bytes(original)
            assert_fingerprint(baseline, fingerprint(registry, paths=paths))

    def test_missing_analytic_is_explicitly_not_prepared_and_cannot_match(self):
        with tempfile.TemporaryDirectory() as folder:
            registry, paths = self.fixture(folder)
            paths['asset/analytic_so'].unlink()
            current = fingerprint(registry, paths=paths)
            entry = current['files']['asset/analytic_so']
            self.assertEqual(entry['status'], 'not_prepared')
            self.assertIsNone(entry['sha256'])
            self.assertIsNone(entry['size_bytes'])
            self.assertFalse(current['prepared'])
            with self.assertRaisesRegex(ValueError, 'not prepared'):
                assert_fingerprint(current, copy.deepcopy(current))
            paths['asset/analytic_so'].write_bytes(b'newly-built-library')
            prepared = fingerprint(registry, paths=paths)
            self.assertTrue(prepared['prepared'])
            with self.assertRaises(ValueError):
                assert_fingerprint(current, prepared)

    def test_missing_regular_asset_malformed_digest_and_empty_manifest_fail_closed(self):
        with tempfile.TemporaryDirectory() as folder:
            registry, paths = self.fixture(folder)
            expected = fingerprint(registry, paths=paths)
            tampered = copy.deepcopy(expected)
            tampered['files']['asset/urdf']['sha256'] = '0' * 64
            with self.assertRaisesRegex(ValueError, 'digest'):
                assert_fingerprint(expected, tampered)
            with self.assertRaises(ValueError):
                assert_fingerprint({}, {})
            paths['asset/urdf'].unlink()
            missing = fingerprint(registry, paths=paths)
            self.assertEqual(missing['files']['asset/urdf']['status'], 'missing_required')
            with self.assertRaisesRegex(ValueError, 'not prepared'):
                assert_fingerprint(expected, missing)

    def test_default_manifest_static_import_closure_and_nested_assets(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            def put(name, content):
                path = root / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(content)
                return path
            registry = put('config/registry.json', '{"instances":[]}')
            for name in ('stage_env.py', 'redundancy.py', 'contracts.py', 'features.py'):
                put('tools/motion311_learning/' + name, 'from curobo_core.planner import Planner\n')
            put('tools/curobo_core/__init__.py', 'raise RuntimeError("must never import planner package")\n')
            put('tools/curobo_core/planner.py', 'from .trajectory import helper\nimport v3_cpu_helper\nraise RuntimeError("must never execute")\n')
            put('tools/curobo_core/backend.py', 'pass\n')
            put('tools/curobo_core/trajectory.py', 'helper = 1\n')
            put('tools/v3_cpu_helper.py', 'import missing_third_party\n')
            put('tools/v3_analytic_bridge.cpp', '// fixture\n')
            put('tools/curobo_core/contracts.py', '\n'.join([
                'from pathlib import Path', 'class PlannerAssets:',
                '    robot_config: Path = WORKSPACE_ROOT / "assets/robot.yml"',
                '    mobile_robot_config: Path = WORKSPACE_ROOT / "assets/mobile.yml"',
                '    urdf: Path = WORKSPACE_ROOT / "assets/model.urdf"',
                '    named_poses: Path = WORKSPACE_ROOT / "assets/poses.yaml"',
                '    box_fit: Path = WORKSPACE_ROOT / "assets/box.json"',
                '    target_poses: Path = WORKSPACE_ROOT / "assets/target.json"',
            ]))
            put('assets/robot.yml', 'kinematics:\n  urdf_path: model.urdf\n  collision_spheres: spheres.yaml\n')
            put('assets/mobile.yml', 'robot_cfg:\n  kinematics:\n    urdf_path: mobile.urdf\n')
            put('assets/spheres.yaml', 'collision_spheres: {}\n')
            put('assets/poses.yaml', 'named_poses: {}\n')
            put('assets/box.json', '{}')
            put('assets/target.json', '{}')
            put('assets/model.urdf', '<robot><link name="base"><collision><geometry><mesh filename="collision.stl"/></geometry></collision></link></robot>')
            put('assets/mobile.urdf', '<robot name="mobile"/>')
            mesh = put('assets/collision.stl', 'solid fixture\nendsolid fixture\n')
            for name in ('placement_task_policy.json', 'base_link_collision_frozen.json', 'arm_carriage_collision_frozen.json'):
                put('config/' + name, '{}')
            put('generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json', '{}')
            put('models/conveyor/roller_conveyor_collision_solid_v2.stl', 'fixture')
            put('generated/v3_analytic_071cb95/libv3_analytic_bridge.so', 'fixture')
            current = fingerprint(registry, code_root=root)
            self.assertTrue(current['prepared'])
            files = current['files']
            self.assertIn('code/tools/v3_cpu_helper.py', files)
            self.assertIn('code/tools/curobo_core/trajectory.py', files)
            self.assertTrue(any(row['path'].endswith('/spheres.yaml') for row in files.values()))
            self.assertTrue(any(row['path'] == str(mesh) for row in files.values()))
            self.assertIsNone(assert_fingerprint(current, fingerprint(registry, code_root=root)))
            mesh.write_text('changed mesh bytes')
            with self.assertRaisesRegex(ValueError, 'changed'):
                assert_fingerprint(current, fingerprint(registry, code_root=root))

    def test_unresolved_asset_uri_is_not_silently_skipped(self):
        from motion311_learning.provenance import _linked_assets
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / 'robot.urdf'
            path.write_text('<robot><mesh filename="package://robot/mesh.stl"/></robot>')
            with self.assertRaisesRegex(ValueError, 'unresolved asset URI'):
                _linked_assets({'urdf': path})

    def test_paths_cannot_override_registry(self):
        with tempfile.TemporaryDirectory() as folder:
            registry, paths = self.fixture(folder)
            paths['registry'] = paths['asset/urdf']
            with self.assertRaisesRegex(ValueError, 'override'):
                fingerprint(registry, paths=paths)


if __name__ == '__main__':
    unittest.main()
