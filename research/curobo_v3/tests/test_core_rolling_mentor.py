"""PR65 integration contracts without loading CUDA or changing Mentor core files."""
import ast
from dataclasses import replace
from pathlib import Path
import sys
import types
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from curobo_core.contracts import ACTIVE_JOINTS
from curobo_core.scene import Pose, RobotState, SceneSnapshot
from v3_rolling_replan import stage_input


class MentorRollingContractTest(unittest.TestCase):
    def record(self):
        names = ['base_x', 'base_y', 'base_yaw', *ACTIVE_JOINTS]
        q = [[-.1, 0., np.pi, *([0.] * 15)], [-.1, 0., np.pi, *([.1] * 15)]]
        state = RobotState(tuple(ACTIVE_JOINTS), tuple([0.] * 15), 1,
                           Pose((-.1, 0., 0.), (0., 0., 0., 1.)))
        snapshot = SceneSnapshot('mentor-model', state, ())
        stage = dict(name='return_home', joint_names=names, positions=q, time_from_start_s=[0., 2.],
                     velocities=[[0.] * 18] * 2, accelerations=[[0.] * 18] * 2, jerks=[[0.] * 18] * 2)
        return dict(result=dict(success=True, task={'left': 24, 'right': 20}, timed_segments=[stage],
                                rolling_stage_inputs={'return_home': {'snapshot': snapshot.to_dict()}}))

    def test_runtime_fingerprint_includes_inherited_mentor_files(self):
        import tempfile
        from v3_rolling_replan import runtime_source_hashes
        with tempfile.TemporaryDirectory() as folder:
            tools = Path(folder)
            (tools / 'curobo_core').mkdir()
            (tools / 'curobo_core/rolling_backend.py').write_text('adapter = 1\n')
            planner = tools / 'v3_single_arm_planner.py'
            planner.write_text('planner = 1\n')
            before = runtime_source_hashes(tools)
            self.assertEqual(set(before), {'v3_single_arm_planner.py', 'curobo_core/rolling_backend.py'})
            planner.write_text('planner = 2\n')
            after = runtime_source_hashes(tools)
            self.assertNotEqual(before['v3_single_arm_planner.py'], after['v3_single_arm_planner.py'])
            self.assertEqual(before['curobo_core/rolling_backend.py'], after['curobo_core/rolling_backend.py'])

    def test_phase_contract_cannot_silently_drop_payload_or_unknown_constraints(self):
        import math
        path = Path(__file__).resolve().parents[1] / 'tools/curobo_core/rolling_curobo.py'
        cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '__init__')
        class Backend:
            pass
        namespace = dict(CuroboBackend=Backend, math=math)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
        for phase in ('approach', 'contact_approach', 'extract', 'transport', 'conveyor_place', 'return_home'):
            payload = phase in ('extract', 'transport', 'conveyor_place')
            namespace['__init__'](types.SimpleNamespace(), Backend(), {}, phase=phase, payload=payload)
            with self.subTest(phase=phase), self.assertRaisesRegex(ValueError, 'phase contract'):
                namespace['__init__'](types.SimpleNamespace(), Backend(), {}, phase=phase, payload=not payload)
        with self.assertRaisesRegex(ValueError, 'explicit online motion contract'):
            namespace['__init__'](types.SimpleNamespace(), Backend(), {}, phase='extract_unknown', payload=False)
        with self.assertRaisesRegex(ValueError, 'explicit boolean'):
            namespace['__init__'](types.SimpleNamespace(), Backend(), {}, allow_braking_fallback='false', payload=False)

    def test_fixed_nonzero_base_pose_is_preserved_not_zeroed(self):
        record = self.record()
        trajectory, snapshot, boxes, payload = stage_input(record, 'return_home')
        self.assertEqual(tuple(trajectory['joint_names']), ACTIVE_JOINTS)
        self.assertEqual(snapshot.state.base_pose.position, (-.1, 0., 0.))
        self.assertEqual(snapshot.state.base_pose.quaternion_wxyz, (0., 0., 0., 1.))
        self.assertEqual(boxes, {'left': 24, 'right': 20})
        self.assertFalse(payload)

    def test_unknown_legacy_scene_and_moving_base_cannot_silently_use_new_checker(self):
        record = self.record()
        record['result'].pop('rolling_stage_inputs')
        with self.assertRaisesRegex(ValueError, 'regenerate input'):
            stage_input(record, 'return_home')
        record = self.record()
        record['result']['timed_segments'][0]['positions'][1][0] += .1
        with self.assertRaisesRegex(ValueError, 'moving-base'):
            stage_input(record, 'return_home')

    def test_online_adapter_does_not_override_mentor_geometric_search_or_timing(self):
        path = Path(__file__).resolve().parents[1] / 'tools/curobo_core/rolling_backend.py'
        tree = ast.parse(path.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
        methods = {n.name for n in cls.body if isinstance(n, ast.FunctionDef)}
        self.assertEqual(cls.bases[0].id, 'TimedSingleArmPlanner')
        self.assertTrue({'online_search_path', 'optimize_rrt_path', 'snapshot_for_stage'} <= methods)
        self.assertFalse({'full_cycle', 'time_parameterize_result', 'search_path',
                          'collision_checker', 'warmup_resources'} & methods)

    def test_trajopt_approach_scene_includes_task_boxes_until_contact(self):
        path = Path(__file__).resolve().parents[1] / 'tools/curobo_core/rolling_backend.py'
        method = next(n for n in ast.walk(ast.parse(path.read_text()))
                      if isinstance(n, ast.FunctionDef) and n.name == 'optimization_scene')
        # Remove local CUDA Scene import for this control-flow test.
        for node in ast.walk(method):
            if isinstance(node, ast.If):
                node.body = [n for n in node.body if not isinstance(n, ast.ImportFrom)]
        calls = []
        context = {'to_curobo_scene': lambda snapshot: (calls.append(snapshot) or 'all_objects')}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), context)
        backend = types.SimpleNamespace(online_phase='approach', snapshot=object(), scene=lambda boxes: 'payload_scene')
        self.assertEqual(context['optimization_scene'](backend, {'left': 24}), 'all_objects')
        self.assertEqual(calls, [backend.snapshot])
        backend.online_phase = 'transport'
        self.assertEqual(context['optimization_scene'](backend, {'left': 24}), 'payload_scene')

    def test_phase_gate_reuses_contact_gate_and_payload_cuboid_checker(self):
        path = Path(__file__).resolve().parents[1] / 'tools/curobo_core/rolling_backend.py'
        method = next(n for n in ast.walk(ast.parse(path.read_text()))
                      if isinstance(n, ast.FunctionDef) and n.name == 'online_validity')
        checker, target = object(), object()
        calls = []
        backend = types.SimpleNamespace(
            collision_checker=lambda boxes, payload: (calls.append(payload) or checker),
            target_validity=lambda supplied: target if supplied is checker else None)
        # Strip the runtime-only import; inject the exact constructor boundary.
        method.body = [n for n in method.body if not isinstance(n, ast.ImportFrom)]
        context = {'GpuValidity': lambda supplied, **kw: (supplied, kw)}
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), context)
        self.assertIs(context['online_validity'](backend, {}, False, 'approach'), target)
        for phase, payload in (("contact_approach", False), ("extract", False),
                               ("transport", True), ("conveyor_place", True), ("return_home", False)):
            value = context['online_validity'](backend, {}, phase == "extract" or payload, phase)
            self.assertEqual(value, (checker, {'check_ground': True}))
        # The validation robot is empty during extract; loaded endpoint is a separate gate.
        self.assertEqual(calls, [False, False, False, True, True, False])


if __name__ == '__main__':
    unittest.main()
