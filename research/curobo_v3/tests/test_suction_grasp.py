"""Actual attachment, cap footprint and explicit plane normal for eccentric contacts."""
import ast
from pathlib import Path
import sys
import unittest
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from curobo_core.box_pick import BoxPick, layout_model_key
from curobo_core.suction_grasp import contact_cap_spheres


class SuctionGraspTest(unittest.TestCase):
    def test_offset_roll_changes_attachment_and_stays_in_full_proxy_footprint(self):
        for side, sign in [('left', 1), ('right', -1)]:
            centered = BoxPick('40x40x30', (1.1680520510673524, 0., .8), side, carry_joint_home=True)
            for displacement in (.04, .08):
                pick = BoxPick(centered.size, centered.center_m, side, carry_joint_home=True,
                               suction_offset_m=(0., sign*displacement, 0.), suction_roll_deg=90.)
                np.testing.assert_allclose(pick.contact_tool()[:3, 3],
                    np.array(pick.center_m)+[-.15, sign*displacement, 0.], atol=1e-12)
                np.testing.assert_allclose(pick.contact_tool()[:3, 2], [1., 0., 0.], atol=1e-12)
                expected = np.eye(4);expected[:3, 3] = pick.center_m
                np.testing.assert_allclose(pick.contact_tool()@pick.tool_to_box(), expected, atol=1e-12)
                from curobo_core.layout_contract import carry_tool
                np.testing.assert_allclose(pick.carry_box(), carry_tool(side, 0.)@pick.tool_to_box(), atol=1e-12)
                self.assertGreater(np.linalg.norm(pick.tool_to_box()-centered.tool_to_box()), .01)
            for offset, roll in [((0., .1, 0.), 90.), ((0., .04, 0.), 0.), ((.001, 0., 0.), 0.), ((0., float('nan'), 0.), 0.)]:
                with self.assertRaises(ValueError):
                    BoxPick(centered.size, centered.center_m, side, suction_offset_m=offset, suction_roll_deg=roll)
            with self.assertRaises(ValueError):
                BoxPick(centered.size, centered.center_m, side, suction_roll_deg=float('inf'))
        caps = contact_cap_spheres('left')
        self.assertTrue(np.isfinite(caps).all());self.assertGreater(len(caps), 1)

    def test_near_y_side_face_keeps_requested_minus_x_extraction(self):
        pick = BoxPick('46.5x25.5x58.2', (1.1680520510673524, .5, 1.4), 'right',
                       carry_joint_home=True, side_face='y-', suction_roll_deg=90.)
        tool = pick.contact_tool()
        np.testing.assert_allclose(tool[:3, 3], [1.1680520510673524, .5-.465/2, 1.4], atol=1e-12)
        np.testing.assert_allclose(tool[:3, 2], [0., 1., 0.], atol=1e-12)
        end = tool.copy();end[0, 3] -= .255
        expected = np.eye(4);expected[:3, 3] = np.asarray(pick.center_m)-[.255, 0., 0.]
        np.testing.assert_allclose(end@pick.tool_to_box(), expected, atol=1e-12)
        for fields in [dict(side_face='bad'), dict(side_face='y-', suction_roll_deg=0.),
                       dict(side_face='y-', suction_roll_deg=90., suction_offset_m=(0., .001, 0.))]:
            with self.assertRaises(ValueError):
                BoxPick(pick.size, pick.center_m, 'right', **fields)

    def test_stacked_auto_recovery_keeps_centers_threshold_and_explicit_grasps(self):
        from types import SimpleNamespace as NS
        import tempfile, json
        from dataclasses import asdict, replace
        file = ROOT/'tools/v3_box_layout_planner.py'
        cls = next(n for n in ast.parse(file.read_text()).body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'plan_layout')
        from curobo_core.layout_contract import validate_pair
        env = dict(np=np, Path=Path, json=json, asdict=asdict, replace=replace, validate_pair=validate_pair)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(file), 'exec'), env)
        picks = {s:BoxPick(size, (1.1680520510673524, 0., z), s, carry_joint_home=True)
                 for s, size, z in [('left', '40x40x30', .8), ('right', '46.5x25.5x58.2', 1.4)]}
        for adapt, count in [(True, 2), (False, 1)]:
            calls = []
            def plan(actual, *args):
                calls.append(actual)
                return dict(success=len(calls) == 2, failure=None if len(calls) == 2 else 'finite original failure')
            with tempfile.TemporaryDirectory() as directory:
                result = env['plan_layout'](NS(_plan_layout=plan), picks, directory, adapt_grasp=adapt)
                self.assertEqual(len(calls), count)
                self.assertEqual(len(result['grasp_trials']), count)
                for side in picks:
                    self.assertEqual(calls[-1][side].center_m, picks[side].center_m)
                    self.assertEqual(calls[-1][side].side_min_height_m, .8)
                if adapt:
                    self.assertEqual(calls[-1]['left'].suction_offset_m, (0., 0., -.08))
                    self.assertEqual(calls[-1]['right'].suction_offset_m, (0., 0., .08))
                    self.assertTrue(all(Path(t['result_file']).exists() for t in result['grasp_trials']))

    def test_same_side_face_recovery_is_automatic_and_preserves_source_poses(self):
        from types import SimpleNamespace as NS
        from dataclasses import asdict, replace
        from curobo_core.layout_contract import validate_pair
        import tempfile, json
        file = ROOT/'tools/v3_box_layout_planner.py'
        cls = next(n for n in ast.parse(file.read_text()).body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'plan_layout')
        env = dict(np=np, Path=Path, json=json, asdict=asdict, replace=replace, validate_pair=validate_pair)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(file), 'exec'), env)
        picks = {s:BoxPick(size, (1.1680520510673524, .5, z), s, carry_joint_home=True, side_min_height_m=.79)
                 for s, size, z in [('left', '40x40x30', .8), ('right', '46.5x25.5x58.2', 1.4)]}
        seen = []
        def plan(actual, *args):
            seen.append(actual)
            return dict(success=actual['right'].side_face == 'y-', failure=None)
        with tempfile.TemporaryDirectory() as directory:
            result = env['plan_layout'](NS(_plan_layout=plan), picks, directory)
            self.assertTrue(result['success'])
            self.assertEqual(seen[-1]['right'].side_face, 'y-')
            self.assertEqual(seen[-1]['right'].suction_roll_deg, 90.)
            for actual in seen:
                for side in picks:
                    self.assertEqual(actual[side].center_m, picks[side].center_m)
                    self.assertEqual(actual[side].side_min_height_m, .79)
            self.assertEqual(result['grasp_trials'][-1]['suction']['right']['face'], 'y-')

    def test_carry_preview_applies_selected_face_orientation_and_rejects_bad_metadata(self):
        from types import SimpleNamespace as NS
        from scipy.spatial.transform import Rotation
        from v3_box_layout import apply_carry_preview
        pick = BoxPick('46.5x25.5x58.2', (1.1680520510673524, .5, 1.4), 'right',
                       side_face='y-', suction_roll_deg=90., carry_joint_home=True, carry_updown_m=-.5)
        matrix = pick.carry_box()
        ghost = NS(position=np.zeros(3), wxyz=np.array([1., 0., 0., 0.]))
        event = dict(targets={'right': matrix[:3, 3].tolist()}, box_matrices={'right': matrix.tolist()})
        apply_carry_preview({'right':ghost}, event)
        np.testing.assert_allclose(ghost.position, matrix[:3, 3], atol=1e-12)
        np.testing.assert_allclose(Rotation.from_quat(np.roll(ghost.wxyz, -1)).as_matrix(), matrix[:3, :3], atol=1e-12)
        before = ghost.position.copy()
        bad = matrix.copy();bad[0, 0] += .1
        with self.assertRaises(ValueError):
            apply_carry_preview({'right':ghost}, dict(event, box_matrices={'right':bad.tolist()}))
        np.testing.assert_array_equal(ghost.position, before)

    def test_actual_attachment_invalidates_same_scene_collision_resources(self):
        from types import SimpleNamespace as NS
        from curobo_core.scene import AttachedObject, Pose
        file = ROOT/'tools/curobo_core/backend.py'
        cls = next(n for n in ast.parse(file.read_text()).body if isinstance(n, ast.ClassDef))
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'prepare_task')
        env = dict(resource_key=lambda *args:'same-scene-and-boxes', gc=NS(collect=lambda:None))
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(file), 'exec'), env)
        current = [AttachedObject('box', (.3, .4, .4), 'left_tool0', Pose((.15, 0., 0.)))]
        owner = NS(snapshot=None, box_fit=None, _cached_task=None, _cached_solver=object(),
                   _cached_checkers={'loaded': object()}, task_attachments=lambda boxes:tuple(current))
        prepare = env['prepare_task']
        prepare(owner, {'left': 0})
        owner._cached_checkers['loaded'] = 'retained';owner._cached_solver = 'retained'
        prepare(owner, {'left': 0})
        self.assertEqual(owner._cached_checkers, {'loaded': 'retained'})
        current[0] = AttachedObject('box', (.3, .4, .4), 'left_tool0', Pose((.15, .04, 0.)))
        prepare(owner, {'left': 0})
        self.assertEqual(owner._cached_checkers, {});self.assertIsNone(owner._cached_solver)

    def test_eccentric_contact_uses_actual_normal_and_does_not_exempt_neighbor(self):
        file = ROOT/'tools/v3_contact_validity.py'
        method = next(n for n in ast.parse(file.read_text()).body if isinstance(n, ast.FunctionDef) and n.name == 'target_in_front_of_contact_plane')
        namespace = dict(np=np, torch=torch)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(file), 'exec'), namespace)
        contact = [.85, .08, 1.]
        target = dict(center=[1., 0., 1.], contact=contact, inward_normal=[1., 0., 0.], dimensions=[.3, .4, .4], tool_cap_reach_m=.05)
        normal = torch.tensor([[1., 0., 0.]])
        position = torch.tensor([contact])
        check = namespace['target_in_front_of_contact_plane']
        self.assertTrue(bool(check(position, normal, target, .001, True).all()))
        self.assertFalse(bool(check(position, -normal, target, .001, True).any()))
        wrong = dict(target, center=[1., -.4, 1.], contact=[.85, -.4, 1.])
        self.assertFalse(bool(check(position, normal, wrong, .001, True).any()))
        legacy = dict(target);legacy.pop('inward_normal')
        self.assertFalse(bool(check(position, normal, legacy, .001, True).any()))


if __name__ == '__main__':unittest.main()
