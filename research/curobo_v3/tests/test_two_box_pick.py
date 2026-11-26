from pathlib import Path
import json
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
from curobo_core.box_pick import BOX_DIMENSIONS, BoxPick, payload_fit, timed_path_checks, diverse_contact_order
from v3_contact_roll_policy import aligned_grasp_transform, aligned_tool_pose


class TwoBoxPickTest(unittest.TestCase):
    def test_dimensions_contacts_attachments_and_45_degree_carry(self):
        rng = np.random.default_rng(82)
        for size, dims in BOX_DIMENSIONS.items():
            for side in ('left', 'right'):
                for z in (.4, .8 - 1e-8, .8, 1.4):
                    for _ in range(12):
                        center = (*rng.uniform(-2, 2, 2), z)
                        task = BoxPick(size, center, side)
                        self.assertEqual(task.top, z < .8)
                        self.assertEqual(task.extraction_distance_m, dims[0])
                        contact = task.contact_tool()
                        box = contact @ task.tool_to_box()
                        np.testing.assert_allclose(box[:3, 3], center, atol=1e-12)
                        np.testing.assert_allclose(box[:3, :3], np.eye(3), atol=1e-12)
                        expected = np.array(center)
                        expected[2 if task.top else 0] += dims[2]/2 if task.top else -dims[0]/2
                        np.testing.assert_allclose(contact[:3, 3], expected, atol=1e-12)
                        inward = np.asarray(center) - contact[:3, 3]
                        np.testing.assert_allclose(contact[:3, 2], inward/np.linalg.norm(inward), atol=1e-12)
                        carry = task.carry_tool() @ task.tool_to_box()
                        np.testing.assert_allclose(carry, task.carry_box(), atol=1e-12)
                        np.testing.assert_allclose(Rotation.from_matrix(carry[:3, :3]).as_euler('xyz', degrees=True), [0, -45, 0], atol=1e-10)

    def test_default_attachment_unchanged(self):
        for top, height in ((True, .4), (False, 1.0)):
            for side in ('left', 'right'):
                task = BoxPick('40x40x30', (1., 0., height), side)
                np.testing.assert_allclose(task.tool_to_box(), aligned_grasp_transform(side, top))

    def test_default_carry_matches_pr82_not_just_absolute_pitch(self):
        root = Path(__file__).resolve().parents[1]
        policy = json.loads((root/'config/placement_task_policy.json').read_text())['side']
        for side in ('left', 'right'):
            pose = aligned_tool_pose(dict(position=[policy['position']['x'], policy['position']['y_'+side], policy['position']['z']], quaternion=policy['quaternion_wxyz']))
            expected = np.eye(4)
            expected[:3, 3] = pose['position']
            expected[:3, :3] = Rotation.from_quat(np.roll(pose['quaternion'], -1)).as_matrix()
            np.testing.assert_allclose(BoxPick('40x40x30', (1., 0., 1.), side).carry_tool(), expected, atol=1e-12)

    def test_scaled_spheres_stay_inside_real_box(self):
        root = Path(__file__).resolve().parents[1]
        source = json.loads((root/'generated/v3_suction_v322_6bb184b/frozen_collision_model/box_voxel_edge_corner_final.json').read_text())
        original = json.dumps(source, sort_keys=True)
        for dims in BOX_DIMENSIONS.values():
            fit = payload_fit(source, dims)
            self.assertEqual(len(fit['radii']), 40)
            self.assertTrue(np.all(np.asarray(fit['radii']) > 0))
            self.assertTrue(np.all(np.abs(fit['centers']) + np.asarray(fit['radii'])[:, None] <= np.asarray(dims)/2 + 1e-6))
        self.assertEqual(json.dumps(source, sort_keys=True), original)

    def test_original_tall_carry_intersects_column_and_offsets_keep_legacy_anchor(self):
        # A strict interior witness proves the old target is geometrically impossible.
        task = BoxPick('46.5x25.5x58.2', (1.18, .45, 1.02), 'left', carry_offset_m=(0., 0., 0.))
        point = np.array([.3, .1975, 1.35])
        box = task.carry_box()
        self.assertTrue(np.all(np.abs((point-box[:3, 3]) @ box[:3, :3]) < np.asarray(task.dimensions_m)/2))
        root = Path(__file__).resolve().parents[1]
        column = json.loads((root/'generated/endpoint_optimizer_compare/base_cuboids_20261009/frozen_cuboids.json').read_text())['cuboids'][-1]
        self.assertGreater(float(np.min(np.asarray(column['dimensions'])/2-np.abs(point-column['center']))), .001)
        self.assertGreater(float(np.min(np.asarray(task.dimensions_m)/2-np.abs((point-box[:3, 3]) @ box[:3, :3]))), .001)
        shifted = BoxPick(task.size, task.center_m, task.side, carry_offset_m=(.05, .0325, 0.))
        np.testing.assert_allclose(shifted.carry_box()[:3, 3]-box[:3, 3], [.05, .0325, 0.])
        np.testing.assert_allclose(shifted.carry_box()[:3, :3], box[:3, :3])
        np.testing.assert_allclose(shifted.contact_tool(), task.contact_tool())
        with self.assertRaises(ValueError):
            BoxPick(task.size, task.center_m, task.side, carry_offset_m=(0., float('nan'), 0.))

    def test_boundary_fixture_exercises_switch_without_box_wall_penetration(self):
        root = Path(__file__).resolve().parents[1]
        cases = json.loads((root/'config/two_box_pick/cube_boundary_cases.json').read_text())
        self.assertEqual(len(cases), 16)
        self.assertEqual({row['side'] for row in cases}, {'left', 'right'})
        self.assertEqual({row['center_m'][2] for row in cases[:4]}, {.8-1e-5, .8})
        probes = json.loads((root/'config/two_box_pick/approved_center_probes_cases.json').read_text())
        self.assertEqual(len(probes), 32)
        self.assertEqual({row['size'] for row in probes}, set(BOX_DIMENSIONS))
        for row in cases + probes:
            task = BoxPick(**row)
            center, half = np.asarray(task.center_m), np.asarray(task.dimensions_m)/2
            self.assertGreaterEqual(center[2]-half[2], 0)
            self.assertLessEqual(center[2]+half[2], 2.4)
            self.assertLessEqual(abs(center[1])+half[1], 1.2)
            self.assertGreater(center[0]-half[0], .5)
            self.assertLess(center[0]+half[0], 1.7080530510673522)
        # These bounds prove only geometric containment, not IK or path feasibility.

    def test_precontact_is_outside_real_proxy_caps_not_hardcoded_30mm(self):
        import yaml
        root = Path(__file__).resolve().parents[1]
        kin = yaml.safe_load((root/'generated/zero_model_candidate_20261018/fixed.yaml').read_text())['kinematics']
        for side in ('left', 'right'):
            task = BoxPick('40x40x30', (1.18, .45, 1.84), side)
            cap_reach = max(item['center'][2]+item['radius']-.135 for item in kin['collision_spheres'][side+'_link7'] if item['radius']>0)
            self.assertGreater(cap_reach, .03)  # The PR82 frozen proxies protrude beyond the old stand-off.
            self.assertGreaterEqual(task.precontact_distance(cap_reach)-cap_reach, .005-1e-12)
            self.assertGreaterEqual(task.precontact_distance(cap_reach), .03)
        with self.assertRaises(ValueError):
            BoxPick('40x40x30', (1.18, .45, 1.84), 'left', precontact_margin_m=0.)

    def test_native_timing_checks_actual_arrays_and_rest_seams(self):
        import copy
        root = Path(__file__).resolve().parents[1]
        policy = json.loads((root/'config/fast_three_layers.json').read_text())
        arrays = {key: np.zeros((3, 14 if key == 'torque' else 15)) for key in ('position', 'velocity', 'acceleration', 'jerk', 'torque')}
        arrays['times'] = np.array([0., .5, 1.])
        self.assertTrue(timed_path_checks(arrays, np.zeros(15), np.zeros(15), policy)['valid'])
        for key, index, value in (('position', (2, 1), .01), ('velocity', (0, 1), .01), ('acceleration', (2, 1), .01),
                                  ('velocity', (1, 1), 2.), ('jerk', (1, 1), 501.), ('torque', (1, 0), 300.)):
            bad = copy.deepcopy(arrays); bad[key][index] = value
            self.assertFalse(timed_path_checks(bad, np.zeros(15), np.zeros(15), policy)['valid'], key)
        for clock in ([0., 0., 1.], [0., 1., float('nan')]):
            bad = dict(arrays, times=np.asarray(clock))
            with self.assertRaises(ValueError):timed_path_checks(bad, np.zeros(15), np.zeros(15), policy)

    def test_payload_inertia_uses_the_two_real_sizes(self):
        from curobo_core.torque import cuboid_inertia
        old = cuboid_inertia(10., BOX_DIMENSIONS['40x40x30'])
        tall = cuboid_inertia(10., BOX_DIMENSIONS['46.5x25.5x58.2'])
        np.testing.assert_allclose(old, np.diag([.2666666666666667, .2083333333333333, .2083333333333333]), atol=1e-12)
        np.testing.assert_allclose(tall, np.diag([.4624575, .3364575, .234375]), atol=1e-12)
        self.assertFalse(np.allclose(old, tall))

    def test_contact_order_preserves_all_states_and_exposes_opposite_elbow(self):
        rows=np.zeros((8,15));rows[:,4]=[-1,-1,-1,-1,-1,1,1,1]
        order=diverse_contact_order(rows,'left')
        np.testing.assert_array_equal(order,[0,5,1,6,2,7,3,4])
        np.testing.assert_array_equal(np.sort(order),np.arange(len(rows)))
        rows[:,11]=-rows[:,4]
        np.testing.assert_array_equal(diverse_contact_order(rows,'right'),order)
        rows[:,4]=-1
        np.testing.assert_array_equal(diverse_contact_order(rows,'left'),np.arange(len(rows)))
        np.testing.assert_array_equal(diverse_contact_order(np.empty((0,15)),'left'),[])
        with self.assertRaises(ValueError):diverse_contact_order(np.zeros((1,14)),'left')

    def test_requested_carry_envelope_exposes_invalid_tall_experiment(self):
        root=Path(__file__).resolve().parents[1]
        source=json.loads((root/'generated/v3_suction_v322_6bb184b/frozen_collision_model/box_voxel_edge_corner_final.json').read_text())
        for side,sign in (('left',1),('right',-1)):
            cube=BoxPick('40x40x30',(1.,sign*.45,1.02),side)
            self.assertLess(cube.carry_payload_envelope(payload_fit(source,cube.dimensions_m))['legacy_payload_sphere_radius_m'],1.05)
            tall=BoxPick('46.5x25.5x58.2',(1.,sign*.45,.491),side,carry_offset_m=(.05,sign*.0325,0.))
            env=tall.carry_payload_envelope(payload_fit(source,tall.dimensions_m))
            self.assertGreater(env['legacy_payload_sphere_radius_m'],1.11)
            self.assertGreater(env['legacy_radius_lower_bound_with_pose_tolerance_m'],1.05)
            self.assertGreater(env['box_corner_radius_m'],env['legacy_payload_sphere_radius_m'])

    def test_approved_size_mode_defaults_and_resolved_record_replay(self):
        from dataclasses import asdict
        root = Path(__file__).resolve().parents[1]
        source = json.loads((root/'generated/v3_suction_v322_6bb184b/frozen_collision_model/box_voxel_edge_corner_final.json').read_text())
        for size in BOX_DIMENSIONS:
            for side, sign in (('left', 1), ('right', -1)):
                for z in (.491, .8-1e-8, .8, 1.02):
                    task = BoxPick(size, (1., sign*.45, z), side)
                    xy = (.55, .425) if size == '40x40x30' else ((.51, .4385) if z < .8 else (.34 if side == 'left' else .38, .56))
                    np.testing.assert_allclose(task.carry_box()[:3, 3], [xy[0], sign*xy[1], 1.2], atol=1e-12)
                    self.assertEqual(task, BoxPick(**json.loads(json.dumps(asdict(task)))))
                    np.testing.assert_array_equal(task.carry_box(), BoxPick(**asdict(task)).carry_box())
                    env = task.carry_payload_envelope(payload_fit(source, task.dimensions_m))
                    self.assertLess(max(env['legacy_payload_sphere_radius_m'], env['box_corner_radius_m']), 1.05)
                    original = BoxPick(size, task.center_m, side, carry_offset_m=(0., 0., 0.))
                    np.testing.assert_allclose(original.carry_box()[:3, 3], [.55, sign*.425, 1.2])

    def test_replay_uses_native_times_and_arrays_and_rejects_failed_or_broken_seams(self):
        from experiments.two_box_pick_20261119.replay_native import native_samples
        from tempfile import TemporaryDirectory
        import copy
        with TemporaryDirectory() as folder:
            record = dict(success=True, timing=dict(success=True, stages=[]))
            for i, phase in enumerate(('approach', 'contact', 'extract', 'transport')):
                path = Path(folder)/f'{phase}.npz'
                q = np.full((3, 15), float(i)); q[-1] += 1
                np.savez(path, times=[0., .3, 1.], position=q)
                record['timing']['stages'].append(dict(phase=phase, success=True, trajectory_file=str(path)))
            times, rows, phases = native_samples(record)
            np.testing.assert_allclose(times, [0, .3, 1, 1.3, 2, 2.3, 3, 3.3, 4])
            np.testing.assert_array_equal(rows[-1], np.full(15, 4.))
            self.assertEqual(phases[-1], 'transport')
            bad = copy.deepcopy(record); bad['success'] = False
            with self.assertRaises(ValueError):native_samples(bad)
            np.savez(path, times=[0., .3, 1.], position=np.zeros((3, 15)))
            with self.assertRaises(ValueError):native_samples(record)

    def test_old_omitted_offset_inputs_are_audited_as_original_anchor(self):
        from dataclasses import asdict
        from experiments.two_box_pick_20261119.verify_cases import recorded_task
        original = dict(size='46.5x25.5x58.2', center_m=[1., .45, .491], side='left')
        legacy = BoxPick(**original, carry_offset_m=(0., 0., 0.))
        self.assertEqual(recorded_task(dict(task=asdict(legacy)), original), legacy)
        current = BoxPick(**original)
        self.assertEqual(recorded_task(dict(task=asdict(current), carry_target_contract='2026-11-19-v2'), original), current)
        with self.assertRaises(AssertionError):
            recorded_task(dict(task=asdict(current)), original)
        with self.assertRaises(AssertionError):
            recorded_task(dict(task=asdict(legacy)), dict(original, center_m=[1.1, .45, .491]))

    def test_reject_invalid_inputs(self):
        for center in ((0, 0, .1), (0, float('nan'), 1), (0, 0), (0, 0, float('inf'))):
            with self.assertRaises(ValueError):
                BoxPick('40x40x30', center, 'left')
        for threshold in (-.1, float('nan')):
            with self.assertRaises(ValueError):
                BoxPick('40x40x30', (1, 0, 1), 'left', threshold)


if __name__ == '__main__':
    unittest.main()
