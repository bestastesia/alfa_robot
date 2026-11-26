"""CPU regression: shared center plane, actual URDF FK and exact joint carry."""
from dataclasses import replace
from pathlib import Path
import sys
import unittest

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from curobo_core.box_pick import BoxPick
from curobo_core.layout_contract import base_front_x, carry_configuration, carry_reference, carry_tool
from v3_box_layout import parse_layout


class JointContractTest(unittest.TestCase):
    def document(self):
        return dict(frame_id='world', front_distance_m=.68, boxes=[
            dict(size=size, side=side, center_m=[base_front_x()+.68, y, z])
            for side, size, y, z in [('left', '40x40x30', .5, .5),
                                     ('right', '46.5x25.5x58.2', -.5, 1.1)]])

    def test_plane_height_and_distance_are_actual_input_constraints(self):
        parse_layout(self.document())
        for field, value in [('x', .01), ('height', .001), ('distance', .01)]:
            doc = self.document()
            if field == 'x':doc['boxes'][0]['center_m'][0] += value
            elif field == 'height':doc['boxes'][1]['center_m'][2] += value
            else:
                for row in doc['boxes']:row['center_m'][0] += value
            with self.assertRaises(ValueError):parse_layout(doc)
        for distance in np.arange(.60, .901, .02):
            doc = self.document(); doc['front_distance_m'] = float(distance)
            for row in doc['boxes']:row['center_m'][0] = base_front_x()+distance
            parse_layout(doc)

    def test_fk_uses_specified_joints_and_free_lift_for_both_modes_and_sizes(self):
        import yourdfpy
        robot = yourdfpy.URDF.load(Path(__file__).resolve().parents[1]/'generated/fast_runtime_assets/fixed.urdf', load_meshes=False)
        self.assertEqual(carry_reference()[2], (-1., 0.))
        for updown in (-1., -.41, 0.):
            cfg = carry_configuration(updown)
            robot.update_cfg(cfg)
            np.testing.assert_allclose([cfg[f'left_joint{i}'] for i in range(1, 8)],
                                       np.deg2rad([40, -46, 0, -100, -30, -75, 90]), atol=1e-12)
            for side in ('left', 'right'):
                for size in ('40x40x30', '46.5x25.5x58.2'):
                    for z in (.65, .8):
                        pick = BoxPick(size, (base_front_x()+.68, .5 if side == 'left' else -.5, z), side,
                                       carry_joint_home=True, carry_updown_m=updown)
                        expected = robot.get_transform(side+'_tool0', robot.base_link)
                        np.testing.assert_allclose(carry_tool(side, updown), expected, atol=1e-12)
                        np.testing.assert_allclose(pick.carry_tool(), expected, atol=1e-12)
                        np.testing.assert_allclose(pick.carry_box(), expected@pick.tool_to_box(), atol=1e-12)
                        self.assertEqual(pick.extraction_distance_m, .30 if size == '40x40x30' else .255)
                        self.assertEqual(pick.top, z < .8)
        doc = self.document();doc['boxes'][0]['carry_offset_m'] = [.1, 0, 0]
        with self.assertRaises(ValueError):parse_layout(doc)


    def test_current_layout_threshold_candidate_is_global_and_legacy_replay_keeps_old_default(self):
        for height, top in [(.789, True), (.79, False), (.791, False)]:
            document = self.document()
            document['boxes'][0]['center_m'][2] = height
            picks = parse_layout(document)
            self.assertEqual(picks['left'].top, top)
            self.assertEqual({p.side_min_height_m for p in picks.values()}, {.79})
        self.assertTrue(BoxPick('40x40x30', (base_front_x()+.66, .5, .79), 'left').top)

    def test_input_cannot_override_joint_contract_or_global_threshold(self):
        for field, value in [('carry_joint_home', False), ('carry_joint_home', 'yes'), ('carry_updown_m', -.3),
                             ('side_min_height_m', .9)]:
            document = self.document();document['boxes'][0][field] = value
            with self.assertRaises(ValueError):parse_layout(document)
        for value in (float('nan'), float('inf'), .7):
            document = self.document();document['front_distance_m'] = value
            with self.assertRaises(ValueError):parse_layout(document)
        with self.assertRaises(ValueError):
            BoxPick('40x40x30', (base_front_x()+.66, .5, 1.), 'left',
                    carry_joint_home=True, carry_offset_m=(.1, 0, 0))

    def test_near_y_face_removes_original_same_side_front_face_witness(self):
        import yourdfpy
        from curobo_core.contact_reach import urdf_reach_model, contact_reach_witnesses
        from v3_zero_model_candidate import OFFSETS_DEG, JOINT_SIGNS
        root = Path(__file__).resolve().parents[1]
        robot = yourdfpy.URDF.load(root/'generated/fast_runtime_assets/fixed.urdf', load_meshes=False)
        model = urdf_reach_model(robot, np.deg2rad([OFFSETS_DEG['left'], OFFSETS_DEG['right']]), JOINT_SIGNS)
        picks = {
            'left': BoxPick('40x40x30', (base_front_x()+.66, .5, .8), 'left', side_min_height_m=.79),
            'right': BoxPick('46.5x25.5x58.2', (base_front_x()+.66, .5, 1.4), 'right',
                             side_min_height_m=.79, side_face='y-', suction_roll_deg=90.)}
        self.assertEqual(contact_reach_witnesses(picks, model, (-1., 0.)), [])

    def test_input_rules_do_not_imply_front_or_top_contact_reachability(self):
        import yourdfpy
        from curobo_core.contact_reach import urdf_reach_model, contact_reach_witnesses
        from v3_zero_model_candidate import OFFSETS_DEG, JOINT_SIGNS
        root = Path(__file__).resolve().parents[1]
        robot = yourdfpy.URDF.load(root/'generated/fast_runtime_assets/fixed.urdf', load_meshes=False)
        model = urdf_reach_model(robot, np.deg2rad([OFFSETS_DEG['left'], OFFSETS_DEG['right']]), JOINT_SIGNS)
        for cm in range(60, 91, 2):
            for threshold in (.8, 1., 1.5):
                for swap in (False, True):
                    boxes = [('40x40x30', .8), ('46.5x25.5x58.2', 1.4)]
                    if swap:boxes.reverse()
                    document = dict(frame_id='world', front_distance_m=cm/100, boxes=[
                        dict(side=side, size=size, center_m=[base_front_x()+cm/100, .5, z],
                             side_min_height_m=threshold)
                        for side, (size, z) in zip(('left', 'right'), boxes)])
                    picks = parse_layout(document)
                    witnesses = contact_reach_witnesses(picks, model, (-1., 0.))
                    self.assertTrue(witnesses, (cm, threshold, swap))
                    self.assertIn('right', {w['side'] for w in witnesses})


if __name__ == '__main__':unittest.main()
