"""Explicit GPU regression; ordinary CPU discovery must not import the CUDA planner."""
import os
from pathlib import Path
import sys
import unittest


@unittest.skipUnless(os.environ.get('RUN_GPU_TESTS') == '1', 'run this regression through gpuq')
class CandidateFeedbackTest(unittest.TestCase):
    def test_rejected_candidate_is_not_returned_and_next_candidate_can_pass(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
        import numpy as np
        from curobo_core.box_pick import BoxPick
        from v3_two_box_pick import TwoBoxPickPlanner
        seen=[]

        def validate(record):
            self.assertFalse(record['success'], 'a pending external validation must not look accepted')
            self.assertTrue(record['geometry_success'])
            seen.append(record['attempts'][-1]['contact_index'])
            return dict(success=len(seen)==2, regression_validator=True)

        result=TwoBoxPickPlanner().plan_pick(BoxPick('40x40x30',(1.18,.45,1.02),'left'),
            max_contacts=3,candidate_validator=validate)
        self.assertTrue(result['success'])
        self.assertIsNone(result['failure'])
        self.assertEqual(len(seen),2)
        self.assertGreater(seen[1],seen[0])
        outcomes=[a['candidate_validation']['success'] for a in result['attempts'] if 'candidate_validation' in a]
        self.assertEqual(outcomes,[False,True])
        frames=np.asarray(result['frames']);phases=np.asarray(result['phases'])
        seams=np.flatnonzero(phases[1:]!=phases[:-1])+1
        self.assertLessEqual(float(np.abs(frames[seams]-frames[seams-1]).max()),1e-5)
        self.assertGreaterEqual(result['geometry_wall_seconds'],0.)
    def test_native_margin_checks_distinguish_frozen_margin_failure_from_collision(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
        import torch
        from curobo_core.box_pick import BoxPick
        from v3_two_box_pick import TwoBoxPickPlanner
        planner=TwoBoxPickPlanner();planner.configure_pick(BoxPick('40x40x30',(1.18,.45,1.02),'left'))
        checker=planner.checker(planner.selected_task)
        # Frozen R12 timed interior sample 802: no hard collision, but the native 5mm base term is positive.
        witness=[-0.290716290473938, 1.009682297706604, -1.3129098415374756, 0.311583936214447, -1.5018374919891357, -0.045047421008348465, -0.6673352122306824, 1.5626144409179688, -1.127656102180481, 1.3074531555175781, -0.35783493518829346, 1.5657542943954468, -0.006240665912628174, 0.6969549655914307, -1.5671453475952148]
        values=torch.tensor([planner.home_values.tolist(),witness],device='cuda',dtype=torch.float32)
        hard=planner.target_validity(checker)
        hard.scene_penetration_tolerance_m=.001
        self.assertTrue(bool(hard.mask(values).all()))
        self.assertEqual(planner.native_margin_mask(checker,values).cpu().tolist(),[True,False])
        planner.native_project_margins=True
        native=planner.target_validity(checker);native.scene_penetration_tolerance_m=.001
        self.assertEqual(native.mask(values).cpu().tolist(),[True,False])
    def test_native_invalid_extraction_endpoint_is_rejected_before_rrt(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
        from curobo_core.box_pick import BoxPick
        from v3_two_box_pick import TwoBoxPickPlanner
        planner = TwoBoxPickPlanner()
        planner.native_project_margins = True
        result = planner.plan_pick(BoxPick('40x40x30', (1., .45, .201), 'left'), max_contacts=1)
        self.assertFalse(result['success'])
        self.assertFalse(result['attempts'][0]['native_transport_start_valid'])
        self.assertEqual(result['attempts'][0]['failure'], 'extracted state fails native transport-start base/ground margins')
        self.assertNotIn('transport_search', result['attempts'][0])

    def test_incompatible_requested_carry_target_never_reaches_native_validation(self):
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
        from curobo_core.box_pick import BoxPick
        from v3_two_box_pick import TwoBoxPickPlanner
        def unexpected(record):
            self.fail('invalid return target must be reported before native work')
        task=BoxPick('46.5x25.5x58.2',(1.,.45,.491),'left',carry_offset_m=(.05,.0325,0.))
        result=TwoBoxPickPlanner().plan_pick(task,candidate_validator=unexpected)
        self.assertFalse(result['success'])
        self.assertEqual(result['failure_kind'],'carry_target_contract_conflict')
        self.assertEqual(result['attempts'],[])
        self.assertEqual(result['frames'],[])



if __name__=='__main__':unittest.main()
