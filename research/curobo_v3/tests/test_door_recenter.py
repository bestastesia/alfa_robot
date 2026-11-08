"""CPU contract checks; the saved real-scene runs are a separate acceptance gate."""
from dataclasses import replace
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import torch
from scipy.spatial.transform import Rotation, Slerp

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'tools'))
from curobo_core.door_recenter import (DOOR_COLUMN_ORDER,DoorPlanner,DoorTask,LEFT,RIGHT,
                                         SupportCompensatedValidity,door_pair,door_pick_base_pose,
                                         snapshot_in_base,snapshot_in_world)
from curobo_core.door_sequential import check_frozen
from curobo_core.scene import AttachedObject, Pose, RobotState, SceneObject, SceneSnapshot, SceneStore
from curobo_core.adapter import pose_matrix
from curobo_core.contracts import ACTIVE_JOINTS
from v3_cartesian_transport import CartesianTransportSpace
from v3_conveyor_cycle_planner import ConveyorCyclePlanner
from v3_door_recenter_viewer import load_round_results, overall_timing_markdown, stage_timing_markdown
from v3_door_recenter import certified_cache_result


class RecenterTests(unittest.TestCase):
    def test_five_column_order_matches_the_original_mentor_order(self):
        self.assertEqual(DOOR_COLUMN_ORDER,(4,0,3,1,2))
        self.assertEqual([door_pair(column) for column in DOOR_COLUMN_ORDER],
                         [(19,24),(15,20),(18,23),(16,21),(17,22)])
        with self.assertRaises(ValueError):door_pair(5)

    def test_column_pick_pose_faces_each_column_without_moving_the_world(self):
        yaws=[math.degrees(2*math.atan2(door_pick_base_pose(column).quaternion_wxyz[3],
                                       door_pick_base_pose(column).quaternion_wxyz[0])) for column in range(5)]
        np.testing.assert_allclose(yaws,(-30.,0.,0.,0.,30.),atol=1e-12)
        np.testing.assert_allclose([door_pick_base_pose(column).position[1] for column in range(5)],
                                   (-.2,-.4,0.,.4,.2),atol=1e-12)
        state=RobotState.from_mapping(dict.fromkeys(ACTIVE_JOINTS,0.),123,door_pick_base_pose(4))
        snapshot=SceneSnapshot('test',state,(SceneObject('box',(.3,.4,.4),Pose((1.,.8,1.))),))
        local=snapshot_in_base(snapshot)
        self.assertEqual(local.state.base_pose,Pose())
        restored=snapshot_in_world(local,state.base_pose)
        self.assertEqual(restored.state.base_pose,state.base_pose)
        np.testing.assert_allclose(restored.object('box').pose.position,(1.,.8,1.),atol=1e-12)
        np.testing.assert_allclose(restored.object('box').pose.quaternion_wxyz,(1.,0.,0.,0.),atol=1e-12)

    def test_latest_placement_atlas_is_the_recenter_contract(self):
        policy=json.loads((ROOT/'config/placement_task_policy.json').read_text())
        self.assertEqual(policy['side']['atlas_path'],'generated/selected_placement_20261013/all_solutions.npz')
        atlas=np.load(ROOT/policy['side']['atlas_path'])
        self.assertEqual(atlas['joints'].shape,(2816,15))
        self.assertLessEqual(float(atlas['radius'].max()),policy['side']['placement_radius_m'])

    def test_grasp_roll_order_is_configurable_and_validated(self):
        task=DoorTask.__new__(DoorTask)
        self.assertEqual(tuple(round(math.degrees(x)) for x in task.grasp_roll_candidates('left')),
                         (0,30,-30,45,-45))
        task.left_grasp_rolls_deg=(0.,45.)
        self.assertEqual(tuple(round(math.degrees(x)) for x in task.grasp_roll_candidates('left')),(0,45))
        self.assertEqual(tuple(round(math.degrees(x)) for x in task.grasp_roll_candidates('right')),(0,30,-30,45,-45))
        task.right_grasp_rolls_deg=(0.,-30.)
        self.assertEqual(tuple(round(math.degrees(x)) for x in task.grasp_roll_candidates('right')),(0,-30))
        task.left_grasp_rolls_deg=(0.,0.)
        with self.assertRaises(ValueError):task.grasp_roll_candidates('left')
        task.right_grasp_rolls_deg=(45.,45.)
        with self.assertRaises(ValueError):task.grasp_roll_candidates('right')

    def test_certified_plan_cache_fails_closed(self):
        result=dict(success=True,time_parameterized=True,motion_duration_s=10.,
            final_snapshot={'attachments':[]},navigation_path_validated=False,
            dual_recenter_verification={'actual_whole_robot_payload_radius_m':1.0},
            torque_validation={'success':True,'geometry_frozen':True,'maximum_position_delta':0.})
        self.assertTrue(certified_cache_result(result))
        for field,value in [('motion_duration_s',61.),('navigation_path_validated',True)]:
            changed=dict(result);changed[field]=value;self.assertFalse(certified_cache_result(changed))
        changed=dict(result);changed['final_snapshot']={'attachments':[{}]}
        self.assertFalse(certified_cache_result(changed))
        changed=dict(result);changed['torque_validation']={'success':False}
        self.assertFalse(certified_cache_result(changed))

    def test_viewer_loads_each_round_timing_instead_of_reusing_the_aggregate(self):
        import tempfile
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            rounds=[]
            for name,total in [('a.json',1000.),('b.json',2000.)]:
                (root/name).write_text(json.dumps({'total_ms':total,'stage_timing_ms':{'initialize':total}}))
                rounds.append({'path':name})
            aggregate={'rounds':rounds,'round_frame_ranges':[{},{}]}
            loaded=load_round_results(aggregate,root/'five-columns.json')
            self.assertIn('1.000s',overall_timing_markdown(loaded[0]))
            self.assertIn('2.000s',overall_timing_markdown(loaded[1]))
            rounds[1]['path']='missing.json'
            with self.assertRaises(FileNotFoundError):load_round_results(aggregate,root/'five-columns.json')

    def test_viewer_timing_summary_separates_additive_and_diagnostic_costs(self):
        result=dict(total_ms=1000.,motion_duration_s=2.,
            stage_timing_ms={'initialize':100.,'right_precontact':900.},
            stage_breakdown_ms={'right_precontact':{'search':600.,'audit':100.,'collision':450.}},
            stage_timing_spans=[{'stage':'initialize','start_ms':0.,'wall_ms':100.},
                                {'stage':'right_precontact','start_ms':100.,'wall_ms':900.}],
            timed_segments=[{'name':'initialize','duration_s':0.},
                            {'name':'right_precontact','duration_s':2.}],
            timing_ms={'search':600.})
        overall=overall_timing_markdown(result);detail=stage_timing_markdown(result,'right_precontact')
        self.assertIn('1.000s',overall)
        self.assertIn('RRT/任务空间搜索 | 600.00 ms',detail)
        self.assertIn('候选生成、检查器构建、时间化/RNEA及其他 | 200.00 ms',detail)
        self.assertIn('碰撞/FK诊断：450.00 ms',detail)

    def test_future_lowering_is_lazy_and_stops_after_selected_endpoint_succeeds(self):
        task=DoorTask.__new__(DoorTask)
        task.support_pose=np.eye(4);task.lower_distance=.4;task.lift_grid=np.array([-.5,-.3])
        limits=SimpleNamespace(position=(torch.tensor([-1.]),torch.tensor([0.])))
        task.validity=lambda:SimpleNamespace(checker=SimpleNamespace(
            kinematics=SimpleNamespace(get_joint_limits=lambda:limits)))
        calls=[]
        def extract(candidate,*args,**kwargs):
            calls.append(kwargs['lift_end']);return np.vstack([candidate,candidate]),{'ok':True}
        task.planner=SimpleNamespace(analytic_extract=extract)
        rows,attempts=task.validate_future_lowering(np.r_[-.1,np.zeros(14)])
        self.assertEqual(len(calls),1)
        self.assertEqual(len(attempts),1)
        self.assertIsNotNone(rows)
        np.testing.assert_allclose(task.support_pose,np.eye(4))

    def test_upper_precontact_stages_lower_arm_before_right_cartesian_approach(self):
        task=DoorTask.__new__(DoorTask);task.q=np.zeros(15);task.q[RIGHT]=.1
        task.request=SimpleNamespace(seed=11);task.planner=SimpleNamespace(active_side='right')
        task.tensor=lambda values:torch.as_tensor(values)
        validity=SimpleNamespace(checker=object(),mask=lambda rows:torch.ones(len(rows),dtype=torch.bool))
        staged=np.zeros((2,15));staged[0,LEFT]=.1;staged[0,RIGHT]=.2
        staged[1,LEFT]=.3;staged[1,RIGHT]=.4
        calls=[]
        def plan(planner,start,goals,*args,active_indices=None,**kwargs):
            calls.append((planner.active_side,list(active_indices),np.asarray(start).copy()))
            return np.vstack([start,np.asarray(goals)[0]]),{'side':planner.active_side}
        with patch('curobo_core.door_recenter.plan_cartesian_transport',side_effect=plan):
            path,stats=task.sequential_upper_precontact(staged,validity)
        self.assertEqual([(x[0],x[1]) for x in calls],[('left',LEFT),('right',RIGHT)])
        np.testing.assert_allclose(path[0],task.q)
        np.testing.assert_allclose(path[-1,LEFT],path[-2,LEFT])
        np.testing.assert_allclose(path[-1,RIGHT],staged[0,RIGHT])
        self.assertEqual(stats['split_index'],1)
        self.assertEqual(task.planner.active_side,'right')

    def test_rotation_metric_detects_tool_backtracking(self):
        task=DoorTask.__new__(DoorTask)
        angles=iter((0.,math.pi/2,0.))
        task.tool=lambda row,side:np.block([[Rotation.from_euler('z',next(angles)).as_matrix(),np.zeros((3,1))],[np.zeros((1,3)),np.ones((1,1))]])
        metric=task.path_rotation_metrics(np.zeros((3,15)),'left')
        self.assertAlmostEqual(metric['total_deg'],180.,places=5)
        self.assertAlmostEqual(metric['net_deg'],0.,places=5)
        self.assertAlmostEqual(metric['extra_deg'],180.,places=5)

    def test_frozen_support_and_lift_are_enforced(self):
        rows = np.zeros((2,15)); rows[-1,1] = .1
        self.assertEqual(check_frozen(rows,np.zeros(15),LEFT),0)
        for index in [0,8,14]:
            changed = rows.copy(); changed[-1,index] = .01
            with self.assertRaises(ValueError): check_frozen(changed,np.zeros(15),LEFT)

    def test_cartesian_mapping_cannot_move_lift_or_support_arm(self):
        space = CartesianTransportSpace.__new__(CartesianTransportSpace)
        space.start = np.arange(15,dtype=float)/100
        space.goal_rows = np.ones((1,15))*2
        space.active_indices = LEFT
        space.derived_indices = ()
        space.row_mapper = None
        space.arm, space.idle = slice(1,8), RIGHT
        space.side_index, space.psi_start, space.arm_branch = 0, 0., (1,1,1)
        space.slerp = Slerp([0,1],Rotation.from_euler('z',np.array([[0.],[.2]])))
        received = []
        def inverse(lift):
            received.append(lift.copy())
            return np.repeat(np.eye(4)[None],len(lift),axis=0)
        space.carriage = SimpleNamespace(inverse=inverse)
        space.ik = SimpleNamespace(solve=lambda *args: (np.zeros((2,7)),np.ones(2,dtype=bool)))
        states = np.array([[.3,.4,.5,0.,-.9,0.],[.5,.4,.6,1.,.8,0.]])
        rows, found = space.joints(states)
        np.testing.assert_allclose(received[0],space.start[0])
        np.testing.assert_allclose(rows[:,RIGHT],np.tile(space.start[RIGHT],(2,1)))
        np.testing.assert_allclose(rows[:,0],space.start[0])
        self.assertTrue(found.all())

    def test_cartesian_mapper_may_derive_support_arm_without_freeing_it(self):
        space = CartesianTransportSpace.__new__(CartesianTransportSpace)
        space.start = np.zeros(15); space.goal_rows = np.zeros((1,15))
        space.active_indices = [0]+LEFT; space.derived_indices = tuple(RIGHT)
        space.arm, space.idle = slice(1,8), RIGHT
        space.side_index, space.psi_start, space.arm_branch = 0, 0., (1,1,1)
        space.slerp = Slerp([0,1],Rotation.from_euler('z',np.array([[0.],[.2]])))
        space.carriage = SimpleNamespace(inverse=lambda lift:np.repeat(np.eye(4)[None],len(lift),axis=0))
        space.ik = SimpleNamespace(solve=lambda *args:(np.ones((2,7)),np.ones(2,dtype=bool)))
        def mapper(rows):
            rows=rows.copy();rows[:,RIGHT]=2.;return rows,np.ones(len(rows),dtype=bool)
        space.row_mapper=mapper
        rows,found=space.joints(np.array([[0,0,0,0,-.3,0],[.1,0,0,1,-.2,0]]))
        np.testing.assert_allclose(rows[:,RIGHT],2.)
        self.assertTrue(found.all())

    def test_support_validity_maps_eight_search_variables_to_full_rows(self):
        import torch
        class Full:
            weights=torch.ones(15)
            stability_weight=0.
            checker=SimpleNamespace()
            def evaluate(self,rows):
                return rows[:,8].eq(3),torch.zeros(len(rows))
        reference=np.zeros(15)
        mapper=lambda rows:(np.column_stack([rows[:,:8],np.full((len(rows),7),3.)]),
                            np.ones(len(rows),dtype=bool))
        validity=SupportCompensatedValidity(Full(),reference,mapper)
        values=torch.tensor([[-.2,*([1.]*7)],[-.1,*([2.]*7)]],device='cuda')
        expanded=validity.expand(values)
        self.assertEqual(expanded.shape,(2,15))
        np.testing.assert_allclose(expanded[:,RIGHT].cpu(),3.)
        self.assertTrue(validity.mask(values).all())

    def test_suffix_uses_actual_tool_offsets_and_reuses_methods(self):
        planner = DoorPlanner.__new__(DoorPlanner)
        planner.held = tuple(AttachedObject(f'wall_box_{i:02d}',(.3,.4,.4),side+'_tool0',Pose((.02,.03,.14)))
                             for side,i in [('left',17),('right',22)])
        np.testing.assert_allclose(planner.payload_transform('left')[:3,3],[.02,.03,.14])
        self.assertEqual(len(planner.task_attachments({'left':17,'right':22})),2)
        with self.assertRaises(ValueError): planner.task_attachments({'left':17})
        self.assertIs(DoorPlanner.conveyor_suffix,ConveyorCyclePlanner.conveyor_suffix)

    def test_off_center_contacts_do_not_shift_ordinary_placement_targets(self):
        from v3_task_scene import grasp_transform
        planner = DoorPlanner.__new__(DoorPlanner)
        policy=json.loads((ROOT/'config/placement_task_policy.json').read_text())['side']
        planner.carry_target_poses = {side:pose_matrix(Pose(
            (policy['position']['x'],policy['position']['y_'+side],policy['position']['z']),
            tuple(policy['quaternion_wxyz']))) for side in ('left','right')}
        planner.held = (AttachedObject('wall_box_17',(.3,.4,.4),'left_tool0',Pose((.03,.01,.15))),)
        ordinary = planner.carry_target_poses['left'] @ grasp_transform('left',False)
        target = planner.placement_box_seed('left',np.eye(4))
        np.testing.assert_allclose(target,ordinary)
        tool_goal = target @ np.linalg.inv(planner.payload_transform('left'))
        np.testing.assert_allclose(tool_goal @ planner.payload_transform('left'),ordinary)

    def test_shared_parking_keeps_ordinary_strict_and_bounds_door_correction(self):
        ordinary = ConveyorCyclePlanner.__new__(ConveyorCyclePlanner)
        with self.assertRaises(Exception):
            ordinary.shared_parking_pose([np.array([0.,-.01]),np.array([0.,.01])])
        door = DoorPlanner.__new__(DoorPlanner)
        parking,residual=door.shared_parking_pose([np.array([.2,-.013]),np.array([.2,.013])])
        np.testing.assert_allclose(parking,[.2,0.])
        self.assertAlmostEqual(residual,.013)
        with self.assertRaises(Exception):
            door.shared_parking_pose([np.array([0.,-.1]),np.array([0.,.1])])

    def test_scene_has_no_duplicate_or_early_release(self):
        state = RobotState.from_mapping(dict.fromkeys(ACTIVE_JOINTS,0.),123)
        boxes = tuple(SceneObject(f'wall_box_{i:02d}',(.3,.4,.4),Pose((1.,0.,z)))
                      for i,z in [(17,1.4),(22,1.8)])
        store = SceneStore(SceneSnapshot('test',state,boxes))
        for side,i in [('right',22),('left',17)]:
            store.attach(AttachedObject(f'wall_box_{i:02d}',(.3,.4,.4),side+'_tool0',Pose()),store.snapshot().revision)
        self.assertFalse(store.snapshot().objects)
        store.update_state(state.with_positions({'updown':-.4}),store.snapshot().revision)
        self.assertEqual(len(store.snapshot().attachments),2)

    def test_order_and_fail_closed_before_navigation(self):
        class Flow(DoorTask):
            def __init__(self, fail=False):
                self.log=[]; self.fail_recenter=fail; self.q=np.zeros(15); self.report={}
                self.request=SimpleNamespace(upper_box=22,lower_box=17,snapshot=SimpleNamespace(object=lambda key:SimpleNamespace(pose=Pose((0,0,1.8 if key.endswith('22') else 1.4)))))
                self.store=SimpleNamespace(snapshot=lambda:SimpleNamespace(attachments=[SimpleNamespace(object_id='wall_box_22')]))
            def enter(self, phase): self.log.append(phase)
            def audit(self,*args,**kwargs): pass
            def append(self,rows): pass
            def contact(self,side,*args): self.log.append(side+'_contact')
            def contact_upper_ranked(self,*args): self.log.append('right_contact')
            def contact_lower_ranked(self,*args): self.log.append('left_contact')
            def extract(self,side,*args): self.log.append(side+'_extract')
            def box_pose(self,*args): return np.eye(4)
            def recenter(self,side,*args,**kwargs):
                self.log.append(side+'_recenter')
                if self.fail_recenter: raise RuntimeError('blocked')
            def lower_upper_with_lower_hold(self): self.log.append('right_lower')
            def extract_upper_with_lower_hold(self): self.log.append('right_extract')
            def recenter_latest_pair(self): self.log.append('right_recenter')
            def align_pair(self): self.log.append('align')
            def append_suffix(self): self.log.append('ordinary_suffix')
        flow=Flow(); flow.run()
        self.assertEqual(flow.log,['initialize','right_contact','left_contact','left_extract','left_recenter',
                                   'right_lower','right_extract','right_recenter','align','ordinary_suffix'])
        flow=Flow(True)
        with self.assertRaises(RuntimeError): flow.run()
        self.assertNotIn('ordinary_suffix',flow.log)
        self.assertNotIn('right_lower',flow.log)


class DynamicsTests(unittest.TestCase):
    def test_dynamics_gate_preserves_positions_and_uses_actual_active_side(self):
        task=DoorTask.__new__(DoorTask)
        task.planner=SimpleNamespace(args=SimpleNamespace(),box_fit={'dimensions_m':[.3,.4,.4]})
        attachment={'parent_link':'right_tool0','tool_to_object':{'position':[0,0,.15],'quaternion_wxyz':[1,0,0,0]}}
        task.report=dict(time_parameterized=True,joint_names=list(ACTIVE_JOINTS),
            frames=[np.zeros(15).tolist(),np.ones(15).tolist(),np.ones(15).tolist(),np.ones(15).tolist()],
            velocities=[np.zeros(15).tolist()]*4,accelerations=[np.zeros(15).tolist()]*4,
            jerks=[np.zeros(15).tolist()]*4,time_from_start_s=[0.,.5,1.,1.],
            attachments_by_frame=[[attachment],[attachment],[attachment],[]],
            timed_segments=[dict(name='move',frame_start=0,frame_end=2,duration_s=1.,method='test'),
                            dict(name='release',frame_start=3,frame_end=3,duration_s=0.,method='event')])
        task.fail=lambda message:(_ for _ in ()).throw(RuntimeError(message))
        calls=[]
        class Model:
            def constrain(self,segment,sides,tool_to_box,deadline=None):
                calls.append((sides,tool_to_box));audit={'maximum_utilization':.4,'maximum_gravity_utilization':.3}
                return segment,audit,{'success':True,'retiming_mode':'none','time_scale':1.,'after':audit}
        with patch('curobo_core.door_recenter.default_torque_model',return_value=Model()):
            task.apply_dynamics()
        self.assertEqual(calls[0][0],('right',))
        self.assertTrue(task.report['torque_validation']['success'])
        np.testing.assert_allclose(task.report['frames'][1],np.ones(15))

class TimingTests(unittest.TestCase):
    def test_timing_preserves_events_and_rechecks_frozen_joints(self):
        import torch
        task=DoorTask.__new__(DoorTask)
        q=np.zeros(15); end=q.copy();end[1]=.1
        state=RobotState.from_mapping(dict(zip(ACTIVE_JOINTS,q)),123)
        snapshot=SceneSnapshot('test',state,())
        task.store=SceneStore(snapshot);task.support_pose=None;task.contact_pair=None;task.q=end
        caps=SimpleNamespace(**{k:torch.tensor([[-1.]*15,[1.]*15]) for k in ('velocity','acceleration','jerk')})
        valid=SimpleNamespace(checker=SimpleNamespace(kinematics=SimpleNamespace(get_joint_limits=lambda:caps)),
                              mask=lambda rows:torch.ones(len(rows),dtype=torch.bool))
        task.planner=SimpleNamespace(snapshot=snapshot)
        task.validity=lambda:valid
        task.tensor=lambda x:torch.tensor(np.asarray(x),dtype=torch.float32)
        checked=[]
        task.audit=lambda rows,active,reference:checked.append(check_frozen(rows,reference,active))
        task.fail=lambda message:(_ for _ in ()).throw(RuntimeError(message))
        task.motion_segments=[(0,0,snapshot,None,None),(1,2,snapshot,None,None)]
        task.report=dict(joint_names=list(ACTIVE_JOINTS),frames=[q.tolist(),q.tolist(),end.tolist()],
            phases=['initialize','left_recenter','left_recenter'],payload=[False,True,True],
            attachments_by_frame=[[],[{'object_id':'lower'}],[{'object_id':'lower'}]],
            predicted_scene_events=[{'phase':'left_attach','frame_index':1}])
        task.finalize_timing()
        self.assertTrue(task.report['time_parameterized'])
        self.assertGreater(task.report['motion_duration_s'],0)
        np.testing.assert_allclose(task.report['frames'][-1],end,atol=1e-8)
        self.assertEqual(task.report['predicted_scene_events'][0]['frame_index'],1)
        self.assertIn('left_recenter',[item['phase'] for item in task.report['retiming_geometry_reuse']])
        self.assertTrue(np.isfinite(task.report['time_from_start_s']).all())

    def test_comparison_never_pairs_different_input_identities(self):
        import tempfile
        from v3_door_recenter_compare import summarize
        with tempfile.TemporaryDirectory() as d:
            paths=[]
            for algorithm,identity in [('joint_rrt','one'),('cartesian_rrt','two')]:
                path=Path(d)/(algorithm+'.json');paths.append(path)
                path.write_text(json.dumps(dict(seed=11,recenter_algorithm=algorithm,success=False,
                    recenter_searches=[dict(input_identity=identity,algorithm=algorithm,success=False)])))
            report=summarize(paths)
            self.assertFalse(report['paired_inputs'])
            self.assertIsNone(report['summary']['joint_rrt']['paired_success_medians']['actual_search_ms'])

if __name__=='__main__': unittest.main()
