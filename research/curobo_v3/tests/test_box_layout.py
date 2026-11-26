"""CPU-only editor contract checks; no CUDA planner is imported by the UI module."""
from dataclasses import asdict
import copy
import sys
from pathlib import Path
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from curobo_core.box_pick import BOX_DIMENSIONS, BoxPick, native_rejection_reasons, paired_carry_candidates
from curobo_core.layout_contract import base_front_x
from curobo_core.torque import TorqueModel
from v3_box_layout import parse_layout, layout_signature, status_markdown, scaled_estimate


class BoxLayoutTest(unittest.TestCase):
    def setUp(self):
        self.doc = dict(frame_id='world', boxes=[dict(size='40x40x30', side=s, center_m=[base_front_x()+.68, sign*.45, 1.02])
                                               for s, sign in [('left', 1), ('right', -1)]])

    def test_swap_only_changes_length_height_not_extraction_or_attachment(self):
        for size, (x, y, z) in BOX_DIMENSIONS.items():
            for side in ('left', 'right'):
                for height in (.491, 1.02):
                    for swapped in (False, True):
                        task = BoxPick(size, (1., .45, height), side, swap_length_height=swapped)
                        self.assertEqual(task.dimensions_m, (x, z, y) if swapped else (x, y, z))
                        self.assertEqual(task.extraction_distance_m, x)
                        actual = task.contact_tool()@task.tool_to_box()
                        np.testing.assert_allclose(actual[:3, 3], task.center_m, atol=1e-12)
                        np.testing.assert_allclose(actual[:3, :3], np.eye(3), atol=1e-12)
                        self.assertEqual(BoxPick(**asdict(task)), task)
        with self.assertRaises(ValueError):BoxPick('40x40x30', (1, 0, 1), 'left', swap_length_height='false')

    def test_endpoint_proposals_stay_inside_existing_pose_and_lift_gates(self):
        from curobo_core.box_pick import extraction_endpoint_proposals
        picks = parse_layout(self.doc)
        spheres = np.array([[.6, 0., 1.4, .1], [.5, 0., .8, .15]])
        result = extraction_endpoint_proposals(picks, spheres, [0., 0., 1.], (-1., 0.), [-.7])
        self.assertEqual(len(result), 1)
        for proposal in result:
            self.assertTrue(-1. <= proposal['lift_m'] <= 0.)
            for side, pose in proposal['boxes'].items():
                pose = np.asarray(pose)
                nominal = np.asarray(picks[side].center_m)-[picks[side].extraction_distance_m, 0., 0.]
                self.assertLessEqual(np.linalg.norm(pose[:3, 3]-nominal), .001800000001)
                self.assertLessEqual(np.rad2deg(Rotation.from_matrix(pose[:3, :3]).magnitude()), .950000001)
            self.assertIn('no IK', proposal['scope'])
        with self.assertRaises(ValueError):
            extraction_endpoint_proposals(picks, spheres, [0., 0., 0.], (-1., 0.), [-.7])

    def test_transport_starts_at_actual_endpoint_without_reverse_roots(self):
        import ast
        from types import SimpleNamespace as NS
        import torch
        path = Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
        tree = ast.parse(path.read_text())
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'BoxLayoutPlanner')
        node = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'pair_path')
        calls = []
        wrong_root = False
        def search(roots, goals, *args, **kwargs):
            self.assertEqual(roots.shape, (15,))
            self.assertEqual(args[-1], 20261123)
            self.assertEqual(args[-2], 12.5)
            calls.append(roots.numpy().copy())
            if wrong_root:return np.stack((roots.numpy()+.1, goals[0].numpy())), {}
            return np.stack((roots.numpy(), goals[0].numpy())), {}
        env = dict(np=np, torch=NS(float32=torch.float32, as_tensor=lambda x,device,dtype=None:torch.as_tensor(x,dtype=dtype)),
            batched_rrt_connect_multi_goal=search, densify=lambda rows:rows)
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), env)
        function = env['pair_path']
        checker = NS(kinematics=NS(get_joint_limits=lambda:NS(position=(torch.full((15,), -1.), torch.ones(15)))))
        planner = NS(path_validity=lambda *args:NS())
        start = np.linspace(-.2, .2, 15, dtype=np.float32)
        goal = np.ones((1, 15), np.float32)*.5
        rows, detail = function(planner, start, goal, checker, seed=20261123, budget_s=12.5)
        np.testing.assert_array_equal(calls[0], start)
        np.testing.assert_array_equal(rows[0], start)
        np.testing.assert_array_equal(rows[-1], goal[0])
        self.assertEqual(detail['budget_s'], 12.5)
        self.assertNotIn('escape_rows', [arg.arg for arg in node.args.args])
        for budget in (0., -1., float('nan')):
            with self.assertRaises(ValueError):function(planner, start, goal, checker, budget_s=budget)
        wrong_root = True
        with self.assertRaisesRegex(RuntimeError, 'actual requested state'):
            function(planner, start, goal, checker, seed=20261123, budget_s=12.5)

    def test_lateral_proposal_still_checks_native_sampler_joint_margin(self):
        import ast
        import torch
        file=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
        cls=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='BoxLayoutPlanner')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='forward_lateral_prefix')
        callback=next(n for n in method.body if isinstance(n,ast.FunctionDef) and n.name=='candidate_filter')
        check=next(n for n in callback.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='joint' for t in n.targets))
        values=torch.zeros((3,15));values[:,0]=torch.tensor([1e-6,2e-5,1.-1e-6])
        env=dict(values=values,lower=torch.zeros(15)-torch.tensor([0.]+[1.]*14),upper=torch.ones(15))
        exec(compile(ast.Module(body=[check],type_ignores=[]),str(file),'exec'),env)
        torch.testing.assert_close(env['joint'],torch.tensor([False,True,False]))

    def test_alignment_candidate_rotates_both_cartons_without_changing_centers(self):
        import ast
        file=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
        cls=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='BoxLayoutPlanner')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='forward_lateral_prefix')
        node=next(n for n in ast.walk(method) if isinstance(n,ast.If) and isinstance(n.test,ast.Name) and n.test.id=='align_moving'
            and any(isinstance(child,ast.For) for child in n.body))
        for enabled in (False,True):
            ends={s:np.eye(4) for s in ('left','right')}
            for i,T in enumerate(ends.values()):T[:3,:3]=Rotation.from_euler('y',i+1,degrees=True).as_matrix();T[:3,3]=[1.,i*.2,.6+i]
            before={s:T.copy() for s,T in ends.items()}
            env=dict(np=np,align_moving=enabled,ends=ends)
            exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec'),env)
            for side,T in ends.items():
                np.testing.assert_array_equal(T[:3,3],before[side][:3,3])
                np.testing.assert_array_equal(T[:3,:3],np.eye(3) if enabled else before[side][:3,:3])

    def test_alignment_proposal_clearance_is_additional_and_consumed_before_fk_fallback(self):
        import ast,torch
        from types import SimpleNamespace as NS
        file=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
        cls=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='BoxLayoutPlanner')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='forward_lateral_prefix')
        node=next(n for n in method.body if isinstance(n,ast.FunctionDef) and n.name=='candidate_filter')
        values=torch.zeros((2,15));state=NS(robot_spheres=torch.tensor([-10e-6,-4e-6]).reshape(1,2,1,1),tool_poses=object())
        def classify(current):return torch.ones((1,2),dtype=torch.bool),torch.zeros((1,2),dtype=torch.bool),current.cuda_robot_model_state.robot_spheres[:,:,:,0].amax(-1)
        def payload(checker,rows,**kw):kw['state'].robot_spheres.fill_(999.);return torch.tensor([True,True])
        planner=NS(picks={'left':NS(dimensions_m=(.3,.4,.4),center_m=(1.,0.,1.))},
            native_margin_mask=lambda *args,**kw:torch.tensor([True,True]),payload_search_mask=payload)
        stats=dict(rows=0,source_rejected=0,native_margin_rejected=0,full_payload_rejected=0,joint_bound_rejected=0,proposal_clearance_rejected=0)
        env=dict(np=np,torch=torch,SimpleNamespace=NS,self=planner,checker=object(),lower=torch.full((15,),-1.),upper=torch.ones(15),
            limits={'left':.001},filter_statistics=stats,clearance_layer=NS(classify_with_robot_depth=classify),
            box_overlap_depth=lambda *args:np.zeros(2))
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec'),env)
        actual={'left':np.repeat(np.eye(4)[None],2,axis=0)}
        result=env['candidate_filter'](values,state,actual)
        np.testing.assert_array_equal(result,[True,False]);self.assertEqual(stats['proposal_clearance_rejected'],1)
        self.assertTrue(bool((state.robot_spheres==999.).all()))
        env['clearance_layer']=None
        planner.native_margin_mask=lambda *args,**kw:torch.tensor([True,False])
        result=env['candidate_filter'](values,state,actual)
        np.testing.assert_array_equal(result,[True,False])  # A reserve can never restore an originally invalid row.

    def test_fresh_forward_root_keeps_original_root_and_true_connecting_prefix(self):
        import ast
        import torch
        from types import SimpleNamespace as NS
        path=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
        cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='BoxLayoutPlanner')
        node=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='pair_path')
        start=np.zeros(15,dtype=np.float32);prefix=np.stack((start,start+.02));goal=np.ones((1,15),dtype=np.float32)*.5
        selected=[1];calls=[]
        def search(roots,goals,*args,**kwargs):
            calls.append(roots.numpy().copy())
            return np.stack((roots[selected[0]].numpy(),goals[0].numpy())),{}
        proxy=NS(float32=torch.float32,as_tensor=lambda value,device,dtype=None:torch.as_tensor(value,dtype=dtype))
        env=dict(np=np,torch=proxy,batched_rrt_connect_multi_goal=search,densify=lambda values:values)
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),env)
        checker=NS(guidance_payload=True,kinematics=NS(get_joint_limits=lambda:NS(position=(torch.full((15,),-1.),torch.ones(15)))))
        generated=[]
        def propose(values,checker,**kwargs):generated.append(values.copy());return prefix.copy(),dict(fresh=True)
        planner=NS(path_validity=lambda *args:NS(),forward_lateral_prefix=propose)
        rows,detail=env['pair_path'](planner,start,goal,checker)
        np.testing.assert_array_equal(calls[0][0],start);np.testing.assert_array_equal(calls[0][1],prefix[-1])
        np.testing.assert_array_equal(rows[:2],prefix);self.assertEqual(detail['selected_start_root'],1)
        selected[0]=0
        rows,detail=env['pair_path'](planner,start,goal,checker)
        np.testing.assert_array_equal(rows[0],start);self.assertEqual(len(rows),2)
        self.assertEqual(len(generated),2,'the forward prefix must be regenerated, not cached as a trajectory')
        planner.forward_lateral_prefix=lambda *args,**kwargs:(prefix+.1,{})
        with self.assertRaisesRegex(RuntimeError,'actual extraction endpoint'):env['pair_path'](planner,start,goal,checker)

    def test_search_padding_duplicates_last_row_and_preserves_original_outputs(self):
        import ast
        from types import SimpleNamespace as NS
        import torch
        path = Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
        cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'BoxLayoutPlanner')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'path_validity')
        env = dict(torch=torch)
        exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), env)
        for count in (1, 512, 513, 2599, 4097):
            rows = torch.arange(count*15, dtype=torch.float32).reshape(count, 15)
            calls = []
            def evaluate(values, margin):
                calls.append(values.clone())
                return values[:, 0].remainder(2) == 0, values[:, 1]*2
            validity = NS(evaluate=evaluate)
            result = env['path_validity'](None, NS(guidance_payload=False), strict=validity)
            mask, stability = result.evaluate(rows)
            torch.testing.assert_close(mask, rows[:, 0].remainder(2) == 0)
            torch.testing.assert_close(stability, rows[:, 1]*2)
            self.assertEqual(len(calls[0]) % 512, 0)
            torch.testing.assert_close(calls[0][:count], rows)
            if count % 512:
                torch.testing.assert_close(calls[0][count:], rows[-1:].expand((-count)%512, -1))
            self.assertEqual(validity.search_padding_frames, (-count)%512)
            self.assertTrue(validity.validate_returned_path)

    def test_source_slot_observation_is_not_a_new_acceptance_gate(self):
        from curobo_core.box_pick import source_slot_observation
        pick = BoxPick('40x40x30', (1., 0., 1.), 'left')
        poses = np.repeat(np.eye(4)[None], 3, axis=0)
        poses[:, :3, 3] = [[.699, 0., 1.], [.8, 0., 1.], [.65, 0., 1.]]
        before = poses.copy()
        observation = source_slot_observation(pick, poses)
        self.assertTrue(observation['diagnostic_only'])
        self.assertTrue(observation['returns_after_clear'])
        self.assertEqual(observation['first_clear_frame'], 0)
        self.assertEqual(observation['first_return_frame'], 1)
        self.assertAlmostEqual(observation['max_center_forward_m'], .101)
        self.assertAlmostEqual(observation['maximum_slot_overlap_m'], .1)
        np.testing.assert_array_equal(poses, before)
        text = status_markdown(dict(events=[dict(stage='transport', kind='native', state='accepted',
            wall_seconds=1., details=dict(source_slot_observation={'left':observation}))]))
        for label in ('工艺诊断', '尚无最终工艺检查结果', '不是新增拒绝判据', '101.000mm', '100.000mm'):
            self.assertIn(label, text)
        poses[:, :3, 3] = [[.701, 0., 1.], [.69, 0., 1.], [.6, 0., 1.]]
        observation = source_slot_observation(pick, poses)
        self.assertFalse(observation['returns_after_clear'])
        self.assertEqual(observation['first_clear_frame'], 1)
        self.assertAlmostEqual(observation['initial_slot_overlap_m'], .001)
        self.assertEqual(observation['deeper_than_initial_m'], 0.)
        poses[:, :3, 3] = [[.699, 0., 1.], [1.1, .6, 1.], [.6, .6, 1.]]
        observation = source_slot_observation(pick, poses)
        self.assertGreater(observation['max_center_forward_m'], .4)
        self.assertFalse(observation['returns_after_clear'])
        self.assertEqual(observation['maximum_slot_overlap_m'], 0.)
        for bad in (np.zeros((0, 4, 4)), np.full((1, 4, 4), np.nan), np.zeros((3, 15))):
            with self.assertRaises(ValueError):source_slot_observation(pick, bad)

    def test_source_slot_process_gate_is_distinct_from_physical_success(self):
        from curobo_core.box_pick import source_slot_observation, source_slot_process_checks
        pick = BoxPick('40x40x30', (1., 0., 1.), 'left')
        def check(centers):
            poses = np.repeat(np.eye(4)[None], len(centers), axis=0)
            poses[:, :3, 3] = centers
            return source_slot_process_checks({'left':source_slot_observation(pick, poses)})
        reentered = check([[.699, 0., 1.], [.8, 0., 1.], [.65, 0., 1.]])
        self.assertFalse(reentered['valid'])
        self.assertEqual(len(reentered['rejection_reasons']), 2)
        self.assertFalse(reentered['diagnostic_only'])
        clearing = check([[.701, 0., 1.], [.69, 0., 1.], [.6, 0., 1.]])
        self.assertTrue(clearing['valid'])
        reentered_less = check([[.701, 0., 1.], [.69, 0., 1.], [.7005, 0., 1.]])
        self.assertFalse(reentered_less['valid'])
        self.assertEqual(len(reentered_less['rejection_reasons']), 1)
        outside = check([[.699, 0., 1.], [1.1, .6, 1.], [.6, .6, 1.]])
        self.assertTrue(outside['valid'])  # Forward X alone is not reentry into an OBB.
        never_clear = check([[.701, 0., 1.], [.7005, 0., 1.]])
        self.assertFalse(never_clear['valid'])
        self.assertIn('never fully separates', never_clear['rejection_reasons'][0])
        text = status_markdown(dict(events=[dict(stage='transport', state='accepted',
            details=dict(source_slot_observation=clearing['source_slot_observation'])),
            dict(stage='transport_process', state='accepted', details=clearing)]))
        self.assertIn('不加深原位重叠、完全离开后不回插', text)
        self.assertIn('passed', text)

    def test_transport_search_applies_source_slot_bound_and_keeps_physical_mask(self):
        import ast
        import torch
        from types import SimpleNamespace as NS
        path = Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
        cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'BoxLayoutPlanner')
        node = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'path_validity')
        proxy = NS(cat=torch.cat, as_tensor=lambda x,device,dtype=None:torch.as_tensor(x,dtype=dtype), float32=torch.float32)
        rotation_inputs = []
        env = dict(torch=proxy, ACTIVE_JOINTS=list(range(15)), BoxRotationCost=lambda *args,**kw:rotation_inputs.append(args[2:4]))
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), env)
        validity = NS(evaluate=lambda rows,margin:(rows[:, 1] >= 0, rows[:, 2].clone()))
        planner = NS(payload_search_mask=lambda checker,rows:torch.ones(len(rows),dtype=torch.bool),
            native_margin_mask=lambda checker,rows:torch.ones(len(rows),dtype=torch.bool),
            source_slot_depths=lambda checker,rows:rows[:, :1],
            static_load_mask=lambda checker,rows:rows[:, 2] >= 0)
        checker = NS(guidance_payload=True, task_tool_to_box={})
        result = env['path_validity'](planner, checker, strict=validity, start=np.array([.001]+[0.]*14,dtype=np.float64), goal=np.zeros(15,dtype=np.float64))
        self.assertTrue(all(value.dtype == torch.float32 for value in rotation_inputs[0]))
        rows = torch.zeros((5, 15));rows[:, 0] = torch.tensor([.001, .001002, -.1, -.2, -.3]);rows[3, 1] = -1;rows[4, 2] = -1
        mask, stability = result.evaluate(rows)
        torch.testing.assert_close(mask, torch.tensor([True, False, True, False, False]))
        torch.testing.assert_close(stability, rows[:, 2])

    def test_rrt_steering_preserves_legacy_and_adds_checked_microsteps(self):
        import ast
        import torch
        from types import SimpleNamespace as NS
        path = Path(__file__).resolve().parents[1]/'tools/v3_multiroot_search.py'
        tree = ast.parse(path.read_text())
        function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'batched_rrt_connect_multi_goal')
        setup = [n for n in function.body if isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id in ('step_choices', 'fine_step_limits', 'step_limits') for t in n.targets)]
        loop = next(n for n in function.body if isinstance(n, ast.While))
        scaling = next(n for n in loop.body if isinstance(n, ast.Assign)
            and any(isinstance(t, ast.Name) and t.id == 'scale' for t in n.targets))
        delta = torch.linspace(.1, .5, 1024)[:, None].expand(-1, 15).clone()
        weights = torch.ones(15)
        for choices in (None, (.28, .03, .01, .003)):
            validity = NS() if choices is None else NS(steering_steps=choices)
            env = dict(torch=torch, start=torch.zeros(15), validity=validity, delta=delta, weights=weights)
            exec(compile(ast.Module(body=[*setup, scaling],type_ignores=[]),str(path),'exec'),env)
            steps = (delta*env['scale'][:, None]).abs().amax(-1)
            torch.testing.assert_close(steps, delta.abs().amax(-1).clamp(max=.28), rtol=0, atol=0)
            if choices is not None:
                failed = next(n for n in loop.body if isinstance(n, ast.If) and ast.unparse(n.test) == 'not edge_valid.any()')
                activation = next(n for n in failed.body if isinstance(n, ast.If))
                for expand_start, count in ((False, 1), (True, 2), (True, 1)):
                    env.update(expand_start=expand_start, start_tree=torch.zeros((count, 15)), roots=torch.zeros((1, 15)))
                    exec(compile(ast.Module(body=[activation, scaling],type_ignores=[]),str(path),'exec'),env)
                    steps = (delta*env['scale'][:, None]).abs().amax(-1)
                    if not expand_start or count != 1:
                        torch.testing.assert_close(steps, delta.abs().amax(-1).clamp(max=.28), rtol=0, atol=0)
                self.assertLessEqual(float(steps[1::4].max()), .03000001)
                self.assertLessEqual(float(steps[2::4].max()), .01000001)
                self.assertLessEqual(float(steps[3::4].max()), .00300001)
                self.assertGreater(float(steps[::4].max()), .27)
        self.assertTrue(any(isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
            and n.func.attr == 'edges' for n in ast.walk(loop)))

    def test_worker_keeps_physical_and_process_failures_separate(self):
        import json
        import tempfile
        from types import SimpleNamespace as NS
        from unittest.mock import patch
        from v3_box_layout import solve_file
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp); source = root/'request.json'; source.write_text(json.dumps(self.doc))
            for process_ok in (True, False):
                report = dict(success=process_ok, timing=dict(success=True),
                    process_validation=dict(policy='no_deeper_overlap_and_no_return_after_clear_v1',valid=process_ok))
                planner = NS(plan_layout=lambda *args, **kwargs:report)
                fake_module = NS(BoxLayoutPlanner=lambda:planner)
                args = NS(solve_file=source, output=root/str(process_ok), max_contacts=16)
                with patch.dict(sys.modules, {'v3_box_layout_planner':fake_module}), \
                     patch('v3_box_layout.verify_layout_result', side_effect=ValueError('independent check failed')):
                    self.assertEqual(solve_file(args), 1)
                data = json.loads((args.output/'result.json').read_text())
                self.assertFalse(data['task_success'])
                self.assertEqual(data['physical_success'], not process_ok)
                if process_ok:self.assertIn('independent check failed', data['failure'])

    def test_adaptive_continuation_retains_only_checked_prefix_and_restores_sampler(self):
        import ast,time
        from types import SimpleNamespace as NS
        file=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
        node=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='adaptive_lateral')
        env=dict(np=np,time=time,interpolate_boxes=lambda starts,ends,values:{s:np.repeat(T[None],len(values),axis=0) for s,T in starts.items()})
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec'),env)
        calls=[];events=[]
        lm=NS(samples=512)
        def plan(case):
            grid=case['reference_fractions'];calls.append(case)
            assert len(grid)==lm.samples and grid[0]==0. and grid[-1]==1.
            assert (np.diff(grid)>0).all() and case['edge_resolution']==.001
            attempt=len(calls);fraction=0. if attempt==1 else (grid[20] if attempt==2 else 1.)
            start=case['start'];end=start.copy();end[0]+=fraction
            rows=[start.tolist()] if fraction==0 else [start.tolist(),end.tolist()]
            return dict(success=attempt==3,failure=None if attempt==3 else 'interpolated edge collision',
                frames=rows,progress=[0.] if fraction==0 else [0.,fraction],point_reports=[dict(point=1,progress=float(fraction))],
                max_reference_position_error_m=0.,max_reference_orientation_error_deg=0.,total_ms=1.,lm_ms=1.,candidate_filter_ms=0.,edge_audit_ms=0.)
        lm.plan=plan;case=dict(start=np.zeros(15),starts={'left':np.eye(4)},ends={'left':np.eye(4)})
        result=env['adaptive_lateral'](lm,case,progress=events.append)
        self.assertTrue(result['success']);self.assertEqual(lm.samples,512)
        self.assertEqual(len(result['frames']),3);self.assertEqual(result['reference_node_attempts'],3)
        self.assertEqual([e['state'] for e in events],['rejected','rejected','done'])
        np.testing.assert_array_equal(calls[2]['start'],np.asarray(result['frames'][1]))
        self.assertEqual(result['progress'][-1],1.)
        starts_rejected=[]
        def reject_start(case):
            starts_rejected.append(case)
            return dict(success=False,failure='starting state collision',frames=[case['start'].tolist()],progress=[0.],
                point_reports=[],max_reference_position_error_m=0.,max_reference_orientation_error_deg=0.,
                total_ms=1.,lm_ms=0.,candidate_filter_ms=0.,edge_audit_ms=0.)
        lm.plan=reject_start
        rejected=env['adaptive_lateral'](lm,case)
        self.assertFalse(rejected['success']);self.assertEqual(len(starts_rejected),1);self.assertEqual(lm.samples,512)
        def raises(case):raise RuntimeError('test failure')
        lm.plan=raises
        with self.assertRaises(RuntimeError):env['adaptive_lateral'](lm,case)
        self.assertEqual(lm.samples,512)

    def test_failed_lateral_proposals_try_alignment_then_original_grid(self):
        import ast
        from types import SimpleNamespace as NS
        file=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
        cls=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='BoxLayoutPlanner')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='forward_lateral_prefix')
        node=next(n for n in method.body if isinstance(n,ast.FunctionDef) and n.name=='failed')
        choice=next(n for n in method.body if isinstance(n,ast.If) and 'observation_epsilon_m' in ast.unparse(n.test))
        for depth,requested,expected in ((1.7e-6,True,False),(4.5e-6,True,False),(.0037,True,True),(.0037,False,False)):
            selection=dict(adaptive=requested,initial={'left':dict(initial_slot_overlap_m=depth,observation_epsilon_m=1e-6)},moving=['left'],report={})
            exec(compile(ast.Module(body=[choice],type_ignores=[]),str(file),'exec'),selection)
            self.assertEqual(selection['adaptive'],expected)
            self.assertEqual(selection['initial']['left']['observation_epsilon_m'],1e-6)
        calls=[];events=[];rows=np.zeros((2,15))
        def baseline(start,checker,**kw):calls.append(kw);return rows,dict(adaptive_lateral=False,wall_seconds=3.)
        env=dict(report=dict(stages=[]),adaptive=True,align_moving=False,progress=events.append,side='left',self=NS(forward_lateral_prefix=baseline),
            start=np.zeros(15),checker=object(),began=2.,time=NS(perf_counter=lambda:10.))
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec'),env)
        returned,detail=env['failed']('micro collision')
        self.assertIs(returned,rows);self.assertTrue(calls[0]['adaptive']);self.assertTrue(calls[0]['align_moving']);self.assertEqual(len(calls),1)
        self.assertEqual(detail['adaptive_attempt']['failure'],'micro collision');self.assertEqual(detail['wall_seconds'],8.)
        env['align_moving']=True;env['report']=dict(stages=[])
        env['failed']('alignment also rejected')
        self.assertFalse(calls[1]['adaptive']);self.assertFalse(calls[1]['align_moving'])
        env['adaptive']=False
        self.assertIsNone(env['failed']('still rejected')[0]);self.assertEqual(len(calls),2)

    def test_terminal_endpoint_guidance_tracks_original_self_geometry_and_pose_bounds(self):
        import ast
        from curobo_core.box_pick import extraction_endpoint_proposals
        from curobo_core.payload_geometry import box_sphere_depth
        file=Path(__file__).resolve().parents[1]/'tools/curobo_core/box_pick.py'
        function=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='extraction_endpoint_proposals')
        boxes=next(n for n in function.body if isinstance(n,ast.FunctionDef) and n.name=='boxes')
        residual=next(n for n in function.body if isinstance(n,ast.FunctionDef) and n.name=='residual')
        pick=BoxPick('40x40x30',(1.,0.,1.),'left');body=np.array([[.45,0.,1.,.04]])
        env=dict(np=np,Rotation=Rotation,picks={'left':pick},sides=('left',),origins={'left':np.array([.7,0.,1.])},
            spheres=body,axis=np.array([0.,0.,1.]),box_sphere_depth=box_sphere_depth,terminals=None,base=None)
        exec(compile(ast.Module(body=[boxes,residual],type_ignores=[]),str(file),'exec'),env)
        values=np.zeros(7);self.assertAlmostEqual(env['residual'](values),-60.)
        local=np.array([[-.25,0.,0.,.05]])
        env['terminals']={'left':(local,np.array([True]))}
        self.assertAlmostEqual(env['residual'](values),90.)  # Box clears while original terminal sphere collides.
        env['terminals']={'left':(local,np.array([False]))}
        self.assertAlmostEqual(env['residual'](values),-60.)  # Preserve the original self-ignore policy.
        geometry={'left':dict(spheres=local,mask=np.array([True]))}
        proposals=extraction_endpoint_proposals({'left':pick},body,[0.,0.,1.],(-1.,0.),[-.7],terminal_geometry=geometry)
        self.assertEqual(len(proposals),1);self.assertEqual(proposals[0]['guidance_geometry'],'box_and_original_terminal_self')
        for proposal in proposals:
            pose=np.asarray(proposal['boxes']['left'])
            self.assertTrue(-1.<=proposal['lift_m']<=0.)
            self.assertLessEqual(np.linalg.norm(pose[:3,3]-[.7,0.,1.]),.001800000001)
            self.assertLessEqual(np.rad2deg(Rotation.from_matrix(pose[:3,:3]).magnitude()),.950000001)
            self.assertIn('no IK',proposal['scope'])
        invalid=[{'right':geometry['left']}, {'left':dict(spheres=np.zeros((0,4)),mask=[True])},
            {'left':dict(spheres=[[0.,0.,0.,0.]],mask=[True])}, {'left':dict(spheres=[[0.,0.,float('nan'),.1]],mask=[True])},
            {'left':dict(spheres=local,mask=[True,False])}]
        for data in invalid:
            with self.assertRaises(ValueError):extraction_endpoint_proposals({'left':pick},body,[0.,0.,1.],(-1.,0.),[-.7],terminal_geometry=data)

    def test_terminal_guidance_stream_keeps_original_proposals_first_and_honors_ignore_masks(self):
        import ast,time
        from types import SimpleNamespace as NS
        file=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
        cls=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='BoxLayoutPlanner')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='reverse_extractions')
        stream=next(n for n in method.body if isinstance(n,ast.FunctionDef) and n.name=='proposal_stream')
        pick=BoxPick('40x40x30',(1.,0.,.6),'left');link_to_tool=np.eye(4);link_to_tool[:3,3]=[0.,0.,.2]
        transforms=[]
        def transform(*args):transforms.append(args);return link_to_tool
        planner=NS(picks={'left':pick},fk_robot=NS(get_transform=transform),_contact_reach_model={'lift_axis':np.array([0.,0.,1.])},
            robot={'kinematics':dict(self_collision_ignore={'left_link7':['head'],'arm_carriage':['left_link7']},
                collision_spheres={'left_link7':[dict(center=[0.,0.,0.],radius=.05)]})},
            box_fit={'left':dict(centers=[[0.,0.,0.]],radii=[.05])})
        calls=[];events=[];original=[dict(tag='original0'),dict(tag='original1')]
        def extra(*args,**kwargs):calls.append((args,kwargs));return [dict(tag='enhanced')]
        env=dict(np=np,time=time,self=planner,proposals=original,progress=events.append,
            extraction_endpoint_proposals=extra,spheres=np.zeros((3,4)),sphere_links=['head','head_yaw','arm_carriage'],bounds=(-1.,0.),seeds=[-.7],Rotation=Rotation,
            loaded=NS(base_cuboid_layer=NS(scene=NS(cuboid=[NS(pose=[0.,0.,0.,1.,0.,0.,0.],dims=[1.,1.,1.])]),
                mesh_checker=NS(collision_cost=NS(config=NS(activation_distance=np.array([.005])))))))
        exec(compile(ast.Module(body=[stream],type_ignores=[]),str(file),'exec'),env)
        generator=env['proposal_stream']()
        self.assertEqual(next(generator),original[0]);self.assertEqual(next(generator),original[1])
        self.assertEqual(calls,[]);self.assertEqual(transforms,[])  # No setup on successful original paths.
        nominal = next(generator)
        self.assertTrue(nominal['nominal_lift_grid'])
        np.testing.assert_allclose(np.asarray(nominal['boxes']['left'])[:3, 3],
                                   np.asarray(pick.center_m)-[pick.extraction_distance_m, 0., 0.])
        np.testing.assert_array_equal(np.asarray(nominal['boxes']['left'])[:3, :3], np.eye(3))
        self.assertEqual(calls,[]);self.assertEqual(transforms,[])
        self.assertEqual(next(generator),dict(tag='enhanced'))
        self.assertEqual(len(calls),1);self.assertEqual(transforms,[('left_tool0','left_link7')])
        geometry=calls[0][1]['terminal_geometry']['left']
        np.testing.assert_array_equal(geometry['mask'],[False,True,False])
        expected=np.linalg.inv(pick.tool_to_box())@np.linalg.inv(link_to_tool)
        np.testing.assert_allclose(geometry['spheres'][0,:3],expected[:3,3],atol=1e-15)
        self.assertEqual([e['state'] for e in events],['running','done'])
        self.assertNotIn('native_base_geometry',calls[0][1])
        self.assertEqual(next(generator),dict(tag='enhanced'))
        with self.assertRaises(StopIteration):next(generator)
        self.assertIn('native_base_geometry',calls[1][1])
        self.assertEqual(calls[1][1]['native_base_geometry']['activation_m'],.005)
        generator=env['proposal_stream']();next(generator);generator.close()
        self.assertEqual(len(calls),2)  # Early task completion never evaluates the fallback.

    def test_fine_pair_uniqueness_proof_preserves_inventory_and_legacy_duplicate_lifts(self):
        import ast
        from collections import Counter
        from types import SimpleNamespace as NS
        file=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
        cls=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='BoxLayoutPlanner')
        method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='enumerate_pair')
        first=next(i for i,n in enumerate(method.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='rows' for t in n.targets))
        last=next(i for i,n in enumerate(method.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='validity' for t in n.targets))
        block=compile(ast.Module(body=method.body[first:last],type_ignores=[]),str(file),'exec')
        rng=np.random.default_rng(82);reference=np.zeros(15,dtype=np.float32);weights=np.r_[np.deg2rad(15)/.1,np.ones(14)]
        for lifts in ((-.7,-.65),(-.7,-.7),(-.7,-.700000001)):
            left=np.unique(np.round(rng.normal(size=(7,7)),7),axis=0);right=np.unique(np.round(rng.normal(size=(9,7)),7),axis=0)
            candidates=[]
            for lift in lifts:
                rows=np.zeros((len(left)*len(right),15));rows[:,0]=lift
                rows[:,1:8]=np.repeat(left,len(right),axis=0);rows[:,8:]=np.tile(right,(len(left),1));candidates.append(rows)
            expected=np.unique(np.round(np.concatenate(candidates),7),axis=0).astype(np.float32)
            expected=expected[np.argsort(np.linalg.norm((expected-reference)*weights,axis=1))]
            for arm_limit in (None,32):
                planner=NS(last_pair_enumeration={});env=dict(np=np,self=planner,candidates=candidates,arm_limit=arm_limit,reference=reference,weights=weights)
                exec(block,env);actual=env['rows']
                self.assertEqual(Counter(map(tuple,actual)),Counter(map(tuple,expected)))
                costs=np.linalg.norm((actual-reference)*weights,axis=1)
                self.assertTrue((np.diff(costs)>=-1e-12).all())
                proved=arm_limit is None and len(set(np.round(lifts,7)))==len(lifts)
                if arm_limit is None:self.assertEqual(planner.last_pair_enumeration['fine_pair_uniqueness_proved'],proved)
                if not proved:np.testing.assert_array_equal(actual,expected)
        # Distinct7-decimal64-bit candidates that collapse in32-bit must both remain.
        rows=np.zeros((2,15));rows[:,1]=[3.,3.0000001]
        env=dict(np=np,self=NS(last_pair_enumeration={}),candidates=[rows],arm_limit=None,reference=reference,weights=weights)
        exec(block,env)
        self.assertEqual(len(env['rows']),2);np.testing.assert_array_equal(env['rows'][0],env['rows'][1])

    def test_nominal_reference_proposal_falls_back_and_restores_sampler_without_flipping_solver_success(self):
        import ast,torch,time
        from types import SimpleNamespace as NS
        from curobo_core.box_pick import source_slot_observation
        file=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
        cls=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='BoxLayoutPlanner')
        node=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='nominal_extraction')
        pick=BoxPick('40x40x30',(1.,0.,1.),'left')
        proxy=NS(as_tensor=lambda values,**kw:torch.as_tensor(values))
        for solver_ok,audit_ok,native_ok,overlap,legacy_ok in ((True,True,True,.2e-6,True),
                (False,True,True,.2e-6,True),(True,False,True,.2e-6,True),(True,True,False,.2e-6,True),
                (True,True,True,.75e-6,True),(False,True,True,.2e-6,False)):
            calls=[];audit_calls=[];events=[];lm=NS(samples=128)
            def plan(case):
                calls.append(lm.samples);ok=solver_ok if lm.samples==64 else legacy_ok
                return dict(success=ok,failure=None if ok else 'accurate solutions collide',
                    frames=np.full((2,15),float(lm.samples)).tolist(),point_reports=[],reference_samples=lm.samples)
            lm.plan=plan
            def audit(checker,rows):audit_calls.append(rows.copy());return dict(valid=audit_ok,frames=len(rows))
            checker=NS(task_tool_to_box={'left':pick.tool_to_box()})
            owner=NS(picks={'left':pick},loaded_audit=audit,native_margin_mask=lambda *args:torch.tensor([native_ok]))
            pose=np.eye(4)[None];pose[0,:3,3]=[.7+overlap,0.,1.]
            env=dict(np=np,torch=proxy,time=time,source_slot_observation=source_slot_observation,box_poses=lambda *args:{'left':pose})
            exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec'),env)
            result=env['nominal_extraction'](owner,lm,dict(checker=checker),progress=events.append)
            accepted=solver_ok and audit_ok and native_ok and overlap<=.5e-6
            self.assertEqual(lm.samples,128);self.assertEqual(calls,[64] if accepted else [64,128])
            self.assertEqual(result['success'],True if accepted else legacy_ok)
            self.assertEqual(events[0]['details']['solver_success'],solver_ok)
            self.assertEqual(events[0]['details']['proposal_accepted'],accepted)
            self.assertEqual(events[0]['state'],'done' if accepted else 'rejected')
            if accepted:
                self.assertTrue(result['proposal_payload_audit']['valid'])
                np.testing.assert_array_equal(audit_calls[0],np.full((2,15),64.))
            else:
                self.assertNotIn('proposal_payload_audit',result)
                self.assertIn('retry original128',events[0]['reason'])
                self.assertEqual(result['coarse_reference_attempt']['solver_success'],solver_ok)
        def raises(case):raise RuntimeError('GPU failure')
        lm.plan=raises
        with self.assertRaises(RuntimeError):env['nominal_extraction'](owner,lm,dict(checker=checker))
        self.assertEqual(lm.samples,128)
        text=status_markdown(dict(events=[dict(stage='extract_reference',kind='geometry_substage',state='rejected',
            wall_seconds=.2,reason='coarse endpoint near source-slot numerical boundary; retry original128')]))
        for label in ('抽离参考节点提案','计入抽离总耗时','包含提案筛选','retry original128'):self.assertIn(label,text)

    def test_strong_carry_lift_filter_uses_actual_targets_and_full_pose_envelope(self):
        import ast
        from types import SimpleNamespace as NS
        from curobo_core.payload_geometry import box_sphere_depth
        file=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
        cls=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='BoxLayoutPlanner')
        node=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='carry_lift_blocked')
        env=dict(np=np,box_sphere_depth=box_sphere_depth)
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec'),env)
        for size in ('40x40x30','46.5x25.5x58.2'):
            for swap in (False,True):
                for top,z in ((True,.6),(False,1.)):
                    pick=BoxPick(size,(1.,.5,z),'left',swap_length_height=swap)
                    body=np.eye(4);body[:3,3]=[.3,-.2,.7]
                    planner=NS(picks={'left':pick},robot={'kinematics':{'collision_spheres':{
                        link:[dict(center=[0.,0.,0.],radius=.05)] for link in ('head','head_yaw','arm_carriage')}}},
                        fk_robot=NS(get_transform=lambda *args:body,base_link='world'))
                    box=np.eye(4);box[:3,3]=body[:3,3]
                    def blocked():return env['carry_lift_blocked'](planner,{'left':box@np.linalg.inv(pick.tool_to_box())})
                    self.assertTrue(blocked())
                    # A3mm overlap is within the generous proposal envelope; actual gates still decide.
                    box[0,3]+=(pick.dimensions_m[0]/2+.05-.003)
                    self.assertFalse(blocked())
                    box[:3,:3]=Rotation.from_euler('y',-45,degrees=True).as_matrix();box[0,3]+=1.
                    self.assertFalse(blocked())
                    body[0,3]+=2.
                    self.assertFalse(blocked())

    def test_pair_ik_skips_later_predicates_only_after_the_whole_batch_is_rejected(self):
        import ast,itertools
        import torch
        from types import SimpleNamespace as NS
        file=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
        cls=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='BoxLayoutPlanner')
        node=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='enumerate_pair')
        proxy=NS(as_tensor=lambda values,**kw:torch.as_tensor(values))
        env=dict(np=np,torch=proxy,itertools=itertools)
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec'),env)
        checker=NS(kinematics=NS(get_joint_limits=lambda:NS(position=(torch.full((15,),-1.),torch.ones(15)))))
        for hard_ok,exact_ok,native_ok,held,expected_calls,accepted in (
                (False,True,True,True,[],0),(True,False,True,True,['audit'],0),
                (True,True,False,True,['audit','native'],0),
                (True,True,True,True,['audit','native','envelope'],1),
                (True,True,True,False,['audit','native'],1)):
            calls=[]
            def audit(*args):calls.append('audit');return dict(invalid_frame_indices=[] if exact_ok else [0])
            def native(checker,rows):calls.append('native');return torch.full((len(rows),),native_ok)
            def envelope(checker,rows):calls.append('envelope');return torch.zeros(len(rows))
            validity=NS(mask=lambda rows:torch.full((len(rows),),hard_ok))
            env['GpuValidity']=lambda *args,**kw:validity
            planner=NS(fk=lambda row:None,fk_robot=NS(get_transform=lambda *args:np.eye(4),base_link='world'),
                analytic=NS(solve=lambda *args:np.zeros((1,7))),path_validity=lambda *args:validity,carry_lift_blocked=lambda *args:False,
                loaded_audit=audit,native_margin_mask=native,full_envelope_radius=envelope,carry_radius_limit=1.05)
            rows=env['enumerate_pair'](planner,dict(left=np.eye(4),right=np.eye(4)),np.zeros(15),checker,lift_values=[0.],held=held)
            self.assertEqual(len(rows),accepted);self.assertEqual(calls,expected_calls)
            self.assertEqual(planner.last_pair_enumeration['hard_valid'],int(hard_ok))
            self.assertEqual(planner.last_pair_enumeration['exact_valid'],int(hard_ok and exact_ok))
            self.assertEqual(planner.last_pair_enumeration['native_valid'],int(hard_ok and exact_ok and native_ok))
            self.assertEqual(planner.last_pair_enumeration['envelope_valid'],accepted)

        # A partially valid batch must still reach every remaining predicate.
        calls.clear()
        planner.analytic.solve=lambda *args:np.array([[0.]*7,[.1]*7])
        validity.mask=lambda rows:torch.arange(len(rows)).remainder(2)==0
        rows=env['enumerate_pair'](planner,dict(left=np.eye(4),right=np.eye(4)),np.zeros(15),checker,lift_values=[0.],held=True)
        self.assertEqual(len(rows),2);self.assertEqual(calls,['audit','native','envelope'])
        self.assertEqual(planner.last_pair_enumeration['native_valid'],2)
        # Strong carry overlap skips the analytic proposal, but contact and non-held recovery are untouched.
        planner.carry_lift_blocked=lambda *args:True
        rows=env['enumerate_pair'](planner,dict(left=np.eye(4),right=np.eye(4)),np.zeros(15),checker,lift_values=[0.],held=True)
        self.assertEqual(len(rows),0);self.assertEqual(planner.last_pair_enumeration['pruned_lift_m'],[0.])
        rows=env['enumerate_pair'](planner,dict(left=np.eye(4),right=np.eye(4)),np.zeros(15),checker,lift_values=[0.],held=False)
        self.assertEqual(len(rows),2);self.assertEqual(planner.last_pair_enumeration['pruned_lift_m'],[])
        planner.target_validity=lambda *args,**kwargs:validity
        rows=env['enumerate_pair'](planner,dict(left=np.eye(4),right=np.eye(4)),np.zeros(15),checker,lift_values=[0.],contact=True,held=True)
        self.assertEqual(len(rows),2);self.assertEqual(planner.last_pair_enumeration['pruned_lift_m'],[])

    def test_layout_seed_control_projection_preserves_fixed_endpoints_and_default_fit(self):
        import ast
        from scipy.interpolate import BSpline
        file=Path(__file__).resolve().parents[1]/'tools/v3_fast_path.py'
        node=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='fit_path')
        env=dict(np=np,BSpline=BSpline)
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec'),env)
        fit=env['fit_path'];t=np.linspace(0,1,100)
        positions=np.c_[t,np.minimum(1.,2*np.sin(np.pi*t))]
        unbounded=fit(dict(positions=positions),32)
        bounded=fit(dict(positions=positions),32,control_bounds=(np.zeros(2),np.ones(2)))
        self.assertGreater(unbounded[0].max(),1.)
        controls,start,end,report,curve=bounded
        self.assertTrue(((controls>=0)&(controls<=1)).all())
        self.assertGreater(report['projected_seed_control_values'],0)
        np.testing.assert_array_equal(start,positions[0]);np.testing.assert_array_equal(end,positions[-1])
        np.testing.assert_allclose(curve([0.,1.]),positions[[0,-1]],atol=1e-15)
        samples=curve(np.linspace(0,1,4097))
        self.assertTrue(((samples>=-1e-15)&(samples<=1+1e-15)).all())
        np.testing.assert_array_equal(fit(dict(positions=positions),32,control_bounds=None)[0],unbounded[0])
        for bounds in ((np.zeros(3),np.ones(3)),(np.ones(2),np.zeros(2)),(np.zeros(2),np.full(2,np.nan)),(np.full(2,.1),np.ones(2))):
            with self.assertRaises(ValueError):fit(dict(positions=positions),32,control_bounds=bounds)

    def test_source_slot_gpu_sat_matches_cpu_without_initializing_cuda(self):
        import ast
        import torch
        from types import SimpleNamespace as NS
        from curobo_core.payload_geometry import box_overlap_depth, box_overlap_depth_tensor
        path = Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
        cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'BoxLayoutPlanner')
        node = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'source_slot_depths')
        def quaternion_matrix(values):
            return torch.as_tensor(Rotation.from_quat(np.roll(values.numpy(), -1, axis=1)).as_matrix(), dtype=values.dtype)
        env = dict(torch=torch, JointState=NS(from_position=lambda rows, **kw:rows), ACTIVE_JOINTS=list(range(15)),
            torch_quaternion_to_matrix=quaternion_matrix, box_overlap_depth_tensor=box_overlap_depth_tensor)
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), env)
        picks = parse_layout(self.doc)
        rng = np.random.default_rng(82); count = 1027
        poses, tools, expected = {}, {}, []
        for side, pick in picks.items():
            boxes = np.repeat(np.eye(4)[None], count, axis=0)
            boxes[:, :3, :3] = Rotation.random(count, random_state=rng).as_matrix()
            boxes[:, :3, 3] = np.asarray(pick.center_m)+rng.uniform(-.6, .6, (count, 3))
            tool = boxes@np.linalg.inv(pick.tool_to_box())
            tools[side] = (torch.as_tensor(tool[:, :3, 3], dtype=torch.float32),
                torch.as_tensor(np.roll(Rotation.from_matrix(tool[:, :3, :3]).as_quat(), 1, axis=1), dtype=torch.float32))
            expected.append(box_overlap_depth(boxes[:, :3, 3], boxes[:, :3, :3], np.asarray(pick.dimensions_m)/2,
                np.asarray(pick.center_m), np.eye(3), np.asarray(pick.dimensions_m)/2))
        def fk(rows):
            indices = rows[0, :, 0].long()
            return NS(tool_poses=NS(to_dict=lambda:{side+'_tool0':NS(position=p[indices], quaternion=q[indices])
                for side,(p,q) in tools.items()}))
        checker = NS(kinematics=NS(compute_kinematics=fk), task_tool_to_box={s:p.tool_to_box() for s,p in picks.items()})
        rows = torch.zeros((count, 15)); rows[:, 0] = torch.arange(count)
        actual = env['source_slot_depths'](NS(picks=picks), checker, rows).numpy()
        np.testing.assert_allclose(actual, np.stack(expected, axis=-1), atol=1e-6, rtol=0)

    def test_live_lateral_substage_timing_and_rejection_are_visible(self):
        text=status_markdown(dict(events=[
            dict(stage='lateral_right',kind='geometry',state='rejected',wall_seconds=.6,
                 reason='source_rejected=32, full_payload_rejected=6'),
            dict(stage='transport',kind='geometry',state='done',wall_seconds=8.4)]))
        for label in ('右箱正向横移候选','0.600','source_rejected=32','含横移/搜索','子项不重复累加'):
            self.assertIn(label,text)

    def test_timing_history_does_not_hide_initial_stages_or_early_rejections(self):
        events = [dict(stage='recover_pose_search', kind='geometry', state='done', wall_seconds=.5)]
        events += [dict(stage='candidate', kind='geometry', state='rejected', wall_seconds=.1,
                       attempt=i, reason='original-rejection-'+str(i)) for i in range(12)]
        events += [dict(stage='later-'+str(i), kind='geometry', state='done', wall_seconds=.2) for i in range(65)]
        text = status_markdown(dict(events=events))
        self.assertIn('容差内末态种子搜索', text)
        for i in range(12):self.assertIn('original-rejection-'+str(i), text)
        self.assertIn('later-64', text)

    def test_world_frame_shared_lift_contract_and_full_box_overlap(self):
        self.assertEqual(set(parse_layout(self.doc)), {'left', 'right'})
        for bad in ('frame', 'base', 'duplicate', 'overlap', 'floor', 'nan'):
            doc = copy.deepcopy(self.doc)
            if bad == 'frame':doc['frame_id'] = 'base_link'
            if bad == 'base':doc['world_from_robot_base'] = (np.eye(4)+1).tolist()
            if bad == 'duplicate':doc['boxes'][1]['side'] = 'left'
            if bad == 'overlap':doc['boxes'][1]['center_m'] = doc['boxes'][0]['center_m']
            if bad == 'floor':doc['boxes'][0]['center_m'][2] = .1
            if bad == 'nan':doc['boxes'][0]['center_m'][0] = float('nan')
            with self.assertRaises(ValueError, msg=bad):parse_layout(doc)
        touching = copy.deepcopy(self.doc)
        touching['boxes'][1]['center_m'][1] = .05
        parse_layout(touching)  # Mere contact is not an invented penetration rejection.

    def test_request_signature_is_order_independent_and_invalidates_changed_layout(self):
        old = layout_signature(self.doc)
        reordered = copy.deepcopy(self.doc); reordered['boxes'].reverse()
        self.assertEqual(layout_signature(reordered), old)
        self.doc['boxes'][0]['center_m'][1] += .01
        self.assertNotEqual(layout_signature(self.doc), old)

    def test_rejection_reason_keeps_the_actual_failed_native_gate_and_duration(self):
        stage = dict(success=False, time_optimization=dict(success=True, status=0, duration_s=2.5),
            trajectory_optimization=dict(solver_success=False, native_constraint_details={
                'constraints/project_base_and_ground': dict(maximum=.002)}), collision_invalid_frames=[1, 3])
        reasons = native_rejection_reasons(stage)
        self.assertTrue(any('project_base_and_ground' in r for r in reasons))
        self.assertTrue(any('2 samples' in r for r in reasons))
        stage['rejection_reasons'] = reasons
        text = status_markdown(dict(events=[dict(stage='transport', kind='native', state='rejected',
            attempt=2, wall_seconds=3.25, details=stage)]))
        for value in ('3.250', '2.5', 'project_base_and_ground', '拒绝'):self.assertIn(value, text)
        self.assertEqual(native_rejection_reasons(dict(success=True)), [])

    def test_joint_carry_proposals_change_only_held_xyz_and_preserve_explicit_override(self):
        picks = {row['side']: BoxPick(**row) for row in self.doc['boxes']}
        picks['right'] = BoxPick('46.5x25.5x58.2', (1.18, -.45, 1.02), 'right')
        candidates = list(paired_carry_candidates(picks))
        self.assertTrue(any(np.allclose(p['right'].carry_box()[:3, 3], [.38, -.56, 2.]) for p in candidates))
        for candidate in candidates:
            for side, pick in candidate.items():
                self.assertEqual(pick.center_m, picks[side].center_m)
                self.assertEqual(pick.extraction_distance_m, picks[side].extraction_distance_m)
                np.testing.assert_allclose(pick.carry_box()[:3, :3], picks[side].carry_box()[:3, :3])
                np.testing.assert_allclose(pick.tool_to_box(), picks[side].tool_to_box())
        self.assertIs(next(paired_carry_candidates(picks, adapt=False)), picks)
        picks['left'] = BoxPick('46.5x25.5x58.2', (1., .45, .491), 'left', swap_length_height=True)
        np.testing.assert_allclose(next(paired_carry_candidates(picks))['left'].carry_box()[:3, 3], [.43, .5, 1.2])

    def test_each_orientation_proposes_a_real_box_footprint_within_the_held_cylinder(self):
        import json
        from curobo_core.box_pick import payload_fit
        root = Path(__file__).resolve().parents[1]
        fit = json.loads((root/'generated/v3_suction_v322_6bb184b/frozen_collision_model/box_voxel_edge_corner_final.json').read_text())
        for size in BOX_DIMENSIONS:
            for swap in (False, True):
                for z in (.491, 1.02):
                    picks = {side: BoxPick(size, (1., sign*.45, z), side, swap_length_height=swap)
                             for side, sign in [('left', 1), ('right', -1)]}
                    for pick in next(paired_carry_candidates(picks)).values():
                        radius = pick.carry_payload_envelope(payload_fit(fit, pick.dimensions_m))
                        self.assertLess(radius['box_corner_radius_m'], 1.05)
                        self.assertLess(radius['legacy_payload_sphere_radius_m'], 1.05)

    def test_fixed_extraction_witness_is_not_a_finite_search_failure(self):
        from types import SimpleNamespace
        from curobo_core.box_pick import fixed_extraction_blockages
        obstacle = SimpleNamespace(name='fixed_chassis', pose=[.3, 0., .2, 1., 0., 0., 0.], dims=[.7, 1.3, .4])
        picks = {s: BoxPick('40x40x30', (.78, sign*.48, .2), s)
                 for s, sign in [('left', 1), ('right', -1)]}
        blocked = fixed_extraction_blockages(picks, [obstacle])
        self.assertEqual({b['side'] for b in blocked}, {'left', 'right'})
        for b in blocked:
            self.assertGreater(b['overlap_m'], b['pose_envelope_m']+b['world_allowance_m'])
        # Same task with a genuinely free lateral corridor must not be refused.
        clear = {s: BoxPick('40x40x30', (.78, sign*.95, .205), s)
                 for s, sign in [('left', 1), ('right', -1)]}
        self.assertEqual(fixed_extraction_blockages(clear, [obstacle]), [])
        # A tiny nominal overlap inside the accepted pose envelope is NOT proof
        # of infeasibility; leave it to the real planner and original gates.
        touch = {s: BoxPick('40x40x30', (1.10, sign*.48, .2), s)
                 for s, sign in [('left', 1), ('right', -1)]}
        self.assertEqual(fixed_extraction_blockages(touch, [obstacle]), [])

    def test_asymmetric_catalog_keeps_two_sizes_widths_and_explicit_new_inputs(self):
        from experiments.box_layout_20261119.check_asymmetric_layouts import cases
        catalog = cases()
        self.assertEqual(len(catalog), 14)
        self.assertEqual(len({name for name, _ in catalog}), 14)
        modes = set()
        left_sizes = set(); right_sizes = set()
        for name, document in catalog:
            picks = parse_layout(document)
            left_sizes.add(picks['left'].size); right_sizes.add(picks['right'].size)
            modes.add((picks['left'].top, picks['right'].top))
            for pick in picks.values():
                self.assertEqual(pick.extraction_distance_m, .3 if pick.size == '40x40x30' else .255)
            self.assertNotEqual(picks['left'].center_m, picks['right'].center_m)
        self.assertEqual(left_sizes, set(BOX_DIMENSIONS))
        self.assertEqual(right_sizes, set(BOX_DIMENSIONS))
        self.assertEqual(modes, {(False, False), (True, False), (False, True), (True, True)})

    def test_coverage_selection_cannot_silently_omit_unknown_or_duplicate_cases(self):
        from experiments.box_layout_20261119.check_asymmetric_layouts import cases, select_cases
        catalog=cases()
        self.assertEqual(select_cases(catalog,[catalog[0][0]]),catalog[:1])
        with self.assertRaises(ValueError):select_cases(catalog,[catalog[0][0],'missing_case'])
        with self.assertRaises(ValueError):select_cases([catalog[0],catalog[0]])
        with self.assertRaises(ValueError):select_cases([('../escape',catalog[0][1])])
        with self.assertRaises(ValueError):select_cases([])

    def test_reach_refusal_displays_the_actual_geometric_bound(self):
        text=status_markdown(dict(events=[dict(stage='contact_reach_bounds',state='rejected',reason='assigned arm outside reach',
            witnesses=[dict(kind='assigned_arm_joint2_reach_bound',side='right',wrist_to_elbow_distance_lower_m=.558,
                            forearm_length_m=.474,pose_envelope_m=.0101)])]))
        for value in ('右臂','558.00mm','484.10mm','无法接触'):self.assertIn(value,text)

    def test_gpu_estimate_scales_with_the_actual_candidate_budget(self):
        for value, contacts, expected in [('6m',16,'6m'), ('6m',32,'12m'), ('2h',8,'2h'), ('90s',64,'360s'), ('5',16,'5m')]:
            self.assertEqual(scaled_estimate(value, contacts), expected)
        for value in ('0m', '-2h', 'nanm', 'infs', 'nonsense'):
            with self.assertRaises(ValueError):scaled_estimate(value, 16)

    def test_invalid_heterogeneous_dynamics_sizes_fail_before_cuda_initialization(self):
        for dimensions in ({}, {'left': [0, 1, 1]}, {'right': [1, float('nan'), 1]}, {'other': [1, 1, 1]}):
            with self.assertRaises(ValueError):TorqueModel('/not/opened.yaml', payload_dimensions=dimensions)


if __name__ == '__main__':unittest.main()


def test_adaptive_source_proposals_reserve_room_without_changing_legacy_bound():
    import ast
    path=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
    cls=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='BoxLayoutPlanner')
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='forward_lateral_prefix')
    limits=next(n for n in method.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='limits' for t in n.targets))
    for adaptive,expected in ((True,.0060005),(False,.006001)):
        env=dict(adaptive=adaptive,initial={'left':dict(initial_slot_overlap_m=.006,observation_epsilon_m=1e-6)})
        exec(compile(ast.Module(body=[limits],type_ignores=[]),str(path),'exec'),env)
        assert abs(env['limits']['left']-expected)<1e-15


def test_independent_arm_mask_keeps_original_active_arm_and_fixed_spheres_only():
    import ast
    from types import SimpleNamespace as NS
    import torch
    path=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
    node=next(n for n in ast.walk(ast.parse(path.read_text())) if isinstance(n,ast.FunctionDef) and n.name=='independent_arm_mask')
    original=torch.tensor([[[[0.,0.,0.,.1],[.05,0.,0.,.1],[0.,0.,0.,5.],[.05,0.,0.,.1],[0.,0.,0.,5.]],
                             [[0.,0.,0.,.1],[10.,0.,0.,.1],[0.,0.,0.,5.],[.05,0.,0.,.1],[0.,0.,0.,5.]]]])
    ids={'head':torch.tensor([0]),'left_link7':torch.tensor([1,2]),'right_link7':torch.tensor([3,4])}
    original[:,:,2,:]=torch.tensor([100.,0.,0.,.1]);original[:,:,4,:]=torch.tensor([-100.,0.,0.,.1])
    planner=NS(robot={'kinematics':{'collision_spheres':{link:[{}] for link in ids}}},native_margin_mask=lambda *args,**kw:torch.tensor([True,True]))
    rows=np.arange(30,dtype=float).reshape(2,15)/1234567.;captured=[]
    def fk(q):captured.append(q.clone());return NS(robot_spheres=original)
    proxy=NS(as_tensor=lambda value,device,dtype:torch.as_tensor(value,dtype=dtype),zeros=torch.zeros,bool=torch.bool,float32=torch.float32)
    env=dict(np=np,torch=proxy,JointState=NS(from_position=lambda value,**kw:value),ACTIVE_JOINTS=list(range(15)))
    exec(compile(ast.Module(body=[node],type_ignores=[]),str(path),'exec'),env)
    for amount in (0.,2.):
        padding=torch.full((5,),amount)
        def collision(spheres):
            radii=spheres[...,3]+padding
            costs=[]
            for i in range(5):
                for j in range(i):
                    overlap=(radii[...,i]+radii[...,j]-(spheres[...,i,:3]-spheres[...,j,:3]).norm(dim=-1)).clamp(min=0)
                    costs.append(overlap*((radii[...,i]>=0)&(radii[...,j]>=0)))
            return torch.stack(costs).sum(0)[...,None]
        checker=NS(kinematics=NS(compute_kinematics=fk,config=NS(kinematics_config=NS(get_sphere_index_from_link_name=ids.__getitem__)),
            get_self_collision_config=lambda:NS(sphere_padding=padding)),get_self_collision=collision)
        snapshot=original.clone()
        np.testing.assert_array_equal(env['independent_arm_mask'](planner,checker,rows,'left'),[False,True])
        np.testing.assert_array_equal(env['independent_arm_mask'](planner,checker,rows,'right'),[False,False])
        torch.testing.assert_close(original,snapshot,rtol=0,atol=0)
        np.testing.assert_array_equal(captured[-1].numpy()[0],np.round(rows,7).astype(np.float32))


def test_original_sphere_links_kept_by_arm_precheck_have_no_partner_joint_ancestor():
    import xml.etree.ElementTree as ET
    import yaml
    root=Path(__file__).resolve().parents[1]
    config=yaml.safe_load((root/'generated/fast_runtime_assets/fixed.yaml').read_text())['kinematics']
    urdf=ET.parse(config['urdf_path']).getroot()
    parents={joint.find('child').get('link'):joint for joint in urdf.findall('joint')}
    for side,other in (('left','right'),('right','left')):
        for link in config['collision_spheres']:
            if link.startswith(('left_','right_')) and not link.startswith(side+'_'):continue
            current=link
            while current in parents:
                joint=parents[current]
                assert joint.get('type')=='fixed' or not joint.get('name').startswith(other+'_')
                current=joint.find('parent').get('link')


def test_independent_arm_precheck_is_restricted_to_noncontact_fine_enumeration():
    import ast
    from types import SimpleNamespace as NS
    path=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
    method=next(n for n in ast.walk(ast.parse(path.read_text())) if isinstance(n,ast.FunctionDef) and n.name=='enumerate_pair')
    node=next(n for n in ast.walk(method) if isinstance(n,ast.If) and ast.unparse(n.test)=='not contact and arm_limit is None')
    loop=ast.For(target=ast.Name(id='_',ctx=ast.Store()),iter=ast.List(elts=[ast.Constant(value=0)],ctx=ast.Load()),body=[node],orelse=[])
    code=compile(ast.fix_missing_locations(ast.Module(body=[loop],type_ignores=[])),str(path),'exec')
    for contact,limit in ((False,None),(True,None),(False,32),(True,32)):
        calls=[];values=np.arange(21,dtype=float).reshape(3,7)
        def precheck(checker,probes,side):calls.append(probes.copy());return np.array([True,False,True])
        env=dict(np=np,contact=contact,arm_limit=limit,row=np.zeros(15),values=values.copy(),offset=1,side='left',checker=object(),
            self=NS(independent_arm_mask=precheck),arm_checked=dict(left=0,right=0),arm_rejected=dict(left=0,right=0))
        exec(code,env)
        active=not contact and limit is None
        assert len(calls)==int(active)
        np.testing.assert_array_equal(env['values'],values[[0,2]] if active else values)
        assert env['arm_checked']['left']==(3 if active else 0)
        assert env['arm_rejected']['left']==int(active)


def test_endpoint_native_base_guidance_uses_existing_pose_bounds_and_frozen_activation():
    from curobo_core.box_pick import extraction_endpoint_proposals,BoxPick
    picks={'left':BoxPick('40x40x30',(.95,.6,.65),'left')}
    sphere=np.array([[-.1365,-.1865,-.1865,.0135]])
    base=dict(centers=[[0.,0.,.16]],rotations=np.eye(3)[None],halves=[[.5,.5,.3]],spheres={'left':sphere},activation_m=.005)
    proposals=extraction_endpoint_proposals(picks,np.array([[10.,10.,10.,.05]]),[0.,0.,1.],(-1.,0.),[-.2],native_base_geometry=base)
    assert proposals
    T=np.asarray(proposals[0]['boxes']['left']);point=sphere[:,:3]@T[:3,:3].T+T[:3,3]
    delta=np.abs(point-np.array([0.,0.,.16]))-np.array([.5,.5,.3])
    gap=np.linalg.norm(np.maximum(delta,0),axis=-1)+np.minimum(delta.max(-1),0)-sphere[:,3]
    assert gap.min()>=.005
    assert np.rad2deg(Rotation.from_matrix(T[:3,:3]).magnitude())<=.950001
    assert np.linalg.norm(T[:3,3]-[.65,.6,.65])<=.00180001
    invalid=dict(base,halves=[[0.,.5,.3]])
    try:extraction_endpoint_proposals(picks,np.array([[10.,10.,10.,.05]]),[0.,0.,1.],(-1.,0.),[-.2],native_base_geometry=invalid)
    except ValueError:pass
    else:raise AssertionError('invalid native base geometry accepted')


def test_arm_precheck_uses_own_payload_for_original_native_margin_before_pairing():
    import ast
    from types import SimpleNamespace as NS
    import torch
    path=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
    fn=next(n for n in ast.walk(ast.parse(path.read_text())) if isinstance(n,ast.FunctionDef) and n.name=='independent_arm_mask')
    # Slots are head, left original, left payload, right original, right payload.
    original=torch.tensor([[[[0.,0.,1.,.1],[1.,0.,1.,.1],[2.,0.,1.,.1],[3.,0.,1.,.1],[4.,0.,1.,.1]]]])
    ids={'head':torch.tensor([0]),'left_link7':torch.tensor([1,2]),'right_link7':torch.tensor([3,4])}
    seen=[]
    def native(checker,values,*,state):seen.append(state.robot_spheres.clone());return torch.tensor([False])
    planner=NS(robot={'kinematics':{'collision_spheres':{name:[{}] for name in ids}}},native_margin_mask=native)
    checker=NS(kinematics=NS(compute_kinematics=lambda _:NS(robot_spheres=original),
        config=NS(kinematics_config=NS(get_sphere_index_from_link_name=ids.__getitem__)),
        get_self_collision_config=lambda:NS(sphere_padding=torch.zeros(5))),get_self_collision=lambda _:torch.zeros((1,1,1)))
    proxy=NS(as_tensor=lambda values,device,dtype:torch.as_tensor(values,dtype=dtype),zeros=torch.zeros,bool=torch.bool,float32=torch.float32)
    env=dict(torch=proxy,np=np,JointState=NS(from_position=lambda values,**kw:values),ACTIVE_JOINTS=list(range(15)))
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),env)
    assert not env['independent_arm_mask'](planner,checker,np.zeros((1,15)),'left')[0]
    assert (seen[0][...,0:3,3]>0).all() and (seen[0][...,3:,3]<0).all()
    torch.testing.assert_close(original[...,3],torch.full((1,1,5),.1))


def test_transport_search_exhaustion_allows_existing_reverse_recovery_with_same_budgets():
    import ast
    import itertools
    from types import SimpleNamespace
    path = Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
    cls = next(n for n in ast.parse(path.read_text()).body if isinstance(n, ast.ClassDef) and n.name == 'BoxLayoutPlanner')
    plan = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_plan_layout')
    trials = next(n for n in plan.body if isinstance(n, ast.FunctionDef) and n.name == 'trials')
    for failure, spent, expect_recovery in (
        ('joint transport RRT exhausted its 8s search budget; not proof the layout is illegal', 48.58, True),
        ('full-size boxes collide during extraction (including box vs box)', 48.58, True),
        ('native transport: collision', 48.58, False),
        ('joint transport RRT exhausted its 8s search budget; not proof the layout is illegal', 128., False),
    ):
        report = dict(attempts=[dict(failure=failure)]*4, extraction_recovery=[],
                      transport_budget=dict(spent_s=spent,total_s=128.))
        recovery = dict(frames=np.full((1,15),.123))
        calls = []
        def reverse(*args):
            calls.append(True)
            return iter([recovery]*12)
        env = dict(np=np,itertools=itertools,contacts=np.zeros((256,15)),max_contacts=16,
                   report=report,self=SimpleNamespace(reverse_extractions=reverse),
                   empty=None,loaded=None,lm=None,progress=lambda e:None)
        exec(compile(ast.Module(body=[trials],type_ignores=[]),str(path),'exec'),env)
        rows = []
        for row in itertools.islice(env['trials'](),16):
            rows.append(row)
            if len(rows)>4:report['attempts'].append(dict(failure=failure))
        assert len(rows)==16
        assert all(v is None for _,v in rows[:4])
        assert sum(v is not None for _,v in rows)==(8 if expect_recovery else 0)
        assert report['transport_budget']==dict(spent_s=spent,total_s=128.)


def test_handover_failure_reports_the_actual_recovery_error(tmp_path):
    import ast
    import json
    from types import SimpleNamespace
    from unittest.mock import patch
    import v3_handover_layout
    source=Path(__file__).resolve().parents[1]/'tools/v3_box_layout_planner.py'
    cls=next(node for node in ast.parse(source.read_text()).body if isinstance(node,ast.ClassDef) and node.name=='BoxLayoutPlanner')
    method=next(node for node in cls.body if isinstance(node,ast.FunctionDef) and node.name=='plan_layout')
    from dataclasses import replace
    namespace=dict(validate_pair=lambda picks:None,np=np,Path=Path,json=json,asdict=asdict,replace=replace)
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
    picks={side:BoxPick('40x40x30',(1.168052051,.5 if side=='left' else -.5,1.),side) for side in ('left','right')}
    planner=SimpleNamespace(_plan_layout=lambda *args,**kwargs:dict(success=False,failure='initial assigned-arm reach witness'))
    with patch.object(v3_handover_layout,'plan_handover_pick',return_value=dict(success=False,failure='donor_release exceeds actual joint7 upper limit')):
        result=namespace['plan_layout'](planner,picks,tmp_path)
    assert result['failure_kind']=='handover_recovery_failed'
    assert result['failure_before_handover']=='initial assigned-arm reach witness'
    assert 'joint7 upper limit' in result['failure']
    assert not result['success'] and not result['handover_recovery']['success']
    assert len(result['grasp_trials'])==7 and result['grasp_trials'][-1]['mode']=='handover_same_side'
