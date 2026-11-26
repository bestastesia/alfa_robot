"""CPU-only shared-FK contract; the actual LM A/B runs separately through gpuq."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

import torch

TOOLS = Path(__file__).resolve().parents[1]/'tools'


def validity_class():
    tree = ast.parse((TOOLS/'v3_batched_loaded_search.py').read_text())
    nodes = [n for n in tree.body if isinstance(n, (ast.FunctionDef, ast.ClassDef))
             and n.name in ('GpuValidity', 'scene_query_spheres')]
    namespace = dict(torch=torch, math=math, JointState=NS(from_position=lambda q, joint_names: q))
    exec(compile(ast.Module(body=nodes, type_ignores=[]), str(TOOLS/'v3_batched_loaded_search.py'), 'exec'), namespace)
    return namespace['GpuValidity']


class LmFkReuseTest(unittest.TestCase):
    def test_lm_candidate_filter_gets_exact_fresh_state_and_cannot_restore_invalid_rows(self):
        import numpy as np
        file=TOOLS/'v3_lm_conveyor.py'
        cls=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='SequentialPlacementLm')
        plan=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='plan')
        inspect=next(n for n in plan.body if isinstance(n,ast.FunctionDef) and n.name=='inspect')
        state=object();actual=object();calls=[]
        def fk(value):calls.append(value);return state
        checker=NS(setup_batch_tensors=lambda *args:None,kinematics=NS(compute_kinematics=fk))
        case={'attachments':{}}
        proxy=NS(float32=torch.float32,as_tensor=lambda x,device,dtype:torch.as_tensor(x,dtype=dtype))
        env=dict(np=np,torch=proxy,case=case,checker=checker,ACTIVE_JOINTS=list(range(15)),
            JointState=NS(from_position=lambda value,**kw:value),box_poses=lambda *args,**kw:actual,
            validity=NS(evaluate_state=lambda rows,value:(torch.tensor([True,False,True]),None)))
        exec(compile(ast.Module(body=[inspect],type_ignores=[]),str(file),'exec'),env)
        values=np.zeros((3,15),dtype=np.float32)
        self.assertEqual(env['inspect'](values)[1].tolist(),[True,False,True])
        def candidate_filter(rows,value,boxes):
            self.assertIs(value,state);self.assertIs(boxes,actual)
            self.assertEqual(rows.data_ptr(),calls[-1].data_ptr())
            return [False,True,True]
        case['candidate_filter']=candidate_filter
        self.assertEqual(env['inspect'](values)[1].tolist(),[False,False,True])
        self.assertEqual(len(calls),2)
        case['candidate_filter']=lambda *args:np.zeros((3,1),dtype=bool)
        with self.assertRaises(ValueError):env['inspect'](values)

    def test_native_margin_reuses_query_state_without_changing_zero_set(self):
        file=TOOLS/'v3_two_box_pick.py'
        cls=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.ClassDef) and n.name=='TwoBoxPickPlanner')
        node=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='native_margin_mask')
        env=dict(torch=torch);exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec'),env)
        spheres=torch.tensor([[[[0.,0.,1.,.1]],[[0.,0.,.101,.1]],[[0.,0.,1.,.1]]]])
        state=NS(robot_spheres=spheres)
        checker=NS(base_cuboid_layer=NS(cost=lambda value:torch.tensor([[[0.],[0.],[1.]]])))
        rows=torch.zeros((3,15));mask=env['native_margin_mask'](None,checker,rows,state=state)
        torch.testing.assert_close(mask,torch.tensor([True,False,False]))
        with self.assertRaises(ValueError):env['native_margin_mask'](None,checker,rows[:2],state=state)

    def test_shared_fk_preserves_mask_and_counts_without_recomputing(self):
        cls = validity_class()
        calls = []
        def fk(q):
            calls.append(q.shape[1])
            spheres = torch.zeros((1, q.shape[1], 2, 4))
            spheres[..., 3] = .1
            spheres[..., 2] = q[..., 0, None]
            return NS(robot_spheres=spheres)
        collision = NS(update_num_spheres=lambda *args, **kwargs: None,
                       forward=lambda state: torch.zeros_like(state.robot_spheres[..., 0]))
        checker = NS(setup_batch_tensors=lambda *args: None, collision_constraint=collision,
                     kinematics=NS(compute_kinematics=fk),
                     get_self_collision=lambda spheres: torch.zeros_like(spheres[..., 0]),
                     get_bound=lambda q: torch.zeros_like(q))
        value = cls.__new__(cls)
        value.checker = checker; value.states_checked = 0
        value.joint_names = ['joint']; value.active_sides = ()
        value.minimum_box_up_z = .8; value.scene_penetration_tolerance_m = 0.
        value.check_ground = True; value.ground_exempt_sphere_indices = set(); value.ground_z = 0.
        q = torch.tensor([[-1.], [0.], [.1], [1.], [2.]])
        expected = value.evaluate(q)
        self.assertEqual(calls, [5])
        self.assertEqual(expected[0].tolist(), [False, False, True, True, True])
        state = fk(q[None]); count = len(calls)
        actual = value.evaluate_state(q, state)
        self.assertEqual(len(calls), count)
        torch.testing.assert_close(actual[0], expected[0], rtol=0, atol=0)
        torch.testing.assert_close(actual[1], expected[1], rtol=0, atol=0)
        self.assertEqual(value.states_checked, 10)
        for wrong in (q[:1], q.reshape(1, 5, 1), torch.empty((0, 1)), torch.zeros((4097, 1))):
            with self.assertRaises(ValueError):value.evaluate_state(wrong, state)
        calls.clear()
        value.evaluate(torch.ones((5000, 1)))
        self.assertEqual(calls, [4096, 904])


if __name__ == '__main__':unittest.main()


def test_explicit_reference_grid_and_fine_edges_keep_default_sampling():
    import math
    import numpy as np
    file=TOOLS/'v3_lm_conveyor.py';tree=ast.parse(file.read_text())
    cls=next(n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='SequentialPlacementLm')
    plan=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name=='plan')
    index=next(i for i,n in enumerate(plan.body) if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='fractions' for t in n.targets))
    validate=compile(ast.Module(body=plan.body[index:index+2],type_ignores=[]),str(file),'exec')
    for grid in (None,[0.,.1,.9,1.]):
        env=dict(np=np,self=NS(samples=4),case={} if grid is None else dict(reference_fractions=grid))
        exec(validate,env);np.testing.assert_array_equal(env['fractions'],np.linspace(0,1,4) if grid is None else grid)
    import pytest
    for grid in ([],[[0.,.1,.9,1.]],[0.,.5,.5,1.],[0.,1.,.5,1.],[.1,.4,.8,1.],[0.,.1,.9,.99],[0.,.1,float('nan'),1.]):
        with pytest.raises(ValueError):exec(validate,dict(np=np,self=NS(samples=4),case=dict(reference_fractions=grid)))
    nodes=[next(n for n in ast.parse((TOOLS/'v3_batched_loaded_search.py').read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='densify'),
        next(n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='dense_segment')]
    weights=np.r_[np.deg2rad(15)/.1,np.ones(14)]
    env=dict(np=np,math=math,WEIGHTS=weights)
    exec(compile(ast.Module(body=nodes,type_ignores=[]),str(file),'exec'),env)
    first=np.zeros(15);last=np.r_[.01,np.linspace(0,.13,14)]
    old_count=max(1,int(np.ceil(np.max(abs((last-first)*weights))/np.deg2rad(.25))))
    coarse,_=env['dense_segment'](first,last,0.,1.)
    np.testing.assert_array_equal(coarse,np.linspace(first,last,old_count+1).astype(np.float32))
    fine,progress=env['dense_segment'](first,last,0.,1.,maximum_weighted_step=.001)
    weights[0]=max(5.,weights[0])
    assert np.max(abs(np.diff(fine,axis=0)*weights))<=.00100001 and len(fine)>=len(coarse)
    assert progress[0]==0. and progress[-1]==1.
    for step in (0.,-.001,float('nan'),.1):
        with pytest.raises(ValueError):env['dense_segment'](first,last,0.,1.,maximum_weighted_step=step)
