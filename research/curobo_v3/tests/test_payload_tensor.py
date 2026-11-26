"""CPU parity of the float64 tensor SAT; no CUDA initialization."""
from pathlib import Path
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
import torch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'tools'))
from curobo_core.payload_geometry import box_overlap_depth, box_overlap_depth_tensor, box_overlap_depth_projected_tensor


class PayloadTensorTest(unittest.TestCase):
    def test_search_classification_reuses_only_this_query_state(self):
        import ast
        from types import SimpleNamespace as NS
        from unittest.mock import patch
        file=Path(__file__).resolve().parents[1]/'tools/v3_payload_safety.py'
        node=next(n for n in ast.parse(file.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='payload_search_mask')
        env=dict(np=np,torch=torch)
        exec(compile(ast.Module(body=[node],type_ignores=[]),str(file),'exec'),env)
        seen=[]
        class Layer:
            def __init__(self,*args,**kwargs):pass
            def classify(self,value):
                seen.append(value.cuda_robot_model_state)
                return torch.tensor([[True,False,True,True]]),torch.tensor([[False,False,True,False]])
        def no_fk(*args):raise AssertionError('recomputed FK despite fresh state')
        checker=NS(task_tool_to_box={'left':np.eye(4)},guidance_scene=NS(cuboid=[]),
            base_cuboid_layer=NS(scene=NS(cuboid=[])),kinematics=NS(compute_kinematics=no_fk))
        rows=torch.zeros((4,15));state=NS(robot_spheres=torch.zeros((1,4,2,4)),tool_poses=object())
        def audit(values):
            self.assertEqual(len(values),1)
            state.robot_spheres.fill_(999.) # Mirrors the real fallback's mutable FK overwrite.
            return dict(invalid_frame_indices=[0])
        with patch.dict(sys.modules,{'v3_full_payload_cost':NS(FullPayloadCost=Layer)}):
            result=env['payload_search_mask'](checker,rows,{'left':(.3,.4,.4)},{},audit,state=state)
            torch.testing.assert_close(result,torch.tensor([True,False,False,True]))
            second=NS(robot_spheres=torch.zeros((1,4,2,4)),tool_poses=object())
            env['payload_search_mask'](checker,rows,{'left':(.3,.4,.4)},{},audit,state=second)
            self.assertIs(seen[0],state);self.assertIs(seen[1],second)
            self.assertEqual(checker.payload_search_statistics,dict(frames=8,cpu_fallback_frames=2))
            with self.assertRaises(ValueError):
                env['payload_search_mask'](checker,rows[:3],{'left':(.3,.4,.4)},{},audit,state=second)

    def test_projected_search_sat_keeps_axes_and_original_ambiguity_decisions(self):
        rng=np.random.default_rng(82)
        def tensor(values):return torch.as_tensor(values,dtype=torch.float64)
        # Box-world and paired-box broadcasting; all15 explicit axes remain the reference.
        for count,obstacles in ((33,3),(257,7)):
            p=rng.normal(size=(count,3));q=rng.normal(size=(obstacles,3))
            a=Rotation.random(count,random_state=rng).as_matrix();b=Rotation.random(obstacles,random_state=rng).as_matrix()
            args=[tensor(v) for v in (p[None],a[None],[.1275,.2325,.291],q,b,rng.uniform(.05,.5,(obstacles,3)))]
            old=box_overlap_depth_tensor(*args);new=box_overlap_depth_projected_tensor(*args)
            torch.testing.assert_close(new[0],old[0],rtol=0,atol=2e-12)
            torch.testing.assert_close(new[2],old[2],rtol=0,atol=2e-12)
            torch.testing.assert_close(new[1],old[1],rtol=0,atol=0)
        count=257;ha=np.array([.1275,.2325,.291]);hb=np.array([.15,.2,.2])
        for angle in (0.,1e-12,1e-10,1e-8,1e-6,1e-4,.1):
            a=Rotation.random(count,random_state=rng).as_matrix()
            axis=rng.normal(size=(count,3));axis/=np.linalg.norm(axis,axis=1)[:,None]
            b=a@Rotation.from_rotvec(axis*angle).as_matrix();p=rng.normal(size=(count,3))
            for threshold in (1e-6,.0010001):
                offset=rng.choice([0.,-1e-7,1e-7],size=count)+rng.uniform(-2e-9,2e-9,count)
                q=p+a[:,:,0]*(ha[0]+hb[0]-threshold+offset)[:,None]
                args=[tensor(v) for v in (p[None],a[None],ha,q[None,:,None,:],b[None,:,None,:,:],hb[None])]
                old,uo,fo=box_overlap_depth_tensor(*args);new,un,fn=box_overlap_depth_projected_tensor(*args)
                ambiguous_old=(uo&(fo>=threshold-1e-7))|((old-threshold).abs()<=1e-7)
                ambiguous_new=(un&(fn>=threshold-1e-7))|((new-threshold).abs()<=1e-7)
                torch.testing.assert_close(ambiguous_new,ambiguous_old,rtol=0,atol=0)
                changed=(old>threshold)!=(new>threshold)
                self.assertFalse(bool((changed&~ambiguous_old).any()))
                cpu=box_overlap_depth(p,a,ha,q,b,hb)>threshold
                np.testing.assert_array_equal(np.where(ambiguous_old.numpy().reshape(-1),cpu,(old>threshold).numpy().reshape(-1)),
                    np.where(ambiguous_new.numpy().reshape(-1),cpu,(new>threshold).numpy().reshape(-1)))
        # Exact-threshold signed depths may round differently; BOTH must retain original fallback.
        p=np.zeros((33,3));p[:,0]=.3-.0010001+np.linspace(-1e-9,1e-9,33)
        a=Rotation.from_rotvec([[1e-10,0.,0.]]*33).as_matrix()
        args=[tensor(v) for v in (p[None],a[None],[.15,.2,.2],[[0.,0.,0.]],np.eye(3)[None],[[.15,.2,.2]])]
        old,uo,fo=box_overlap_depth_tensor(*args);new,un,fn=box_overlap_depth_projected_tensor(*args)
        self.assertTrue(bool(uo.all()) and bool(un.all()))
        changed=(old>.0010001)!=(new>.0010001)
        self.assertFalse(bool((changed&~(uo&un)).any()))

    def test_signed_depth_random_parallel_and_tolerance_boundaries(self):
        rng = np.random.default_rng(82)
        count, obstacles = 257, 7
        p = rng.normal(size=(count, 3)); a = Rotation.random(count, random_state=rng).as_matrix()
        q = rng.normal(size=(obstacles, 3)); b = Rotation.random(obstacles, random_state=rng).as_matrix()
        ha = np.array([.1275, .2325, .291]); hb = rng.uniform(.05, .5, (obstacles, 3))
        def tensor(values):return torch.tensor(values, dtype=torch.float64)
        depth, unsure, _ = box_overlap_depth_tensor(tensor(p[None]), tensor(a[None]), tensor(ha), tensor(q), tensor(b), tensor(hb))
        expected = np.stack([box_overlap_depth(p, a, ha, q[i], b[i], hb[i]) for i in range(obstacles)], -1)
        np.testing.assert_allclose(depth.numpy()[0], expected, atol=1e-12, rtol=1e-12)
        self.assertFalse(bool(unsure.any()))
        for angle in (0., 1e-12, 1e-10, 1e-8, 1e-6):
            a = Rotation.from_rotvec([[angle, 0., 0.]]*9).as_matrix()
            p = np.zeros((9, 3))
            for threshold in (1e-6, .0010001):
                overlap = threshold+np.linspace(-1e-9, 1e-9, 9)
                p[:, 0] = ha[0]+hb[0, 0]-overlap
                depth, unsure, _ = box_overlap_depth_tensor(tensor(p[None]), tensor(a[None]), tensor(ha), tensor(np.zeros((1, 3))), tensor(np.eye(3)[None]), tensor(hb[:1]))
                expected = box_overlap_depth(p, a, ha, np.zeros(3), np.eye(3), hb[0])
                np.testing.assert_allclose(depth.numpy()[0, :, 0], expected, atol=1e-12, rtol=0)
                self.assertTrue(bool(unsure.all()), 'parallel/tiny axes must ask the CPU reference, not guess')


if __name__ == '__main__':unittest.main()


def test_query_local_preclassification_keeps_cpu_fallback_and_rejects_bad_batch():
    import ast
    from types import SimpleNamespace as NS
    from unittest.mock import patch
    path=Path(__file__).resolve().parents[1]/'tools/v3_payload_safety.py'
    fn=next(n for n in ast.parse(path.read_text()).body if isinstance(n,ast.FunctionDef) and n.name=='payload_search_mask')
    env=dict(np=np,torch=torch)
    exec(compile(ast.Module(body=[fn],type_ignores=[]),str(path),'exec'),env)
    class Layer:
        def __init__(self,*args,**kwargs):pass
        def classify(self,state):raise AssertionError('recomputed preclassified query')
    checker=NS(task_tool_to_box={'left':np.eye(4)},guidance_scene=NS(cuboid=[]),base_cuboid_layer=NS(scene=NS(cuboid=[])))
    rows=torch.zeros((4,15));state=NS(robot_spheres=torch.zeros((1,4,2,4)),tool_poses=object())
    classification=(torch.tensor([[True,False,True,True]]),torch.tensor([[False,False,True,False]]))
    seen=[]
    def audit(values):
        seen.append(len(values));state.robot_spheres.fill_(999.)
        return dict(invalid_frame_indices=[0])
    with patch.dict(sys.modules,{'v3_full_payload_cost':NS(FullPayloadCost=Layer)}):
        result=env['payload_search_mask'](checker,rows,{'left':(.3,.4,.4)},None,audit,state=state,classification=classification)
        torch.testing.assert_close(result,torch.tensor([True,False,False,True]))
        assert seen==[1] and checker.payload_search_statistics==dict(frames=4,cpu_fallback_frames=1)
        for supplied_state,values in ((None,classification),(state,(torch.ones((1,3),dtype=torch.bool),classification[1]))):
            try:env['payload_search_mask'](checker,rows,{'left':(.3,.4,.4)},None,audit,state=supplied_state,classification=values)
            except ValueError:pass
            else:raise AssertionError('accepted stale or mismatched batch')
