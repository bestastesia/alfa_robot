"""Gradient regression, including a real CUDA query; always invoke with gpuq."""
import os
from pathlib import Path
import sys
import unittest


@unittest.skipUnless(os.environ.get('RUN_GPU_TESTS')=='1','run through gpuq')
class CachedMeshGradientTest(unittest.TestCase):
    def test_upstream_weights_are_propagated_for_both_native_flags(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
        from types import SimpleNamespace
        import torch
        from v3_base_mesh_layer import CachedMeshQuery
        for flag in (False,True):
            with self.subTest(use_grad_input=flag):
                class Operation:
                    config=SimpleNamespace(use_grad_input=flag)
                    _collision_buffer=SimpleNamespace()
                    def forward(self,state):
                        self._collision_buffer.gradient=2*state.robot_spheres
                        return state.robot_spheres.square().sum(-1)
                x=torch.tensor([[[[1.,2.,3.,4.],[2.,3.,4.,5.]]]],dtype=torch.float64,requires_grad=True)
                weights=torch.tensor([[[7.,3.]]],dtype=x.dtype)
                loss=(CachedMeshQuery.apply(x,SimpleNamespace(robot_spheres=x),Operation())*weights).sum()
                actual=torch.autograd.grad(loss,x)[0]
                torch.testing.assert_close(actual,2*x*weights[...,None])

    def test_real_scaled_base_query_matches_finite_difference(self):
        sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
        import copy
        import torch
        from curobo.types import JointState
        from curobo_core.box_pick import BoxPick
        from v3_two_box_pick import TwoBoxPickPlanner
        from v3_wall_ik_benchmark import ACTIVE_JOINTS
        planner=TwoBoxPickPlanner();planner.configure_pick(BoxPick('40x40x30',(1.18,.45,1.02),'left'))
        checker=planner.checker(planner.selected_task);layer=checker.base_cuboid_layer
        state=checker.kinematics.compute_kinematics(JointState.from_position(
            torch.tensor(planner.home_values[None,None],device='cuda'),joint_names=ACTIVE_JOINTS))
        spheres=torch.zeros_like(state.robot_spheres);spheres[...,3]=-1
        index=int(layer.indices[0]);box=layer.scene.cuboid[0]
        spheres[0,0,index]=torch.tensor([box.pose[0]+box.dims[0]/2+.052,box.pose[1],box.pose[2],.05],device='cuda')
        spheres.requires_grad_()
        def value(query):
            template=copy.copy(state);template.robot_spheres=query
            return 7*layer.cost(template).sum()
        loss=value(spheres);actual=float(torch.autograd.grad(loss,spheres)[0][0,0,index,0])
        plus=spheres.detach().clone();minus=plus.clone();epsilon=1e-4
        plus[0,0,index,0]+=epsilon;minus[0,0,index,0]-=epsilon
        expected=float((value(plus)-value(minus))/(2*epsilon))
        self.assertGreater(float(loss.detach()),0.)
        self.assertLess(expected,0.)
        self.assertAlmostEqual(actual,expected,delta=max(1e-3,abs(expected)*.02))


if __name__=='__main__':unittest.main()
