"""CPU geometry contract for full-cuboid guidance; real CUDA/gradient probe is separate."""
import ast
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
import torch

TOOLS = Path(__file__).resolve().parents[1]/'tools'
sys.path.insert(0, str(TOOLS))
from curobo_core.payload_geometry import box_overlap_depth, box_sphere_depth, box_overlap_depth_tensor, box_overlap_depth_projected_tensor


def load_layer():
    def quaternion_matrix(quat):
        data = quat.detach().numpy()
        matrices = Rotation.from_quat(np.roll(data.reshape(-1, 4), -1, axis=1)).as_matrix()
        return torch.as_tensor(matrices.reshape(*data.shape[:-1], 3, 3), dtype=quat.dtype)
    env = dict(np=np, torch=torch, Rotation=Rotation, torch_quaternion_to_matrix=quaternion_matrix,
               box_overlap_depth_projected_tensor=box_overlap_depth_projected_tensor)
    for path, name in [(TOOLS/'v3_fixed_body_boxes.py', 'obb_penetration'), (TOOLS/'v3_full_payload_cost.py', 'FullPayloadCost')]:
        node = next(n for n in ast.parse(path.read_text()).body if isinstance(n, (ast.FunctionDef, ast.ClassDef)) and n.name == name)
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(path), 'exec'), env)
    return env['FullPayloadCost']


class FullPayloadCostTest(unittest.TestCase):
    def test_small_query_graph_uses_fresh_private_inputs_and_preserves_outputs(self):
        from unittest.mock import patch
        cls=load_layer();layer=cls.__new__(cls);layer.attachments={'left':None}
        calls=[]
        class Executor:
            def __init__(self,fn,device,**kw):
                self.fn=fn;self.inputs=None;calls.append(kw)
            def __call__(self,*values):
                if self.inputs is None:self.inputs=[v.clone() for v in values]
                else:
                    for target,value in zip(self.inputs,values):target.copy_(value)
                return tuple(v.clone() for v in self.fn(*self.inputs))
        def eager(state):
            spheres=state.cuda_robot_model_state.robot_spheres
            pose=state.tool_poses.to_dict()['left_tool0']
            return spheres[:,:,0,0]>pose.position[:,:,0],spheres[:,:,0,1]==0
        layer._classify_eager=eager
        def state(values):
            spheres=torch.zeros((1,len(values),1,4));spheres[0,:,0,0]=torch.tensor(values)
            tools={'left_tool0':NS(position=torch.zeros((1,len(values),3)),
                quaternion=torch.tensor([1.,0.,0.,0.]).expand(1,len(values),4))}
            return NS(cuda_robot_model_state=NS(robot_spheres=spheres),tool_poses=NS(to_dict=lambda:tools))
        with patch.dict(sys.modules,{'curobo._src.util.cuda_graph_util':NS(GraphExecutor=Executor)}):
            first=state([1.,-1.]);valid,uncertain=layer._classify_small_batch(first)
            torch.testing.assert_close(valid,torch.tensor([[True,False]]))
            first.cuda_robot_model_state.robot_spheres.fill_(999.)
            second=state([1.]*32);second.tool_poses.to_dict()['left_tool0'].position.fill_(2.)
            updated,_=layer._classify_small_batch(second)
            self.assertFalse(bool(updated.any()))
            third=state([1.,1.]);third.tool_poses.to_dict()['left_tool0'].position.fill_(-2.)
            restored,_=layer._classify_small_batch(third);self.assertTrue(bool(restored.all()))
            torch.testing.assert_close(valid,torch.tensor([[True,False]]))
            self.assertTrue(bool(uncertain.all()))
        self.assertEqual(len(calls),1)
        self.assertEqual(calls[0],dict(use_cuda_graph=True,clone_inputs=True,clone_outputs=True))

    def test_virtual_clearance_does_not_compete_at_real_penetration_weight(self):
        depth = torch.tensor([-.02, -.01, -.001, .001, .01], dtype=torch.float64, requires_grad=True)
        cls = load_layer()
        layer = cls.__new__(cls)
        layer.overlaps = lambda state, margin: {'test': (depth+margin).clamp(min=0).reshape(1, -1, 1)}
        cost = layer.cost(None)
        gradient = torch.autograd.grad(cost.sum(), depth, retain_graph=True)[0]
        old = (depth+.015).clamp(min=0).square()
        old_gradient = torch.autograd.grad(old.sum(), depth)[0]
        torch.testing.assert_close(gradient[1:3], old_gradient[1:3]*.001, rtol=1e-12, atol=1e-12)
        # Real overlap retains its separate quadratic penalty, rather than
        # receiving only the weak virtual-clearance coefficient.
        torch.testing.assert_close(gradient[3:], 2*depth.detach()[3:]+old_gradient[3:]*.001, rtol=1e-12, atol=1e-12)
        self.assertTrue(bool((cost[0, 3:] >= depth.detach()[3:].square()).all()))
        self.assertEqual(float(cost.detach()[0, 0]), 0.)
        self.assertTrue(bool((cost >= 0).all()))

    def test_source_slot_virtual_clearance_has_its_own_early_penalty(self):
        cls = load_layer()
        layer = cls.__new__(cls)
        depth = torch.tensor([-.02, -.01, 0., .01], dtype=torch.float64, requires_grad=True)
        layer.overlaps = lambda state,margin:{'left_source_slot':(depth+margin).clamp(min=0).reshape(1, -1, 1)}
        cost = layer.cost(None)
        gradient = torch.autograd.grad(cost.sum(), depth)[0]
        torch.testing.assert_close(gradient, 2*(depth.detach()+.015).clamp(min=0))
        self.assertGreater(float(gradient[1]), 0.)

    def test_world_robot_payload_ground_match_independent_cpu_geometry(self):
        count = 81
        dimensions = dict(left=(.3, .4, .4), right=(.255, .465, .582))
        indices = NS(get_sphere_index_from_link_name=lambda name: torch.tensor([0, 1] if name == 'left_link7' else [2, 3]))
        kin = NS(config=NS(kinematics_config=indices), total_spheres=5,
                 get_joint_limits=lambda: NS(position=torch.zeros((2, 15))))
        checker = NS(kinematics=kin, task_tool_to_box={s: np.eye(4) for s in dimensions}, base_cuboid_layer=NS(scene=NS(cuboid=[])))
        original = dict(kinematics=dict(collision_spheres=dict(left_link7=[0], right_link7=[0])))
        obstacle = NS(pose=[0., 0., 1., *np.roll(Rotation.from_euler('z', 30, degrees=True).as_quat(), 1)], dims=(.4, .2, .6))
        scene = NS(cuboid=[obstacle])
        layer = load_layer()(checker, original, dimensions, scene)
        positions = dict(left=np.c_[np.linspace(-1., 1., count), np.zeros(count), np.ones(count)],
                         right=np.c_[np.full(count, .7), np.linspace(-.7, .7, count), np.linspace(.2, .6, count)])
        rotations = {s: Rotation.from_euler('z', np.linspace(10, 25, count), degrees=True).as_matrix() for s in dimensions}
        poses = {s+'_tool0': NS(position=torch.tensor(p[None], dtype=torch.float32), quaternion=torch.tensor(
            np.roll(Rotation.from_matrix(rotations[s]).as_quat(), 1, axis=1)[None], dtype=torch.float32)) for s, p in positions.items()}
        spheres = np.zeros((count, 5, 4), dtype=np.float32)
        spheres[:, :4, :3] = 10.; spheres[:, :4, 3] = .05
        spheres[:, 4] = [.5, 0., 1., .1]
        state = NS(cuda_robot_model_state=NS(robot_spheres=torch.tensor(spheres[None])), tool_poses=NS(to_dict=lambda: poses))
        actual = layer.overlaps(state)
        world_rot = Rotation.from_quat(np.roll(obstacle.pose[3:], -1)).as_matrix()
        for side in dimensions:
            expected = box_overlap_depth(positions[side], rotations[side], np.array(dimensions[side])/2,
                np.array(obstacle.pose[:3]), world_rot, np.array(obstacle.dims)/2).clip(min=0)
            np.testing.assert_allclose(actual[side+'_world'].numpy().reshape(-1), expected, atol=1e-6, rtol=1e-5)
            own = 0 if side == 'left' else 2
            ids = [i for i in (0, 2, 4) if i != own]
            expected = box_sphere_depth(spheres[:, ids, :3], spheres[:, ids, 3], positions[side], rotations[side], np.array(dimensions[side])/2).clip(min=0)
            np.testing.assert_allclose(actual[side+'_robot'].numpy()[0], expected, atol=1e-6, rtol=1e-5)
            ground = ((abs(rotations[side][:, 2])*np.array(dimensions[side])/2).sum(-1)-positions[side][:, 2]).clip(min=0)
            np.testing.assert_allclose(actual[side+'_ground'].numpy().reshape(-1), ground, atol=1e-6)
        expected = box_overlap_depth(positions['left'], rotations['left'], np.array(dimensions['left'])/2,
            positions['right'], rotations['right'], np.array(dimensions['right'])/2).clip(min=0)
        np.testing.assert_allclose(actual['box_box'].numpy().reshape(-1), expected, atol=1e-6, rtol=1e-5)
        with self.assertRaises(ValueError):layer.classify(state)
        precise = load_layer()(checker, original, dimensions, scene, dtype=torch.float64)
        valid, uncertain = precise.classify(state)
        expected_valid = np.ones(count, dtype=bool)
        for name, depth in actual.items():
            threshold = .0010001 if name.endswith('_world') else (.001 if name.endswith('_ground') else 1e-6)
            expected_valid &= ~(depth.numpy()[0] > threshold).any(-1)
        np.testing.assert_array_equal(valid.numpy()[0], expected_valid)
        self.assertTrue(bool(uncertain.any()), 'axis-aligned ground-tangent boxes require conservative CPU fallback')
        slots = {side:([.3, 0., 1.], np.asarray(dimensions[side])/2, .001) for side in dimensions}
        layer.update_source_slots(slots)
        source_depths = layer.overlaps(state)
        for side in dimensions:
            expected = (box_overlap_depth(positions[side], rotations[side], np.asarray(dimensions[side])/2,
                np.asarray(slots[side][0]), np.eye(3), slots[side][1])-.001).clip(min=0)
            np.testing.assert_allclose(source_depths[side+'_source_slot'].numpy().reshape(-1), expected, atol=1e-6)
        precise.update_source_slots(slots)
        unchanged, _ = precise.classify(state)
        torch.testing.assert_close(unchanged, valid)  # Guidance must not alter physical classification.
        addresses = {s:{k:v.data_ptr() for k,v in data.items()} for s,data in layer.source_slots.items()}
        layer.update_source_slots({s:([.4, 0., 1.], half, .002) for s,(_,half,_) in slots.items()})
        self.assertEqual(addresses, {s:{k:v.data_ptr() for k,v in data.items()} for s,data in layer.source_slots.items()})
        with self.assertRaises(ValueError):layer.update_source_slots({})
        # Scene updates keep graph-visible buffers stable; topology changes must rebuild.
        address = layer.world_centers.data_ptr()
        obstacle.pose[0] = 3.
        layer.update_world(scene)
        self.assertEqual(address, layer.world_centers.data_ptr())
        self.assertEqual(float(layer.world_centers[0, 0]), 3.)
        with self.assertRaises(ValueError):layer.update_world(NS(cuboid=[obstacle, obstacle]))
        self.assertTrue(torch.isfinite(layer.cost(state)).all())


if __name__ == '__main__':unittest.main()


def test_classification_and_robot_clearance_share_one_overlap_query():
    cls=load_layer();layer=cls.__new__(cls)
    layer.dtype=torch.float64
    calls=[]
    depths={'left_robot':torch.tensor([[[-6e-6,-7e-6],[-5e-6,-8e-6],[-4.99e-6,-1.],[2e-6,-1.],[float('nan'),-1.]]],dtype=torch.float64),
            'right_robot':torch.full((1,5,2),-1.,dtype=torch.float64)}
    def overlaps(state,**kwargs):
        calls.append(kwargs)
        return depths,torch.zeros((1,5),dtype=torch.bool)
    layer._overlaps=overlaps
    state=NS(cuda_robot_model_state=NS(robot_spheres=torch.zeros((1,5,1,4))))
    valid,uncertain,robot_depth=layer.classify_with_robot_depth(state)
    assert calls==[{'signed':True}]
    expected=torch.ones((1,5),dtype=torch.bool)
    for depth in depths.values():expected &= (depth<=-5e-6).all(-1)
    torch.testing.assert_close(robot_depth<=-5e-6,expected)
    default=layer.classify(state)
    assert len(default)==2
    torch.testing.assert_close(default[0],valid)
    torch.testing.assert_close(default[1],uncertain)
