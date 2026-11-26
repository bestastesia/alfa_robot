"""CPU-only parity tests; CUDA FK boundary is replaced, never initialized here."""
import ast
from pathlib import Path
from types import SimpleNamespace as NS
import sys
import unittest

import numpy as np
from scipy.spatial.transform import Rotation
import torch

TOOLS = Path(__file__).resolve().parents[1]/'tools'
sys.path.insert(0, str(TOOLS))
from curobo_core.payload_geometry import box_overlap_depth, box_sphere_depth


def projected_overlap(p, a, ha, q, b, hb):
    """Pre-optimization 15-axis implementation, retained as independent reference."""
    b = np.broadcast_to(b, (len(p), 3, 3))
    at, bt = a.transpose(0, 2, 1), b.transpose(0, 2, 1)
    axes = np.concatenate((at, bt, np.cross(at[:, :, None], bt[:, None]).reshape(len(p), 9, 3)), axis=1)
    lengths = np.linalg.norm(axes, axis=-1)
    unit = axes/np.maximum(lengths[:, :, None], 1e-15)
    ha = np.sum(np.abs(np.einsum('bai,bij->baj', unit, a))*ha, axis=-1)
    hb = np.sum(np.abs(np.einsum('bai,bij->baj', unit, b))*hb, axis=-1)
    depth = ha+hb-np.abs(np.einsum('bi,bai->ba', np.asarray(q)-p, unit))
    depth[lengths < 1e-10] = np.inf
    return depth.min(-1)


def cpu_audit():
    # Compile the actual function, isolating cuRobo/CUDA imports. Tensor/FK semantics
    # are tested on CPU here; real CUDA input parity is a separate gpuq probe.
    node = next(n for n in ast.parse((TOOLS/'v3_payload_safety.py').read_text()).body
                if isinstance(n, ast.FunctionDef) and n.name == 'exact_payload_audit')
    def tensor(rows, device, dtype):
        assert device == 'cuda'
        return torch.as_tensor(rows, dtype=dtype)
    env = dict(np=np, Rotation=Rotation, torch=NS(as_tensor=tensor, float32=torch.float32),
               JointState=NS(from_position=lambda tensor, joint_names: tensor), ACTIVE_JOINTS=[],
               box_overlap_depth=box_overlap_depth, box_sphere_depth=box_sphere_depth)
    exec(compile(ast.Module(body=[node], type_ignores=[]), 'v3_payload_safety.py', 'exec'), env)
    return env['exact_payload_audit']


class PayloadSafetyTest(unittest.TestCase):
    def test_sat_random_parallel_and_gate_boundaries(self):
        rng = np.random.default_rng(82)
        count = 4096
        half = np.array([.1275, .2325, .291])
        other_half = np.array([.3, .2, .675])
        rotations = Rotation.random(count, random_state=rng).as_matrix()
        for other in (np.eye(3), Rotation.random(count, random_state=rng).as_matrix()):
            args = (rng.normal(size=(count, 3)), rotations, half, rng.normal(size=(count, 3)), other, other_half)
            before, after = projected_overlap(*args), box_overlap_depth(*args)
            np.testing.assert_allclose(after, before, atol=1e-13, rtol=1e-13)
            for threshold in (1e-6, .0010001):
                np.testing.assert_array_equal(after > threshold, before > threshold)
        for angle in (0., 1e-12, 1e-10, 1e-8, 1e-6, np.pi):
            rotations = Rotation.from_rotvec([[angle, 0, 0]]*9).as_matrix()
            for threshold in (0., 1e-6, .0010001):
                depths = threshold+np.linspace(-1e-10, 1e-10, 9)
                positions = np.zeros((9, 3)); positions[:, 0] = half[0]+other_half[0]-depths
                args = (positions, rotations, half, np.zeros(3), np.eye(3), other_half)
                before, after = projected_overlap(*args), box_overlap_depth(*args)
                np.testing.assert_allclose(after, before, atol=1e-13, rtol=0)
                np.testing.assert_array_equal(after > threshold, before > threshold)

    def checker(self):
        calls = []
        def compute(rows):
            calls.append(len(rows[0])); rows = rows[0]
            poses = {}
            for side, span in [('left', slice(0, 3)), ('right', slice(3, 6))]:
                quat = torch.zeros((len(rows), 4)); quat[:, 0] = 1.
                poses[side+'_tool0'] = NS(position=rows[:, span], quaternion=quat)
            # Own link7 touch sphere inside own box is allowed. Other original
            # spheres must still be checked; payload spheres excluded as before.
            spheres = torch.zeros((len(rows), 5, 4))
            spheres[:, 0, :3] = rows[:, :3]; spheres[:, 0, 3] = .01
            spheres[:, 2, :3] = rows[:, 3:6]; spheres[:, 2, 3] = .01
            spheres[:, 4, :3] = torch.tensor([5., 0., 1.]); spheres[:, 4, 3] = .1
            return NS(robot_spheres=spheres, tool_poses=NS(to_dict=lambda: poses))
        indices = NS(get_sphere_index_from_link_name=lambda name: torch.tensor([0, 1] if name == 'left_link7' else [2, 3]))
        kin = NS(config=NS(kinematics_config=indices), total_spheres=5, compute_kinematics=compute)
        obstacle = NS(name='wall', pose=[3., 0., 1., 1., 0., 0., 0.], dims=[.5, .5, .5])
        checker = NS(kinematics=kin, guidance_scene=NS(cuboid=[obstacle]), base_cuboid_layer=NS(scene=NS(cuboid=[])))
        return checker, calls

    def test_broadphase_matches_exhaustive_rotated_boxes_and_spheres(self):
        rng = np.random.default_rng(20261122)
        checker, _ = self.checker()
        rows = rng.normal(size=(1025, 6)).astype(np.float32)
        rows[:, :3] += [3., 0., 1.]
        rows[:, 3:6] += [4., 0., 1.]
        dimensions = {'left': (.255, .465, .582), 'right': (.30, .40, .40)}
        robot = {'kinematics': {'collision_spheres': {'left_link7': [0], 'right_link7': [0]}}}
        attachments = {side: np.eye(4) for side in dimensions}
        for attachment in attachments.values():
            attachment[:3, :3] = Rotation.random(random_state=rng).as_matrix()
            attachment[:3, 3] = rng.normal(0, .1, 3)
        actual = cpu_audit()(checker, rows, attachments, dimensions, robot, check_ground=True)
        # Exhaustive primitive evaluation: no AABB preselection and no GPU.
        state = checker.kinematics.compute_kinematics(torch.as_tensor(rows)[None])
        spheres = state.robot_spheres.numpy()
        poses = state.tool_poses.to_dict()
        invalid = np.zeros(len(rows), dtype=bool)
        maxima = dict(box_world=0., box_robot=0., box_box=0.)
        boxes = {}
        for side, attachment in attachments.items():
            positions = poses[side+'_tool0'].position.numpy()+attachment[:3, 3]
            rotations = np.broadcast_to(attachment[:3, :3], (len(rows), 3, 3))
            half = np.asarray(dimensions[side])/2
            boxes[side] = positions, rotations, half
            bottom = positions[:, 2]-(np.abs(rotations[:, 2])*half).sum(-1)
            invalid |= bottom < -.001
            self.assertEqual(actual['minimum_box_ground_clearance_m'][side], float(bottom.min()))
            for obstacle in checker.guidance_scene.cuboid:
                depth = box_overlap_depth(positions, rotations, half, np.asarray(obstacle.pose[:3]),
                    Rotation.from_quat(np.roll(obstacle.pose[3:], -1)).as_matrix(), np.asarray(obstacle.dims)/2)
                invalid |= depth > .0010001
                maxima['box_world'] = max(maxima['box_world'], float(depth.max(initial=0)))
            selected = [2, 4] if side == 'left' else [0, 4]
            depth = box_sphere_depth(spheres[:, selected, :3], spheres[:, selected, 3], positions, rotations, half)
            invalid |= (depth > 1e-6).any(-1)
            maxima['box_robot'] = max(maxima['box_robot'], float(depth.max(initial=0)))
        a, b = boxes.values()
        depth = box_overlap_depth(*a, *b)
        invalid |= depth > 1e-6
        maxima['box_box'] = float(depth.max(initial=0))
        self.assertEqual(actual['invalid_frame_indices'], np.flatnonzero(invalid).tolist())
        self.assertTrue(invalid.any() and not invalid.all())
        for name, value in maxima.items():
            self.assertAlmostEqual(actual['maximum_overlap_m'][name], value, places=12)

    def test_chunked_audit_ground_masks_spheres_and_tensor_inputs(self):
        audit = cpu_audit()
        checker, calls = self.checker()
        rows = np.tile([0., 0., 1., 0., 1., 1.], (513, 1)).astype(np.float32)
        rows[255, 2] = .19  # full box bottom -1cm; spheres alone are insufficient
        rows[256, 3:6] = rows[256, :3]  # two boxes overlap
        rows[512, :3] = [3., 0., 1.]  # world cuboid collision
        rows[511, :3] = [5., 0., 1.]  # original robot sphere, not a payload sphere
        attachments = {side: np.eye(4) for side in ('left', 'right')}
        dimensions = dict(left=(.3, .4, .4), right=(.255, .465, .582))
        robot = dict(kinematics=dict(collision_spheres=dict(left_link7=[0], right_link7=[0])))
        before = audit(checker, rows, attachments, dimensions, robot)
        self.assertEqual(before['invalid_frame_indices'], [256, 511, 512])
        self.assertEqual(calls, [256, 256, 1])
        calls.clear()
        after = audit(checker, torch.from_numpy(rows), attachments, dimensions, robot, check_ground=True)
        self.assertFalse(after['valid'])
        self.assertEqual(after['invalid_frame_indices'], [255, 256, 511, 512])
        self.assertEqual(calls, [256, 256, 1])  # no second full-path FK for ground
        self.assertAlmostEqual(after['minimum_box_ground_clearance_m']['left'], -.01, places=7)
        self.assertTrue(any(f['kind'] == 'box_ground' for f in after['failures']))
        one = audit(checker, rows[:1], {'left': np.eye(4)}, (.3, .4, .4), robot, check_ground=True)
        self.assertTrue(one['valid'])
        self.assertIsInstance(one['minimum_box_ground_clearance_m'], float)
        with self.assertRaises(ValueError):audit(checker, rows, attachments, dimensions, robot, tolerance_m=.0011)


if __name__ == '__main__':unittest.main()
