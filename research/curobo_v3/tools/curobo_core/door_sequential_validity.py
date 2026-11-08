# Reused from five-column-rear-trajopt sequential_validity.py, SHA256 646b8f73d6a05028e1b5277f1f1b07cd976b66c9ed3a409feaf3b8a53baebda1
"""Native validation plus exact, pair-scoped tool contact and payload geometry."""
import time
import math

import torch

from curobo.types import JointState
from v3_batched_loaded_search import GpuValidity
from .adapter import pose_matrix
from .contracts import ACTIVE_JOINTS


def matmul3(a, b):
    """3D geometry products without TF32's ~1e-3 precision truncation.

    cuRobo IK enables TF32 globally. Elementwise three-term dot products keep
    float32 precision independent of batch size without changing global flags.
    """
    return (a[..., :, :, None] * b[..., None, :, :]).sum(-2)


def quaternion_matrix(q):
    """Batch wxyz to rotation; cuRobo FK supplies normalized quaternions."""
    w, x, y, z = q.unbind(-1)
    return torch.stack((1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y),
                        2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x),
                        2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)), -1).reshape(*q.shape[:-1], 3, 3)


def obb_overlap(a_position, a_rotation, a_half, b_position, b_rotation, b_half, tolerance=1e-6):
    """Batched separating-axis test; tangency is not volume penetration."""
    a_axes, b_axes = a_rotation.transpose(-1, -2), b_rotation.transpose(-1, -2)
    cross = torch.linalg.cross(a_axes[..., :, None, :], b_axes[..., None, :, :], dim=-1)
    axes = torch.cat((a_axes, b_axes, cross.flatten(-3, -2)), dim=-2)
    norm = torch.linalg.vector_norm(axes, dim=-1)
    axes = axes / norm.clamp_min(1e-12)[..., None]
    distance = ((b_position-a_position)[..., None, :] * axes).sum(-1).abs()
    extent_a = (matmul3(axes, a_rotation).abs() * a_half[..., None, :]).sum(-1)
    extent_b = (matmul3(axes, b_rotation).abs() * b_half[..., None, :]).sum(-1)
    return ((distance < extent_a + extent_b - tolerance) | (norm < 1e-8)).all(-1)


def world_box_overlaps(position, rotation, half, world_position, world_rotation, world_half, tolerance=1e-6):
    """Same SAT, one batch over all immutable obstacles instead of one launch per box."""
    n, m = len(position), len(world_position)
    return obb_overlap(position[:, None].expand(n, m, 3), rotation[:, None].expand(n, m, 3, 3), half,
                       world_position[None].expand(n, m, 3), world_rotation[None].expand(n, m, 3, 3),
                       world_half[None], tolerance=tolerance).any(dim=-1)


def material_self_overlap(spheres, pairs, padding, payload_pair, tolerance=1e-6):
    """Verify native self pairs in float64, tolerating only payload/payload tangency.

    Robot/robot and robot/payload keep zero penetration tolerance. The payload
    pair uses the existing 1um actual-box SAT tolerance, not a new exemption.
    """
    spheres = spheres.double()
    first, second = spheres[:, pairs[:, 0]], spheres[:, pairs[:, 1]]
    clearance = (torch.linalg.vector_norm(first[..., :3]-second[..., :3], dim=-1)
                 -first[..., 3]-second[..., 3]-padding[pairs[:, 0]]-padding[pairs[:, 1]])
    threshold = torch.where(payload_pair, -tolerance, 0.)
    return ((first[..., 3] > 0.) & (second[..., 3] > 0.) & (clearance < threshold)).any(-1)


class SequentialValidity(GpuValidity):
    def __init__(self,checker,snapshot,robot,tool_meshes,contact_pair=None,geometry_tolerance=1e-6,support_pose=None,support_side=None):
        super().__init__(checker, check_ground=True, ground_z=snapshot.policy.ground_z_m,
                         joint_names=ACTIVE_JOINTS, max_box_tilt_deg=snapshot.policy.max_box_tilt_deg)
        self.snapshot = snapshot
        self.geometry_tolerance = geometry_tolerance
        self.support_pose=support_pose
        self.support_side=support_side or ('right' if support_pose is not None else None)
        self.attachments = snapshot.attachments
        self.collision_ms = 0.
        self.minimum_edge_steps = 2
        self.contact_pair = contact_pair
        self.tool_meshes = tool_meshes
        kin = robot.get("robot_cfg", robot)["kinematics"]
        config = checker.kinematics.config.kinematics_config
        # loaded_robot appends payload spheres after each link's original spheres.
        self.link_indices = {link: [int(i) for i in config.get_sphere_index_from_link_name(link).cpu().tolist()[:len(spheres)]]
                             for link, spheres in kin["collision_spheres"].items()}
        self.robot_indices = [i for indices in self.link_indices.values() for i in indices]
        self.payload_indices = [int(i) for link in self.link_indices
                                for i in config.get_sphere_index_from_link_name(link).cpu().tolist()
                                if int(i) not in self.robot_indices]
        self_params = checker.kinematics.get_self_collision_config()
        self.self_pairs = self_params.collision_pairs.long()
        self.self_padding = self_params.sphere_padding.double()
        payload = torch.zeros(len(self_params.sphere_padding), device=self.self_pairs.device, dtype=torch.bool)
        payload[self.payload_indices] = True
        self.payload_pairs = payload[self.self_pairs[:, 0]] & payload[self.self_pairs[:, 1]]
        self.robot_self_pairs=~(payload[self.self_pairs[:,0]]|payload[self.self_pairs[:,1]])
        from .door_sequential import mesh_contact_proof
        import numpy as np
        for item in self.attachments:
            side = item.parent_link.removesuffix("_tool0")
            proof = mesh_contact_proof(tool_meshes[side], np.linalg.inv(pose_matrix(item.tool_to_object)),
                                       item.dimensions_m, .001, tolerance=geometry_tolerance)
            if item.touch_links != (side+"_link7",) or not proof["valid"]:
                raise ValueError("attachment lacks an exact own-tool contact certificate")
        if contact_pair is not None and any(item.parent_link == contact_pair[0]+"_tool0" for item in self.attachments):
            raise ValueError("a loaded tool cannot contact a second target")

    def _self_collision_cost(self, spheres):
        if not self.payload_indices:
            return super()._self_collision_cost(spheres)
        # Payload-vs-robot and payload-vs-payload are checked below with the
        # exact attached-box OBB; keep native sphere checks only robot-vs-robot.
        rows=spheres.reshape(-1,spheres.shape[-2],4)
        pairs=self.self_pairs[self.robot_self_pairs]
        payload=torch.zeros(len(pairs),device=pairs.device,dtype=torch.bool)
        collision=torch.empty(len(rows),device=rows.device,dtype=torch.bool)
        for offset in range(0,len(rows),512):
            collision[offset:offset+512]=material_self_overlap(rows[offset:offset+512],pairs,self.self_padding,payload,0.)
        return collision.to(spheres.dtype).reshape(*spheres.shape[:-2],1)

    def _world_collision_cost(self, state):
        cost = super()._world_collision_cost(state)
        # Exact box/box tangency produces ~3e-8 m positive float32 SDF cost.
        # Only payload spheres get the SAME 1um tolerance as the independent
        # actual-box SAT gate; robot-world/self/bounds remain unchanged.
        if self.payload_indices:
            # Exact attached-box OBB/world checks run below for every row; payload
            # spheres are only a broad-phase model and overhang tangent neighbours.
            cost[..., self.payload_indices] = 0.
        return cost

    def evaluate(self, values):
        torch.cuda.synchronize()
        started = time.perf_counter()
        valid, _ = super().evaluate(values)
        if self.attachments and not hasattr(self, "world_geometry"):
            import numpy as np
            objects = [item for item in self.snapshot.objects if item.object_id != "ground"]
            matrices = torch.as_tensor(np.array([pose_matrix(item.pose) for item in objects]),
                                       device=values.device, dtype=values.dtype)
            half = torch.as_tensor([item.dimensions_m for item in objects],
                                   device=values.device, dtype=values.dtype)/2
            self.world_geometry = (matrices[:, :3, 3], matrices[:, :3, :3], half)
        costs = torch.zeros(len(values), device=values.device)
        # Bound temporary OBB/sphere tensors, including long RRT edge batches.
        for offset in range(0, len(values), 1024):
            q = values[offset:offset+1024]
            fk = self.checker.kinematics.compute_kinematics(
                JointState.from_position(q[None], joint_names=list(ACTIVE_JOINTS)))
            tools = fk.tool_poses.to_dict()
            all_spheres = fk.robot_spheres.reshape(len(q), -1, 4)
            spheres = all_spheres[:, self.robot_indices]
            if self.contact_pair is not None:
                side, object_id = self.contact_pair
                item = self.snapshot.object(object_id)
                matrix = torch.as_tensor(pose_matrix(item.pose), device=q.device, dtype=q.dtype)
                half = torch.tensor(item.dimensions_m, device=q.device, dtype=q.dtype)/2
                local = matmul3(all_spheres[..., :3]-matrix[:3, 3], matrix[:3, :3])
                distance = torch.linalg.vector_norm((local.abs()-half).clamp_min(0), dim=-1)
                hit = (all_spheres[..., 3] > 0) & (distance < all_spheres[..., 3]-self.geometry_tolerance)
                # Only this tool/target pair is replaced by the mesh proof. Other
                # links and attached boxes still collide with the complete target.
                hit[:, self.link_indices[side+"_link7"]] = False
                pose = tools[side+"_tool0"]
                rotation = quaternion_matrix(pose.quaternion.reshape(-1, 4))
                relative_rotation = matmul3(matrix[:3, :3].T, rotation)
                relative_position = matmul3(pose.position.reshape(-1, 3)-matrix[:3, 3], matrix[:3, :3])
                vertices = torch.as_tensor(self.tool_meshes[side], device=q.device, dtype=q.dtype)
                local_mesh = matmul3(vertices[None], relative_rotation.transpose(-1, -2)) + relative_position[:, None]
                front = local_mesh[..., 0].max(dim=-1).values
                gap = -half[0]-front
                footprint = local_mesh[..., 0] >= front[:, None]-.001
                inside = (local_mesh[..., 1:].abs() <= half[1:]+1e-6).all(-1)
                proof = (gap >= -self.geometry_tolerance) & (gap <= .051) & ((~footprint) | inside).all(-1)
                valid[offset:offset+len(q)] &= ~hit.any(-1) & proof
            boxes = []
            for item in self.attachments:
                pose = tools[item.parent_link]
                r_tool = quaternion_matrix(pose.quaternion.reshape(-1, 4))
                t_tool = pose.position.reshape(-1, 3)
                relative = torch.as_tensor(pose_matrix(item.tool_to_object), device=q.device, dtype=q.dtype)
                rotation = matmul3(r_tool, relative[:3, :3])
                position = t_tool + (r_tool * relative[:3, 3]).sum(-1)
                half = torch.tensor(item.dimensions_m, device=q.device, dtype=q.dtype) / 2
                bottom = position[:, 2] - (rotation[:, 2].abs() * half).sum(-1)
                tilt = 1 - rotation[:, 2, 2].clamp(-1, 1)
                costs[offset:offset+len(q)] = torch.maximum(costs[offset:offset+len(q)], tilt)
                good = (bottom >= self.ground_z-1e-6) & (tilt <= 1-self.minimum_box_up_z)
                # Rigid own-tool contact was certified from the original mesh.
                # Every other robot sphere still checks the exact payload box.
                local = matmul3(spheres[..., :3]-position[:, None], rotation)
                distance = torch.linalg.vector_norm((local.abs()-half).clamp_min(0), dim=-1)
                overlap = (spheres[..., 3] > 0) & (distance < spheres[..., 3]-1e-6)
                own_link = item.parent_link.removesuffix("_tool0")+"_link7"
                own_indices = set(self.link_indices[own_link])
                own_columns = [i for i, sphere_index in enumerate(self.robot_indices) if sphere_index in own_indices]
                overlap[:, own_columns] = False
                good &= ~overlap.any(-1)
                good &= ~world_box_overlaps(position, rotation, half, *self.world_geometry, tolerance=self.geometry_tolerance)
                for other_position, other_rotation, other_half in boxes:
                    good &= ~obb_overlap(position, rotation, half, other_position, other_rotation, other_half, tolerance=self.geometry_tolerance)
                if self.support_pose is not None and item.parent_link == self.support_side+"_tool0":
                    reference = torch.as_tensor(self.support_pose, device=q.device, dtype=q.dtype)
                    position_ok = torch.linalg.vector_norm(position-reference[:3, 3], dim=-1) <= .001
                    cosine = ((rotation*reference[:3, :3]).sum(dim=(-1, -2))-1.)/2
                    good &= position_ok & (cosine >= math.cos(math.radians(.5)))
                boxes.append((position, rotation, half))
                valid[offset:offset+len(q)] &= good
        torch.cuda.synchronize()
        self.collision_ms += (time.perf_counter()-started)*1000
        return valid, costs


class ReducedValidity(GpuValidity):
    """Use upstream RRTConnect in exactly 7 or 8 variables; expand only for FK/checking."""
    def __init__(self, full, reference, indices):
        self.full = full
        self.minimum_edge_steps = 2
        self.indices = list(indices)
        self.joint_names = [ACTIVE_JOINTS[i] for i in self.indices]
        self.reference = torch.as_tensor(reference, device="cuda", dtype=torch.float32)
        self.weights = full.weights[self.indices]
        self.stability_weight = full.stability_weight

    def expand(self, values):
        output = self.reference.expand(len(values), -1).clone()
        output[:, self.indices] = values
        return output

    def evaluate(self, values):
        return self.full.evaluate(self.expand(values))
