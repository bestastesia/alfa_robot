# Reused from five-column-rear-trajopt sequential_validity.py, SHA256 646b8f73d6a05028e1b5277f1f1b07cd976b66c9ed3a409feaf3b8a53baebda1
"""Native validation plus exact, pair-scoped tool contact and payload geometry."""
import copy
from dataclasses import dataclass, replace
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


def box_mesh_obb(item):
    """Exact OBB of a mesh whose vertices are the 8 corners of its local bounding box, else None."""
    import numpy as np
    vertices = np.asarray(item.vertices_m, dtype=float)
    low, high = vertices.min(0), vertices.max(0)
    corner = np.isclose(vertices, high, atol=1e-9, rtol=0)
    if (len(vertices) != 8 or np.any(high-low <= 0) or
            not np.all(corner | np.isclose(vertices, low, atol=1e-9, rtol=0)) or len({tuple(row) for row in corner}) != 8):
        return None
    matrix = pose_matrix(item.pose)
    return matrix[:3, 3]+matrix[:3, :3]@((low+high)/2), matrix[:3, :3], (high-low)/2


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


class SearchInflation:
    """PR #80 free-space search geometry (v3_fixed_body_boxes.WorldMarginChecker): robot sphere radii
    against the world become (r+tolerance)*scale-tolerance and the fixed-body cuboids and exact payload
    boxes scale their half extents; self, internal fixed-body, ground and bounds keep the original
    geometry. States within `radius` (weighted joint distance) of an anchor, the search's start and
    goals, keep the original geometry too: the moments next to a contact use the original volume."""

    def __init__(self, anchors, weights, scale=1.1, tolerance=.001, radius=.1, record=None):
        if not (math.isfinite(scale) and scale >= 1. and 0. <= tolerance <= .001 and radius >= 0.):
            raise ValueError("invalid search inflation")
        self.weights = torch.as_tensor(weights, device="cuda", dtype=torch.float32)
        self.anchors = torch.as_tensor(anchors, device="cuda", dtype=torch.float32).reshape(-1, len(self.weights))
        self.scale, self.tolerance, self.radius = float(scale), float(tolerance), float(radius)
        # A JSON-safe statistics record, updated in place (e.g. one entry of a planner report).
        self.record = {} if record is None else record
        self.record.update(scale=self.scale, tolerance_m=self.tolerance, anchor_radius=self.radius,
                           anchors=len(self.anchors), rows=0, anchor_exempt_rows=0)

    def near(self, values):
        distance = torch.cdist(values*self.weights, self.anchors*self.weights)
        return (distance <= self.radius).any(-1)

    def radii(self, radius):
        return torch.where(radius > 0, (radius+self.tolerance)*self.scale-self.tolerance, radius)


@dataclass(frozen=True)
class CollisionMargins:
    """Planning margins on top of the frozen spheres (MOTION-429); the native checks are unchanged.

    world: links whose spheres grow by their per-sphere coverage margins against world boxes, i.e.
        arms moving in free space; exempt: (side, tool0 points) balls of exempt_radius_m where that
        arm keeps its original spheres, next to a contact it approaches or leaves.
    self_pairs: (link_a, link_b, margin_m) robot sphere pairs that must clear by margin_m.
    payload: (link, margin_m) robot spheres that must clear the carried boxes by margin_m.
    cross_pairs: (left link, right link) sphere pairs that must clear by the coverage margins of the
        free-space arms among them, each counted while its tool0 is outside its exempt balls.
    """
    world: tuple = ()
    exempt: tuple = ()
    exempt_radius_m: float = .06
    self_pairs: tuple = ()
    payload: tuple = ()
    cross_pairs: tuple = ()


class SequentialValidity(GpuValidity):
    inflation = None  # SearchInflation of a free-space search view (see inflated)
    active_inflation = None  # rows of the native batch being checked with the inflated geometry
    shape_bucket = 256

    def __init__(self,checker,snapshot,robot,tool_meshes,contact_pair=None,geometry_tolerance=1e-6,support_pose=None,support_side=None,
                 fixed_body=None,margins=None,sphere_margins=None):
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
        self.self_margin_m = 1e-5
        self.self_band_rows = 0
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
        # PR #80 fixed body (door_model.FixedBodyBoxes): the planner's robot has no base_link spheres.
        self.fixed_body = fixed_body
        if fixed_body is not None:
            device = self.self_pairs.device
            self.fixed_body_boxes = tuple(torch.as_tensor(value, device=device, dtype=torch.float32)
                                          for value in (fixed_body.centers, fixed_body.rotations, fixed_body.halves))
            self.fixed_body_query = [i for link, indices in self.link_indices.items()
                                     if link not in fixed_body.query_excluded_links for i in indices]
            self.fixed_body_world_clear = {}
        self.margins = margins or CollisionMargins()
        device = self.self_pairs.device
        growth, side = [], []
        for link in self.margins.world:
            values = sphere_margins[link]
            if len(values) != len(self.link_indices[link]):
                raise ValueError(f"coverage margins do not match the sphere model of {link}")
            growth.extend(float(value) for value in values)
            side.extend([0 if link.startswith("left_") else 1]*len(values))
        self.world_growth_indices = torch.tensor([i for link in self.margins.world for i in self.link_indices[link]],
                                                 device=device, dtype=torch.long)
        self.world_growth = torch.tensor(growth, device=device)
        self.world_growth_side = torch.tensor(side, device=device, dtype=torch.long)
        self.world_exempt = [None, None]
        for name, points in self.margins.exempt:
            self.world_exempt[0 if name == "left" else 1] = torch.tensor(points, device=device, dtype=torch.float32).reshape(-1, 3)
        pairs = [(i, j, margin) for first, second, margin in self.margins.self_pairs
                 for i in self.link_indices[first] for j in self.link_indices[second]]
        self.margin_pairs = torch.tensor([pair[:2] for pair in pairs], device=device, dtype=torch.long).reshape(-1, 2)
        self.margin_pair_values = torch.tensor([pair[2] for pair in pairs], device=device)
        self.growth_sides = torch.tensor([any(link.startswith(name+"_") for link in self.margins.world)
                                          for name in ("left", "right")], device=device)
        if any(not (first.startswith("left_") and second.startswith("right_")) for first, second in self.margins.cross_pairs):
            raise ValueError("cross-arm margin pairs are (left link, right link)")
        cross = [(i, j, first_margin, second_margin) for first, second in self.margins.cross_pairs
                 for i, first_margin in zip(self.link_indices[first], sphere_margins[first])
                 for j, second_margin in zip(self.link_indices[second], sphere_margins[second])]
        self.cross_pairs = torch.tensor([pair[:2] for pair in cross], device=device, dtype=torch.long).reshape(-1, 2)
        self.cross_margins = torch.tensor([pair[2:] for pair in cross], device=device, dtype=torch.float32).reshape(-1, 2)
        payload_growth = dict(self.margins.payload)
        self.payload_growth = torch.tensor([payload_growth.get(link, 0.) for link, indices in self.link_indices.items()
                                            for _ in indices], device=device)

    def inflated(self, inflation):
        """A free-space search view: the same checks with `inflation` (its time counts towards this validity)."""
        view = copy.copy(self)
        view.inflation, view.counter_owner = inflation, self
        return view

    def fixed_body_collision(self, spheres, scale=None):
        """Rows whose robot spheres come within the activation distance of a fixed-body cuboid, or whose
        cuboids overlap the world (static per scene; half extents scaled per row in free-space search)."""
        centers, rotations, halves = self.fixed_body_boxes
        query = spheres[:, self.fixed_body_query]
        local = ((query[:, :, None, :3]-centers)[..., :, None]*rotations).sum(-2)
        distance = torch.linalg.vector_norm((local.abs()-halves).clamp_min(0), dim=-1)
        radius = query[..., 3:4]
        collision = ((radius > 0) & (distance < radius+self.fixed_body.activation_m)).flatten(1).any(-1)
        for value in {1.} if scale is None else {1., self.inflation.scale}:
            if value not in self.fixed_body_world_clear:
                position, rotation, half = self.world_geometry()
                self.fixed_body_world_clear[value] = not bool(len(position)) or not bool(world_box_overlaps(
                    centers, rotations, halves[:, None]*value, position, rotation, half,
                    tolerance=self.fixed_body.world_tolerance_m).any())
        if scale is None:
            return collision | (not self.fixed_body_world_clear[1.])
        grown = scale > 1.
        return (collision | (grown & (not self.fixed_body_world_clear[self.inflation.scale]))
                | (~grown & (not self.fixed_body_world_clear[1.])))

    def world_geometry(self):
        """Exact OBBs (position, rotation, half extents) of all non-ground world boxes and box meshes."""
        if not hasattr(self, "_world_geometry"):
            import numpy as np
            objects = [item for item in self.snapshot.objects if item.object_id != "ground"]
            boxes = [(pose_matrix(item.pose)[:3, 3], pose_matrix(item.pose)[:3, :3], np.asarray(item.dimensions_m)/2)
                     for item in objects]
            # Payload spheres carry no world cost, so world meshes need the same exact
            # payload OBB test as cuboids. Only box-shaped meshes have an exact OBB.
            for item in self.snapshot.meshes:
                box = box_mesh_obb(item)
                if box is None:
                    raise ValueError(f"payload exact world check requires box meshes: {item.object_id}")
                boxes.append(box)
            shapes = ((-1, 3), (-1, 3, 3), (-1, 3))
            self._world_geometry = tuple(torch.as_tensor(np.array([box[index] for box in boxes]).reshape(shapes[index]),
                                                         device=self.self_pairs.device, dtype=torch.float32)
                                         for index in range(3))
        return self._world_geometry

    def exact_robot_self_collision(self, rows):
        """Float64 zero-tolerance robot-vs-robot pair check (the reference decision)."""
        pairs=self.self_pairs[self.robot_self_pairs]
        payload=torch.zeros(len(pairs),device=pairs.device,dtype=torch.bool)
        collision=torch.empty(len(rows),device=rows.device,dtype=torch.bool)
        for offset in range(0,len(rows),512):
            collision[offset:offset+512]=material_self_overlap(rows[offset:offset+512],pairs,self.self_padding,payload,0.)
        return collision

    def _self_collision_cost(self, spheres):
        if not self.payload_indices:
            return super()._self_collision_cost(spheres)
        # Payload-vs-robot and payload-vs-payload are checked below with the
        # exact attached-box OBB; keep native sphere checks only robot-vs-robot.
        # The native float32 kernel ignores spheres with negative radius. Running it
        # with robot radii widened and narrowed by a margin far above float32 error
        # brackets the exact answer; only rows inside the bracket get the float64 check.
        radius=spheres[...,3]
        active=radius>0
        active[...,self.payload_indices]=False
        bounds=[]
        for delta in (self.self_margin_m,-self.self_margin_m):
            shifted=spheres.clone();shifted[...,3]=torch.where(active,radius+delta,-100.)
            bounds.append(super()._self_collision_cost(shifted).reshape(-1)>0)
        maybe,certain=bounds
        collision=certain.clone()
        band=torch.nonzero(maybe&~certain).reshape(-1)
        if len(band):
            self.self_band_rows+=len(band)
            collision[band]=self.exact_robot_self_collision(spheres.reshape(-1,spheres.shape[-2],4)[band])
        return collision.to(spheres.dtype).reshape(*spheres.shape[:-2],1)

    def _near_exempt(self, tools, side, shape):
        """Where that arm's tool0 is inside one of its exempt balls (rows of `shape`)."""
        points = self.world_exempt[side]
        if points is None or not len(points):
            return torch.zeros(shape, device=self.world_growth.device, dtype=torch.bool)
        tool = tools[("left", "right")[side]+"_tool0"].position.reshape(*shape, 3)
        return (torch.linalg.vector_norm(tool[..., None, :]-points, dim=-1) <= self.margins.exempt_radius_m).any(-1)

    def _cross_arm_clash(self, spheres, tools):
        """Rows (spheres: rows x n x 4) whose arms' meshes may meet: a cross pair closer than the coverage
        margins of its free-space arms whose tool0 is outside the exempt balls (MOTION-429)."""
        grown = torch.stack([self.growth_sides[side] & ~self._near_exempt(tools, side, (len(spheres),))
                             for side in range(2)], dim=-1)
        first, second = spheres[:, self.cross_pairs[:, 0]], spheres[:, self.cross_pairs[:, 1]]
        clearance = torch.linalg.vector_norm(first[..., :3]-second[..., :3], dim=-1)-first[..., 3]-second[..., 3]
        margin = self.cross_margins[:, 0]*grown[:, :1]+self.cross_margins[:, 1]*grown[:, 1:]
        return ((first[..., 3] > 0) & (second[..., 3] > 0) & (clearance < margin)).any(-1)

    def _grown_world_cost(self, state):
        """Native world cost of the free-space arms' coverage-grown spheres (others disabled),
        zero where that arm's tool0 is inside an exempt ball."""
        index = self.world_growth_indices
        spheres = state.robot_spheres
        grown = spheres.clone()
        grown[..., 3] = -100.
        radius = spheres[..., index, 3]
        grown[..., index, 3] = torch.where(radius > 0, radius+self.world_growth, -100.)
        extra = self.checker.collision_constraint.forward(replace(state, robot_spheres=grown))[..., index]
        tools = state.tool_poses.to_dict()
        for side in range(2):
            near = self._near_exempt(tools, side, extra.shape[:-1])
            extra = extra*~(near[..., None] & (self.world_growth_side == side))
        output = torch.zeros(spheres.shape[:-1], device=spheres.device, dtype=extra.dtype)
        output[..., index] = extra
        return output

    def _world_collision_cost(self, state):
        inflate = self.active_inflation
        if inflate is None:
            cost = super()._world_collision_cost(state)
        else:
            # Same native query as the baseline (already set up for this batch), inflated rows grown.
            spheres = state.robot_spheres
            query = spheres.clone()
            query[..., 3] = torch.where(inflate.reshape(*spheres.shape[:-2], 1), self.inflation.radii(spheres[..., 3]),
                                        spheres[..., 3])
            cost = self.checker.collision_constraint.forward(replace(state, robot_spheres=query))
        if len(self.world_growth_indices):
            # The native kernel reuses its output buffer: keep the original cost before the second query.
            cost = cost.clone()+self._grown_world_cost(state)
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
        count = len(values)
        if not count:
            return (torch.zeros(0, device=values.device, dtype=torch.bool),
                    torch.zeros(0, device=values.device))
        # cuRobo-Fast pays a one-time cost per new batch shape (about 15 ms, or a compile on first sight);
        # padding to whole buckets bounds the shapes a search produces. Rows are checked independently.
        padded = -(-count//self.shape_bucket)*self.shape_bucket
        if padded != count:
            values = torch.cat((values, values[-1:].expand(padded-count, -1)))
        inflate = None
        if self.inflation is not None:
            inflate = ~self.inflation.near(values)
            self.inflation.record["rows"] += count
            self.inflation.record["anchor_exempt_rows"] += count-int(inflate[:count].sum())
        valid, costs = self._evaluate(values, inflate)
        torch.cuda.synchronize()
        getattr(self, "counter_owner", self).collision_ms += (time.perf_counter()-started)*1000
        return valid[:count], costs[:count]

    def _evaluate(self, values, inflate):
        """Validity of `values`; rows where `inflate` is set use the free-space search geometry."""
        valid = torch.empty(len(values), device=values.device, dtype=torch.bool)
        for offset in range(0, len(values), 4096):
            self.active_inflation = None if inflate is None else inflate[offset:offset+4096]
            try:
                valid[offset:offset+4096] = super().evaluate(values[offset:offset+4096])[0]
            finally:
                self.active_inflation = None
        scale = None if inflate is None else torch.where(inflate, self.inflation.scale, 1.)
        costs = torch.zeros(len(values), device=values.device)
        # Bound temporary OBB/sphere tensors, including long RRT edge batches.
        for offset in range(0, len(values), 1024):
            q = values[offset:offset+1024]
            fk = self.checker.kinematics.compute_kinematics(
                JointState.from_position(q[None], joint_names=list(ACTIVE_JOINTS)))
            tools = fk.tool_poses.to_dict()
            all_spheres = fk.robot_spheres.reshape(len(q), -1, 4)
            spheres = all_spheres[:, self.robot_indices]
            rows = None if scale is None else scale[offset:offset+len(q)]
            if len(self.margin_pairs):
                first, second = all_spheres[:, self.margin_pairs[:, 0]], all_spheres[:, self.margin_pairs[:, 1]]
                clearance = (torch.linalg.vector_norm(first[..., :3]-second[..., :3], dim=-1)
                             - first[..., 3]-second[..., 3])
                valid[offset:offset+len(q)] &= ~((first[..., 3] > 0) & (second[..., 3] > 0)
                                                 & (clearance < self.margin_pair_values)).any(-1)
            if len(self.cross_pairs):
                valid[offset:offset+len(q)] &= ~self._cross_arm_clash(all_spheres, tools)
            if self.fixed_body is not None:
                valid[offset:offset+len(q)] &= ~self.fixed_body_collision(all_spheres, rows)
            if self.contact_pair is not None:
                side, object_id = self.contact_pair
                item = self.snapshot.object(object_id)
                matrix = torch.as_tensor(pose_matrix(item.pose), device=q.device, dtype=q.dtype)
                half = torch.tensor(item.dimensions_m, device=q.device, dtype=q.dtype)/2
                local = matmul3(all_spheres[..., :3]-matrix[:3, 3], matrix[:3, :3])
                distance = torch.linalg.vector_norm((local.abs()-half).clamp_min(0), dim=-1)
                radius = all_spheres[..., 3]
                if rows is not None:
                    radius = torch.where(rows[:, None] > 1., self.inflation.radii(radius), radius)
                hit = (all_spheres[..., 3] > 0) & (distance < radius-self.geometry_tolerance)
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
                overlap = (spheres[..., 3] > 0) & (distance < spheres[..., 3]+self.payload_growth-1e-6)
                own_link = item.parent_link.removesuffix("_tool0")+"_link7"
                own_indices = set(self.link_indices[own_link])
                own_columns = [i for i, sphere_index in enumerate(self.robot_indices) if sphere_index in own_indices]
                overlap[:, own_columns] = False
                good &= ~overlap.any(-1)
                grown = half if rows is None else half*rows[:, None, None]
                good &= ~world_box_overlaps(position, rotation, grown, *self.world_geometry(), tolerance=self.geometry_tolerance)
                if self.fixed_body is not None:
                    centers, rotations, halves = self.fixed_body_boxes
                    good &= ~world_box_overlaps(position, rotation, half, centers, rotations,
                                                halves+self.fixed_body.activation_m, tolerance=self.geometry_tolerance)
                for other_position, other_rotation, other_half in boxes:
                    good &= ~obb_overlap(position, rotation, half, other_position, other_rotation, other_half, tolerance=self.geometry_tolerance)
                if self.support_pose is not None and item.parent_link == self.support_side+"_tool0":
                    reference = torch.as_tensor(self.support_pose, device=q.device, dtype=q.dtype)
                    position_ok = torch.linalg.vector_norm(position-reference[:3, 3], dim=-1) <= .001
                    cosine = ((rotation*reference[:3, :3]).sum(dim=(-1, -2))-1.)/2
                    good &= position_ok & (cosine >= math.cos(math.radians(.5)))
                boxes.append((position, rotation, half))
                valid[offset:offset+len(q)] &= good
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
