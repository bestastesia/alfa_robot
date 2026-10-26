"""CPU exact-mesh audit for a frozen request. Does not replace the RRT or mutate its model."""
from itertools import combinations

import fcl
import numpy as np
import trimesh
import yourdfpy

from .adapter import pose_matrix
from .scene import Pose


def _mesh_geometry(mesh):
    # STL repeats vertices per face; exact welding restores convex detection without changing the surface.
    vertices, inverse = np.unique(mesh.vertices, axis=0, return_inverse=True)
    mesh = trimesh.Trimesh(vertices=vertices, faces=inverse[mesh.faces], process=False)
    # Use the installed FCL conversion already used by Trimesh; do not approximate mesh by OBB.
    return (trimesh.collision.mesh_to_convex(mesh) if mesh.is_convex
            else trimesh.collision.mesh_to_BVH(mesh))


class MeshStateChecker:
    def __init__(self, robot_config, snapshot, tasks, attachments):
        if snapshot.frame_id != "map" or snapshot.state.base_pose != Pose():
            raise ValueError("mesh sequence audit requires the fixed-base map request contract")
        self.robot = yourdfpy.URDF.load(robot_config['urdf_path'], load_meshes=False, build_scene_graph=True)
        self.snapshot, self.tasks = snapshot, set(f'wall_box_{i:02d}' for i in tasks.values())
        self.ignored = {frozenset((a, b)) for a, links in robot_config['self_collision_ignore'].items() for b in links}
        self.locked = dict(robot_config.get('lock_joints', {}))
        self.shapes = []
        for link in self.robot.link_map.values():
            for index, collision in enumerate(link.collisions):
                source = collision.geometry.mesh
                if source is None:
                    raise ValueError('pinned robot audit requires collision meshes')
                mesh = trimesh.load(source.filename, force='mesh', process=False)
                if source.scale is not None:
                    mesh.apply_scale(source.scale)
                self._add(f'{link.name}:{index}', 'robot', link.name, _mesh_geometry(mesh), mesh.bounds,
                          collision.origin, vertices=mesh.vertices)
        for item in snapshot.objects:
            self._add(item.object_id, 'world', None, fcl.Box(*item.dimensions_m),
                      np.array([-np.array(item.dimensions_m)/2, np.array(item.dimensions_m)/2]), pose_matrix(item.pose))
        for item in snapshot.meshes:
            mesh = trimesh.Trimesh(vertices=item.vertices_m, faces=item.faces, process=False)
            self._add(item.object_id, 'world', None, _mesh_geometry(mesh), mesh.bounds, pose_matrix(item.pose))
        for item in attachments:
            shape = self._add(item.object_id, 'payload', item.parent_link, fcl.Box(*item.dimensions_m),
                             np.array([-np.array(item.dimensions_m)/2, np.array(item.dimensions_m)/2]),
                             pose_matrix(item.tool_to_object))
            parent = next(j.parent for j in self.robot.joint_map.values() if j.child == item.parent_link)
            shape['touch'] = set(item.touch_links) | {item.parent_link, parent}
            # Match the frozen GPU representation, where payload spheres belong to link7.
            shape['effective_link'] = parent
        self.pairs = []
        for i, j in combinations(range(len(self.shapes)), 2):
            a, b = self.shapes[i], self.shapes[j]
            if a['kind'] == b['kind'] == 'world':
                continue
            if a['kind'] == b['kind'] == 'robot' and (a['link'] == b['link'] or frozenset((a['link'], b['link'])) in self.ignored):
                continue
            if {a['kind'], b['kind']} == {'robot', 'payload'}:
                robot, payload = (a, b) if a['kind'] == 'robot' else (b, a)
                if robot['link'] in payload['touch'] or frozenset((robot['link'], payload['effective_link'])) in self.ignored:
                    continue
            # Ground is checked as the policy's infinite plane, not only the fixture cuboid.
            if a['name'] == 'ground' or b['name'] == 'ground':
                continue
            self.pairs.append((i, j))
        self.request = fcl.CollisionRequest(num_max_contacts=1, enable_contact=True)

    def _add(self, name, kind, link, geometry, bounds, origin, vertices=None):
        bounds = np.asarray(bounds)
        item = {'name': name, 'kind': kind, 'link': link, 'object': fcl.CollisionObject(geometry),
                'geometry': geometry, 'center': bounds.mean(axis=0), 'half': (bounds[1]-bounds[0])/2,
                'origin': np.eye(4) if origin is None else np.asarray(origin), 'vertices': vertices}
        self.shapes.append(item)
        return item

    def check(self, positions, *, attached, payload_enabled):
        state = {**self.locked, **dict(zip(self.snapshot.state.joint_names, self.snapshot.state.positions)), **positions}
        missing = set(self.robot.actuated_joint_names)-state.keys()
        if missing:
            raise ValueError(f'missing mesh-audit joints: {sorted(missing)}')
        if not np.isfinite([state[name] for name in self.robot.actuated_joint_names]).all():
            raise ValueError("non-finite mesh-audit joint state")
        for name, value in self.locked.items():
            if abs(state.get(name, value)-value) > 1e-9:
                raise ValueError(f"state violates configured locked joint: {name}")
        for name in self.robot.actuated_joint_names:
            joint = self.robot.joint_map[name]
            if joint.type == 'continuous' or joint.limit is None:
                continue
            if state[name] < joint.limit.lower-1e-6 or state[name] > joint.limit.upper+1e-6:
                return {'joint_limit_violation': name, 'value': state[name]}
        self.robot.update_cfg({name: state[name] for name in self.robot.actuated_joint_names})
        link_poses = {name: self.robot.get_transform(name, self.robot.base_link) for name in {s['link'] for s in self.shapes if s['link']}}
        bounds, enabled = [], []
        for item in self.shapes:
            active = not (item['kind'] == 'payload' and not payload_enabled)
            if item['kind'] == 'world' and item['name'] in self.tasks:
                active = not (attached or self.snapshot.policy.exclude_task_objects_before_contact)
            enabled.append(active)
            transform = item['origin'] if item['link'] is None else link_poses[item['link']] @ item['origin']
            center = transform[:3, :3] @ item['center'] + transform[:3, 3]
            extent = np.abs(transform[:3, :3]) @ item['half']
            bounds.append((center-extent, center+extent))
            if not active:
                continue
            item['object'].setTransform(fcl.Transform(transform[:3, :3], transform[:3, 3]))
            if item['kind'] == 'robot' and item['link'] != self.snapshot.policy.ground_support_link:
                minimum_z = float((item['vertices'] @ transform[2, :3] + transform[2, 3]).min())
            elif item['kind'] == 'payload':
                minimum_z = float((center-extent)[2])
            else:
                continue
            if minimum_z < self.snapshot.policy.ground_z_m-1e-6:
                return {'pair': [item['kind']+':'+item['name'], 'world:ground'],
                        'ground_penetration_m': self.snapshot.policy.ground_z_m-minimum_z}
        bounds = np.asarray(bounds)
        for i, j in self.pairs:
            if not enabled[i] or not enabled[j]:
                continue
            if np.any(np.minimum(bounds[i, 1], bounds[j, 1])-np.maximum(bounds[i, 0], bounds[j, 0]) <= 0):
                continue
            result = fcl.CollisionResult()
            if fcl.collide(self.shapes[i]['object'], self.shapes[j]['object'], self.request, result):
                return {'pair': [self.shapes[k]['kind']+':'+self.shapes[k]['name'] for k in (i, j)],
                        'contact_depth_m': max((float(c.penetration_depth) for c in result.contacts), default=0.)}
        return None
