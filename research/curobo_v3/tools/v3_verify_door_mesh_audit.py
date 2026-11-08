#!/usr/bin/env python3
"""Independent all-frame exact-mesh audit of door rounds (FCL on URDF collision meshes, no spheres).

Each frame's scene is rebuilt from the round JSON alone: the world boxes and conveyor meshes of
its initial snapshot, minus the boxes attached in that frame, with released boxes at their release
poses (a symmetry-derived round reflects its source's); the base pose is the frame's own. Every
robot-link/world, robot self (minus the configured ignore pairs), attached-box/robot (minus touch
links) and attached-box/world pair is tested, plus the ground plane. A pair fails when FCL
reports a penetration deeper than --tolerance-m: designed suction contacts and box-on-box
tangency are certified by the planner within that tolerance. For non-convex (BVH) mesh pairs
FCL's depth is only a triangle-contact estimate; compare failing pairs against a reference run.
"""
import argparse
from itertools import combinations
import json
import math
from pathlib import Path
import time

import fcl
import numpy as np
import trimesh
import yaml
import yourdfpy

from curobo_core.adapter import pose_matrix
from curobo_core.mesh_validation import _mesh_geometry
from curobo_core.scene import Pose


def matrix(item):
    return pose_matrix(Pose(tuple(item['position']), tuple(item['quaternion_wxyz'])))


def corners(half):
    return np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])*half


class FrameAudit:
    def __init__(self, robot_config, tolerance):
        self.robot = yourdfpy.URDF.load(robot_config['urdf_path'], load_meshes=False, build_scene_graph=True)
        self.ignored = {frozenset((a, b)) for a, links in robot_config['self_collision_ignore'].items() for b in links}
        self.locked = dict(robot_config.get('lock_joints', {}))
        self.parents = {joint.child: joint.parent for joint in self.robot.joint_map.values()}
        self.tolerance = tolerance
        self.request = fcl.CollisionRequest(num_max_contacts=1, enable_contact=True)
        self.links = []
        for link in self.robot.link_map.values():
            for index, collision in enumerate(link.collisions):
                source = collision.geometry.mesh
                if source is None:
                    raise ValueError(f'robot audit requires collision meshes: {link.name}')
                mesh = trimesh.load(source.filename, force='mesh', process=False)
                if source.scale is not None:
                    mesh.apply_scale(source.scale)
                self.links.append(dict(name=f'{link.name}:{index}', link=link.name,
                                       object=fcl.CollisionObject(_mesh_geometry(mesh)), vertices=np.asarray(mesh.vertices),
                                       origin=np.eye(4) if collision.origin is None else np.asarray(collision.origin)))
        self.self_pairs = [(i, j) for i, j in combinations(range(len(self.links)), 2)
                           if self.links[i]['link'] != self.links[j]['link']
                           and frozenset((self.links[i]['link'], self.links[j]['link'])) not in self.ignored]
        self.world_cache = {}

    def world(self, key, objects, meshes, base):
        """FCL objects and AABBs of the world in the base frame (static between base moves)."""
        if key not in self.world_cache:
            inverse = np.linalg.inv(base)
            items = []
            for item in objects:
                if item['object_id'] == 'ground':
                    continue
                transform, half = inverse@matrix(item['pose']), np.asarray(item['dimensions_m'])/2
                shape = fcl.CollisionObject(fcl.Box(*item['dimensions_m']), fcl.Transform(transform[:3, :3], transform[:3, 3]))
                points = corners(half)@transform[:3, :3].T+transform[:3, 3]
                items.append(dict(name=item['object_id'], object=shape, low=points.min(0), high=points.max(0)))
            for item in meshes:
                transform = inverse@matrix(item['pose'])
                mesh = trimesh.Trimesh(vertices=item['vertices_m'], faces=item['faces'], process=False)
                shape = fcl.CollisionObject(_mesh_geometry(mesh), fcl.Transform(transform[:3, :3], transform[:3, 3]))
                points = np.asarray(item['vertices_m'])@transform[:3, :3].T+transform[:3, 3]
                items.append(dict(name=item['object_id'], object=shape, low=points.min(0), high=points.max(0)))
            if len(self.world_cache) > 64:
                self.world_cache.pop(next(iter(self.world_cache)))
            self.world_cache[key] = items
        return self.world_cache[key]

    def depth(self, first, second):
        if np.any(np.minimum(first['high'], second['high']) < np.maximum(first['low'], second['low'])):
            return None
        result = fcl.CollisionResult()
        if not fcl.collide(first['object'], second['object'], self.request, result):
            return None
        return max((float(contact.penetration_depth) for contact in result.contacts), default=0.)

    def check(self, positions, world, attachments, base, ground_z, support_link):
        state = {**self.locked, **positions}
        self.robot.update_cfg({name: state.get(name, 0.) for name in self.robot.actuated_joint_names})
        poses = {name: self.robot.get_transform(name, self.robot.base_link)
                 for name in {item['link'] for item in self.links} | {item['parent_link'] for item in attachments}}
        robot = []
        for item in self.links:
            transform = poses[item['link']]@item['origin']
            item['object'].setTransform(fcl.Transform(transform[:3, :3], transform[:3, 3]))
            points = item['vertices']@transform[:3, :3].T+transform[:3, 3]
            robot.append(dict(item, low=points.min(0), high=points.max(0)))
        payload = []
        for item in attachments:
            transform, half = poses[item['parent_link']]@matrix(item['tool_to_object']), np.asarray(item['dimensions_m'])/2
            shape = fcl.CollisionObject(fcl.Box(*item['dimensions_m']), fcl.Transform(transform[:3, :3], transform[:3, 3]))
            points = corners(half)@transform[:3, :3].T+transform[:3, 3]
            touch = set(item.get('touch_links', ())) | {item['parent_link'], self.parents[item['parent_link']]}
            payload.append(dict(name=item['object_id'], object=shape, low=points.min(0), high=points.max(0),
                                touch=touch, effective=self.parents[item['parent_link']]))
        contacts, failures = [], []

        def record(kind, first, second, value):
            if value is None:
                return
            entry = dict(kind=kind, pair=[first, second], depth_m=value)
            (failures if value > self.tolerance else contacts).append(entry)

        # Ground is the policy plane (base frame z equals map z for a planar base).
        floor = ground_z-base[2, 3]
        for item in robot:
            if item['link'] != support_link and item['low'][2] < floor-1e-6:
                failures.append(dict(kind='robot_ground', pair=[item['name'], 'ground'], depth_m=float(floor-item['low'][2])))
        for item in payload:
            if item['low'][2] < floor-1e-6:
                failures.append(dict(kind='payload_ground', pair=[item['name'], 'ground'], depth_m=float(floor-item['low'][2])))
        for item in robot:
            for other in world:
                record('robot_world', item['name'], other['name'], self.depth(item, other))
        for i, j in self.self_pairs:
            record('robot_self', robot[i]['name'], robot[j]['name'], self.depth(robot[i], robot[j]))
        for box in payload:
            for item in robot:
                if item['link'] in box['touch'] or frozenset((item['link'], box['effective'])) in self.ignored:
                    continue
                record('payload_robot', box['name'], item['name'], self.depth(box, item))
            for other in world:
                record('payload_world', box['name'], other['name'], self.depth(box, other))
        for first, second in combinations(payload, 2):
            record('payload_payload', first['name'], second['name'], self.depth(first, second))
        return contacts, failures


def mirrored_release_poses(result):
    """Release poses of a symmetry-derived round: its source's poses reflected across y = 0."""
    source = json.loads(Path(result['symmetry_source']).read_text())
    poses = {}
    for event in source['predicted_scene_events']:
        if event['phase'] == 'release' and event.get('world_pose'):
            index = int(event['object_id'].removeprefix('wall_box_'))
            (x, y, z), (w, qx, qy, qz) = event['world_pose']['position'], event['world_pose']['quaternion_wxyz']
            poses[f'wall_box_{index//5*5+4-index%5:02d}'] = dict(position=[x, -y, z], quaternion_wxyz=[w, -qx, qy, -qz])
    return poses


def round_scene(result):
    """World objects (map frame) at the round start, conveyor meshes, release poses and policy."""
    events = result['predicted_scene_events']
    initial = result.get('initial_snapshot')
    if initial is None:
        # Older reports: the first recorded snapshot plus the boxes attached at that event.
        event = next(event for event in events if event.get('snapshot'))
        initial = dict(event['snapshot'], objects=list(event['snapshot']['objects'])+[
            dict(object_id=item['object_id'], dimensions_m=item['dimensions_m'], pose=event['world_pose'])
            for item in event['snapshot']['attachments'] if item['object_id'] == event['object_id']])
    if initial.get('attachments'):
        raise ValueError('round must start without attachments')
    objects = {item['object_id']: item for item in initial['objects']}
    mirrored = mirrored_release_poses(result) if result.get('symmetry_source') else {}
    releases = [(int(event['frame_index']), event['object_id'], event.get('world_pose') or mirrored[event['object_id']])
                for event in events if event['phase'] == 'release']
    return objects, initial['meshes'], releases, initial['policy']


def audit_round(audit, result, stride):
    objects, meshes, releases, policy = round_scene(result)
    names = result['joint_names']
    frames = np.asarray(result['frames'])
    checked, contacts, failures = 0, {}, []
    for index in range(0, len(frames), stride):
        frame = frames[index]
        attachments = result['attachments_by_frame'][index]
        attached = {item['object_id'] for item in attachments}
        released = {object_id: pose for frame_index, object_id, pose in releases if frame_index <= index}
        present = [dict(item, pose=released.get(object_id, item['pose']))
                   for object_id, item in objects.items() if object_id not in attached]
        x, y, yaw = frame[:3]
        base = np.eye(4)
        base[:3, :3] = ((math.cos(yaw), -math.sin(yaw), 0.), (math.sin(yaw), math.cos(yaw), 0.), (0., 0., 1.))
        base[:2, 3] = x, y
        key = (tuple(np.round(frame[:3], 12)), tuple(sorted(attached)), tuple(sorted(released)))
        world = audit.world(key, present, meshes, base)
        positions = dict(zip(names[3:], frame[3:]))
        found, bad = audit.check(positions, world, attachments, np.eye(4), policy['ground_z_m'], policy['ground_support_link'])
        checked += 1
        for item in found:
            label = (item['kind'], *item['pair'])
            contacts[label] = max(contacts.get(label, 0.), item['depth_m'])
        failures.extend(dict(item, frame=index, phase=result['phases'][index]) for item in bad)
    summary = {}
    for item in failures:
        entry = summary.setdefault((item['kind'], *item['pair'], item['phase']), dict(frames=0, max_depth_m=0., first_frame=item['frame']))
        entry['frames'] += 1
        entry['max_depth_m'] = max(entry['max_depth_m'], item['depth_m'])
    return dict(checked_frames=checked, total_frames=len(frames), failures=failures[:50], failure_count=len(failures),
                failure_summary=[dict(kind=k[0], pair=list(k[1:3]), phase=k[3], **v) for k, v in sorted(summary.items())],
                tolerated_contacts=[dict(kind=k[0], pair=list(k[1:]), max_depth_m=v) for k, v in sorted(contacts.items())])


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--rounds', type=Path, nargs='+', required=True, help='round JSON files')
    p.add_argument('--robot-config', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--tolerance-m', type=float, default=1e-4)
    p.add_argument('--stride', type=int, default=1, help='audit every n-th frame (1 = all frames)')
    a = p.parse_args()
    audit = FrameAudit(yaml.safe_load(a.robot_config.read_text())['kinematics'], a.tolerance_m)
    output = dict(tolerance_m=a.tolerance_m, stride=a.stride, rounds=[])
    for path in a.rounds:
        began = time.perf_counter()
        result = json.loads(path.read_text())
        report = audit_round(audit, result, a.stride)
        report.update(path=str(path), column=result.get('column'), wall_s=time.perf_counter()-began)
        output['rounds'].append(report)
        print(json.dumps({k: report[k] for k in ('path', 'checked_frames', 'total_frames', 'failure_count', 'wall_s')}), flush=True)
    output['success'] = all(item['failure_count'] == 0 and item['checked_frames'] > 0 for item in output['rounds'])
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(output, ensure_ascii=False, indent=1)+'\n')
    print(json.dumps(dict(success=output['success'], rounds=len(output['rounds']))))
    return 0 if output['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
