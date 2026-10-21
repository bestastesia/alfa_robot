"""MOTION-326: CPU replica of the GPU sphere checks that a sphere patch can change.

The frozen model accepts every audited frame, so a patched model rejects a frame exactly when an
appended sphere overlaps a sphere of a non-ignored link (payload spheres count as link7, as in
loaded_robot), a world cuboid (ground excluded, as in to_curobo_scene), or the ground plane (the
base link is exempt, as in GpuValidity). Forward kinematics is vectorized over frames.
"""
import collections

import numpy as np
from scipy.spatial.transform import Rotation

from curobo_core.adapter import pose_matrix
from v3_batched_loaded_search import loaded_robot


class ChainFK:
    """Vectorized forward kinematics over N joint rows for every link of a yourdfpy URDF."""

    def __init__(self, robot):
        self.root = robot.base_link
        children = collections.defaultdict(list)
        for joint in robot.joint_map.values():
            if joint.mimic is not None:
                raise ValueError(f"mimic joint {joint.name} is not supported")
            children[joint.parent].append(joint)
        self.order, stack = [], [self.root]
        while stack:
            link = stack.pop()
            for joint in children[link]:
                self.order.append(joint)
                stack.append(joint.child)

    def transforms(self, values, count):
        """values: joint name -> scalar or (count,) array. Returns link -> (count, 4, 4) in the root frame."""
        result = {self.root: np.broadcast_to(np.eye(4), (count, 4, 4))}
        for joint in self.order:
            origin = joint.origin if joint.origin is not None else np.eye(4)
            local = np.broadcast_to(origin, (count, 4, 4)).copy()
            if joint.type in ("revolute", "continuous", "prismatic"):
                q = np.broadcast_to(np.asarray(values[joint.name], dtype=float), (count,))
                axis = np.asarray(joint.axis if joint.axis is not None else (1., 0., 0.), dtype=float)
                motion = np.broadcast_to(np.eye(4), (count, 4, 4)).copy()
                if joint.type == "prismatic":
                    motion[:, :3, 3] = q[:, None] * axis
                else:
                    motion[:, :3, :3] = Rotation.from_rotvec(q[:, None] * axis).as_matrix()
                local = local @ motion
            elif joint.type != "fixed":
                raise ValueError(f"unsupported joint type {joint.type}")
            result[joint.child] = result[joint.parent] @ local
        return result


def sphere_groups(kinematics, box_fit, attachments, added):
    """(label, link, centers in link frame, radii): frozen links, 'payload:<side>' and '<link>+' appended spheres."""
    loaded = loaded_robot({"kinematics": kinematics}, box_fit, attachments=attachments) if attachments else None
    groups = []
    for link, balls in kinematics["collision_spheres"].items():
        groups.append((link, link, *_arrays(balls)))
        if loaded is not None and link.endswith("_link7"):
            payload = loaded["kinematics"]["collision_spheres"][link][len(balls):]
            if payload:
                groups.append(("payload:" + link.split("_")[0], link, *_arrays(payload)))
    for link, balls in added.items():
        groups.append((link + "+", link, *_arrays(balls)))
    return groups


def _arrays(balls):
    kept = [ball for ball in balls if ball["radius"] > 0]
    return (np.array([ball["center"] for ball in kept], dtype=float).reshape(-1, 3),
            np.array([ball["radius"] for ball in kept], dtype=float))


def world(groups, poses):
    return {label: np.einsum("nij,kj->nki", poses[link][:, :3, :3], centers) + poses[link][:, None, :3, 3]
            for label, link, centers, _ in groups}


def appended_collisions(groups, centers, ignored, cuboids, ground_z, ground_exempt=("base_link",)):
    """Per frame: True where an appended sphere collides; attribution label -> [frames, max overlap mm]."""
    count = next(iter(centers.values())).shape[0]
    hit = np.zeros(count, dtype=bool)
    found = collections.defaultdict(lambda: [0, 0.])

    def record(key, overlap):
        best = overlap.reshape(count, -1).max(axis=1)
        frames = best > 0
        if frames.any():
            hit[:] |= frames
            found[key][0] += int(frames.sum())
            found[key][1] = max(found[key][1], float(best.max()) * 1000)

    for label, link, _, radii in (g for g in groups if g[0].endswith("+")):
        a = centers[label]
        for other, other_link, _, other_radii in groups:
            # payload groups carry link7 as their link, so the ignore table applies to them as on the GPU
            if other.endswith("+") or other_link == link or frozenset((link, other_link)) in ignored:
                continue
            distance = np.linalg.norm(a[:, :, None, :] - centers[other][:, None, :, :], axis=-1)
            record(f"{link} | {other}", radii[None, :, None] + other_radii[None, None, :] - distance)
        for name, pose, half in cuboids:
            local = (a - pose[:3, 3]) @ pose[:3, :3]
            outside = np.linalg.norm(np.maximum(np.abs(local) - half, 0.), axis=-1)
            inside = np.minimum(np.max(np.abs(local) - half, axis=-1), 0.)
            record(f"{link} | world:{name}", radii[None, :] - (outside + inside))
        if link not in ground_exempt:
            record(f"{link} | ground", ground_z - (a[..., 2] - radii[None, :]))
    return hit, {key: dict(frames=value[0], max_overlap_mm=value[1]) for key, value in found.items()}


def world_cuboids(snapshot):
    return [(item.object_id, pose_matrix(item.pose), np.asarray(item.dimensions_m) / 2)
            for item in snapshot.objects if item.object_id != "ground"]
