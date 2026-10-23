"""MOTION-326: CPU replica of the GPU sphere checks that a sphere patch can change.

The frozen model accepts every audited frame, so a patched model rejects a frame exactly when an
appended sphere overlaps a sphere of a non-ignored link (payload spheres count as link7, as in
loaded_robot), a world cuboid (ground excluded, as in to_curobo_scene), the ground plane (the
base link is exempt, as in GpuValidity) or a base cuboid. The ignore table is the planner's:
BallFullCyclePlanner skips the arm <-> base_link sphere pairs (replace_arm_base_pairs) and checks
every sphere except those of base_link and arm_carriage against the frozen base cuboids instead
(BaseCuboidLayer). Forward kinematics is vectorized over frames.

Not replicated: in approach the GPU target validity also treats the box to be grasped as an obstacle,
while the world cuboids here come from the attach snapshot. GPU-verified (patch_eval agree): appended
sphere vs sphere (arm_carriage patch) and appended base cuboids vs all spheres (cuboid-v2); the
appended-sphere vs world/base cuboid paths have not been exercised on the GPU yet.
"""
import collections
import copy
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from curobo_core.adapter import pose_matrix
from v3_base_mesh_layer import replace_arm_base_pairs
from v3_batched_loaded_search import loaded_robot
from v3_research_assets import CUBOID_PATH

BASE_CUBOID_EXEMPT = ("base_link", "arm_carriage")  # BaseMeshLayer.excluded_links


def planner_ignored(kinematics):
    """Link pairs the planner's self-collision check skips: the configured table plus arm <-> base_link."""
    robot = {"kinematics": copy.deepcopy(kinematics)}
    replace_arm_base_pairs(robot)
    return {frozenset((a, b)) for a, links in robot["kinematics"]["self_collision_ignore"].items() for b in links}


def base_cuboids(path=CUBOID_PATH):
    """Frozen base cuboids as (name, pose in the base_link frame, half extents)."""
    result = []
    for item in json.loads(Path(path).read_text())["cuboids"]:
        pose = np.eye(4)
        pose[:3, :3] = Rotation.from_euler("xyz", item["rotation_deg"], degrees=True).as_matrix()
        pose[:3, 3] = item["center"]
        result.append((item["name"], pose, np.asarray(item["dimensions"], dtype=float) / 2))
    return result


def box_distance(points, pose, half):
    """Signed distance of points (..., 3) to a box with the given pose and half extents (negative inside)."""
    local = np.abs((points - pose[:3, 3]) @ pose[:3, :3]) - half
    return np.linalg.norm(np.maximum(local, 0.), axis=-1) + np.minimum(np.max(local, axis=-1), 0.)


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


def appended_collisions(groups, centers, ignored, cuboids, ground_z, ground_exempt=("base_link",), base=None,
                        added_base=None):
    """Per frame: True where an appended sphere or cuboid collides; attribution label -> [frames, max overlap mm].

    base: (base_link poses (N, 4, 4) in the root frame, base_cuboids()) to check appended spheres against the
    base cuboid layer; added_base: (the same poses, cuboids appended to the layer) to check every sphere against.
    """
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
            # payload groups carry link7 as their link, so the ignore table applies to them as on the GPU;
            # a pair of appended groups on two links is checked once
            if other_link == link or frozenset((link, other_link)) in ignored or (other.endswith("+") and other <= label):
                continue
            distance = np.linalg.norm(a[:, :, None, :] - centers[other][:, None, :, :], axis=-1)
            record(f"{link} | {other}", radii[None, :, None] + other_radii[None, None, :] - distance)
        for name, pose, half in cuboids:
            record(f"{link} | world:{name}", radii[None, :] - box_distance(a, pose, half))
        if base is not None and link not in BASE_CUBOID_EXEMPT:
            poses, boxes = base
            relative = np.einsum("nji,nkj->nki", poses[:, :3, :3], a - poses[:, None, :3, 3])  # base_link frame
            for name, pose, half in boxes:
                record(f"{link} | base:{name}", radii[None, :] - box_distance(relative, pose, half))
        if link not in ground_exempt:
            record(f"{link} | ground", ground_z - (a[..., 2] - radii[None, :]))
    if added_base is not None:
        poses, boxes = added_base
        for label, link, _, radii in groups:
            if link in BASE_CUBOID_EXEMPT or not len(radii):
                continue
            relative = np.einsum("nji,nkj->nki", poses[:, :3, :3], centers[label] - poses[:, None, :3, 3])
            for name, pose, half in boxes:
                record(f"{label} | base+:{name}", radii[None, :] - box_distance(relative, pose, half))
    return hit, {key: dict(frames=value[0], max_overlap_mm=value[1]) for key, value in found.items()}


def added_base_cuboids(path):
    """Cuboids that cuboid_patch.py appended to a base cuboid file (listed in its "patch" entry)."""
    added = set(json.loads(Path(path).read_text())["patch"]["added"])
    return [item for item in base_cuboids(path) if item[0] in added]


def world_cuboids(snapshot):
    return [(item.object_id, pose_matrix(item.pose), np.asarray(item.dimensions_m) / 2)
            for item in snapshot.objects if item.object_id != "ground"]
