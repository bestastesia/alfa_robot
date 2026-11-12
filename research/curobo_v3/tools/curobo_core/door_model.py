"""The door planner's robot model: asset root, joint coordinates, tool point and fixed-body geometry.

The URDF decides between the two frozen models:

* legacy (071cb95): tool0 at J7+151 mm and base_link as spheres; joint values are the analytic
  IK library's own;
* re-zeroed alfa_v3_curobo model of PR #80/#82 (``alfa_coordinate_revision`` =
  ``dual-zero-discrete-only-tool135-20261105``): q_library = offsets + signs*q
  (v3_zero_model_candidate), tool0 at J7+135 mm, and base_link as four frozen cuboids whose sphere
  slots are disabled (v3_fixed_body_boxes.disable_fixed_body_spheres).

An asset root is either the frozen legacy layout (relative paths of PlannerAssets) or holds a
DOOR_MODEL.json manifest naming each file (see tools/v3_door_rezero_assets.py).
"""
from dataclasses import dataclass, replace
import hashlib
import json
from pathlib import Path
from typing import Optional
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull
from scipy.spatial.transform import Rotation

from .contracts import WORKSPACE_ROOT, PlannerAssets

MANIFEST = 'DOOR_MODEL.json'
ASSET_NAMES = ('robot_config', 'mobile_robot_config', 'urdf', 'named_poses', 'box_fit')
MODEL_FILES = ('dynamics_root', 'recenter_atlas', 'fixed_body_boxes')
REZERO_REVISION = 'dual-zero-discrete-only-tool135-20261105'


@dataclass(frozen=True)
class DoorAssets(PlannerAssets):
    # None: the legacy files (generated/v3_dynamics_20261019, the placement policy's atlas, base spheres).
    dynamics_root: Optional[Path] = None
    recenter_atlas: Optional[Path] = None
    fixed_body_boxes: Optional[Path] = None


def load_assets(root):
    """DoorAssets of an asset root: its DOOR_MODEL.json manifest, else the frozen legacy layout."""
    root = Path(root)
    manifest = root/MANIFEST
    if not manifest.is_file():
        assets = DoorAssets()
        return replace(assets, **{name: root/getattr(assets, name).relative_to(WORKSPACE_ROOT) for name in ASSET_NAMES})
    files = json.loads(manifest.read_text())['files']
    return DoorAssets(**{name: root/files[name] for name in ASSET_NAMES + MODEL_FILES if name in files})


def asset_identity(assets):
    """sha256 of every model file the planner reads (directories: of each file, by relative path)."""
    output = {}
    for name in ASSET_NAMES + MODEL_FILES:
        path = getattr(assets, name, None)
        if path is None:
            continue
        if Path(path).is_dir():
            digest = {str(item.relative_to(path)): hashlib.sha256(item.read_bytes()).hexdigest()
                      for item in sorted(Path(path).rglob('*')) if item.is_file()}
            output[name] = dict(path=str(path), files=digest)
        else:
            output[name] = dict(path=str(path), sha256=hashlib.sha256(Path(path).read_bytes()).hexdigest())
    return output


def model_revision(urdf):
    """The URDF's joint coordinate revision: None for the legacy model; unknown revisions are refused."""
    revision = ET.parse(urdf).getroot().get('alfa_coordinate_revision')
    if revision not in (None, REZERO_REVISION):
        raise ValueError(f'unsupported joint coordinate revision: {revision}')
    return revision


def clip_at_tool_plane(vertices):
    """Hull vertices of the part of a convex tool hull (Tool0 frame) behind the TCP plane z = 0.

    In the 135 mm model the frozen tool mesh still reaches J7+151 mm. The 16 mm in front of the
    TCP plane is the suction cap, which may enter its own target box only (PR #80 tool_cap_policy:
    own target, sphere intersections on the forward side of the TCP plane omitted). Contact proofs
    therefore certify the hull behind the plane; every other check keeps the whole tool."""
    vertices = np.asarray(vertices, dtype=np.float64)
    z = vertices[:, 2]
    if z.max() <= 0.:
        return vertices
    pieces = [vertices[z <= 0.]]
    edges = {tuple(sorted(pair)) for simplex in ConvexHull(vertices).simplices
             for pair in ((simplex[0], simplex[1]), (simplex[1], simplex[2]), (simplex[0], simplex[2]))}
    for a, b in edges:
        if (z[a] > 0.) != (z[b] > 0.):
            pieces.append((vertices[a]+z[a]/(z[a]-z[b])*(vertices[b]-vertices[a]))[None])
    clipped = np.unique(np.concatenate(pieces), axis=0)
    return clipped[ConvexHull(clipped).vertices]


class FixedBodyBoxes:
    """PR #80 fixed body: base_link and its column as four frozen cuboids, in the planner's root frame.

    Robot spheres off base_link/arm_carriage and the exact payload boxes collide with a cuboid within
    `activation_m` (v3_base_cuboid_layer.BaseCuboidLayer: 5 mm activation distance); a cuboid collides
    with a world box beyond `world_tolerance_m` of SAT overlap (v3_fixed_body_boxes.FixedBodyWorld: 1 mm).
    In free-space search the cuboids' half extents are scaled like WorldMarginChecker.get_bound."""
    activation_m = .005
    world_tolerance_m = .001
    query_excluded_links = ('base_link', 'arm_carriage')

    def __init__(self, definition, urdf_model):
        data = json.loads(Path(definition).read_text())
        if data['frame'] != 'base_link' or not data['cuboids']:
            raise ValueError('fixed-body cuboids must be given in base_link')
        base = urdf_model.get_transform('base_link', urdf_model.base_link)  # a fixed joint
        centers, rotations, halves = [], [], []
        for item in data['cuboids']:
            local = np.eye(4)
            local[:3, :3] = Rotation.from_euler('xyz', item['rotation_deg'], degrees=True).as_matrix()
            local[:3, 3] = item['center']
            transform = base@local
            centers.append(transform[:3, 3])
            rotations.append(transform[:3, :3])
            halves.append(np.asarray(item['dimensions'], dtype=np.float64)/2)
        self.centers, self.rotations, self.halves = np.array(centers), np.array(rotations), np.array(halves)
        self.names = [item['name'] for item in data['cuboids']]
        self.sha256 = hashlib.sha256(Path(definition).read_bytes()).hexdigest()
