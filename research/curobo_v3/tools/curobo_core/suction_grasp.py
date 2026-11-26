"""CPU geometry for a planar suction contact and the existing tool collision proxy."""
from functools import lru_cache
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[2]
SIDE_FACES = {'x-': (0, -1, 0.), 'x+': (0, 1, 180.), 'y-': (1, -1, 90.), 'y+': (1, 1, -90.)}


def tool_in_box(dimensions, top, offset=(0., 0., 0.), roll_deg=0., side_face='x-'):
    """TCP pose in the original axis-aligned box; offset is tangential in box XYZ."""
    result = np.eye(4)
    result[:3, :3] = np.diag([-1., 1., -1.]) if top else [[0., 0., 1.], [0., 1., 0.], [-1., 0., 0.]]
    if top:
        result[:3, 3] = [0., 0., dimensions[2]/2]
    else:
        axis, sign, yaw = SIDE_FACES[side_face]
        result[axis, 3] = sign*dimensions[axis]/2
        result[:3, :3] = Rotation.from_euler('z', yaw, degrees=True).as_matrix()@result[:3, :3]
    result[:3, 3] += offset
    result[:3, :3] = result[:3, :3]@Rotation.from_euler('z', roll_deg, degrees=True).as_matrix()
    return result


@lru_cache(maxsize=2)
def contact_cap_spheres(side):
    """Same cap selection as TaskCyclePlanner; collision proxy, not seal certification."""
    import yaml
    config = yaml.safe_load((ROOT/'generated/fast_runtime_assets/fixed.yaml').read_text())['kinematics']
    tree = ET.parse(ROOT/'generated/fast_runtime_assets/fixed.urdf').getroot()
    origin = np.fromstring(tree.find(f"joint[@name='{side}_tool0_fixed']/origin").get('xyz'), sep=' ')
    caps = [sphere for sphere in config['collision_spheres'][side+'_link7']
            if sphere['radius'] > 0 and sphere['center'][2]+sphere['radius'] > origin[2]]
    if not caps:raise ValueError('no actual tool cap collision proxy available')
    return np.array([[*(np.array(s['center'])-origin), s['radius']] for s in caps])


def validate_grasp(dimensions, top, side, offset, roll_deg, side_face='x-'):
    if side_face not in SIDE_FACES:
        raise ValueError('side suction face must be x-/x+/y-/y+')
    normal_axis = 2 if top else SIDE_FACES[side_face][0]
    offset = np.asarray(offset, dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all() or not np.isfinite(roll_deg):
        raise ValueError('suction offset and roll must be finite')
    if abs(offset[normal_axis]) > 1e-12:
        raise ValueError('suction offset must lie in the selected physical face')
    if np.all(offset == 0.) and roll_deg == 0. and side_face == 'x-':
        return tuple(offset)  # Preserve the exact legacy centered contact and its established proxy policy.
    tool = tool_in_box(dimensions, top, offset, roll_deg, side_face)
    # ponytail: conservative cap-sphere footprint; use measured suction-pad geometry if valid grasps are rejected.
    caps = contact_cap_spheres(side)
    centers = caps[:, :3]@tool[:3, :3].T+tool[:3, 3]
    tangent = [i for i in range(3) if i != normal_axis]
    half = np.asarray(dimensions)[tangent]/2
    if np.any(np.abs(centers[:, tangent])+caps[:, 3, None] > half+1e-9):
        raise ValueError('full tool cap collision proxy footprint extends beyond the suction face')
    return tuple(offset)
