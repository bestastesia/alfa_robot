from dataclasses import replace
import gzip
import json
from pathlib import Path

import numpy as np
from scipy.interpolate import BSpline
from scipy.spatial.transform import Rotation

from curobo_core.adapter import matrix_pose
from curobo_core.contracts import ACTIVE_JOINTS as JOINT_NAMES
from curobo_core.scene import SceneSnapshot
from v3_batched_loaded_search import GpuValidity


ROOT = Path(__file__).resolve().parents[1]
ACTIVE_JOINTS = list(JOINT_NAMES)


def read(path):
    path = Path(path)
    return json.loads(gzip.decompress(path.read_bytes()) if path.suffix == '.gz' else path.read_text())


def save(path, value):
    path = Path(path)
    data = json.dumps(value, ensure_ascii=False, indent=2).encode()
    path.write_bytes(gzip.compress(data) if path.suffix == '.gz' else data)


def stage_setup(planner, source, phase):
    planner.configure_task(source['task'])
    snapshot = SceneSnapshot.from_dict(source['visual_scene_snapshot'])
    positions = np.flatnonzero(np.asarray(source['phases']) == phase)
    first = max(0, int(positions[0]) - 1)
    full = np.asarray(source['frames'][first:positions[-1] + 1], dtype=np.float32)
    names = source['joint_names']
    if phase in ('conveyor_place', 'return_home'):
        base = np.eye(4)
        base[:3, :3] = Rotation.from_euler('z', float(full[0, names.index('base_yaw')])).as_matrix()
        base[:2, 3] = [full[0, names.index('base_x')] - source['visual_base_advance'], full[0, names.index('base_y')]]
        snapshot = replace(snapshot, state=snapshot.state.with_positions(dict(zip(names, full[0])), matrix_pose(base)))
    planner.snapshot = snapshot
    checker = planner.checker(source['task'], payload=phase in ('transport', 'conveyor_place'))
    validity = planner.target_validity(checker) if phase == 'approach' else GpuValidity(checker, check_ground=True)
    validity.scene_penetration_tolerance_m = .001
    validity.contact_penetration_tolerance_m = .001
    return full, [names.index(name) for name in ACTIVE_JOINTS], validity, checker.guidance_scene


def fit_path(segment, count):
    positions = np.asarray(segment['positions'])
    if count < 4 or positions.ndim != 2 or len(positions) < 2 or not np.isfinite(positions).all():
        raise ValueError('Expected finite path with at least two rows and four controls')
    arc = np.r_[0, np.cumsum(np.linalg.norm(np.diff(positions, axis=0), axis=1))]
    if arc[-1] < 1e-12:
        positions = positions[[0, -1]]
        arc = np.array([0., 1.])
    else:
        arc, indices = np.unique(arc, return_index=True)
        arc /= arc[-1]
        positions = positions[indices]
    grid = np.linspace(0, 1, 4097)
    reference = np.stack([np.interp(grid, arc, positions[:, joint]) for joint in range(positions.shape[1])], axis=1)
    knots = np.arange(-3, count + 6) / (count + 2)
    basis = BSpline(knots, np.eye(count + 5), 3)(grid)
    boundary = basis[:, :3].sum(axis=1)[:, None] * positions[0] + basis[:, -3:].sum(axis=1)[:, None] * positions[-1]
    inner = np.linalg.lstsq(basis[:, 3:-3], reference - boundary, rcond=None)[0]
    coefficients = np.vstack([np.tile(positions[0], (3, 1)), inner, np.tile(positions[-1], (3, 1))])
    fitted = basis @ coefficients
    report = dict(arm_max_deviation_deg=float(np.rad2deg(abs(fitted[:, 1:] - reference[:, 1:]).max())),
                  updown_max_deviation_mm=float(abs(fitted[:, 0] - reference[:, 0]).max() * 1000))
    return np.vstack([inner, positions[-1]]), positions[0], positions[-1], report, BSpline(knots, coefficients, 3)
