"""MOTION-306 box-size scene parameter: one (depth x, width y, height z) triple per scene.

Every wall box of an instance has these dimensions (one load). The 5x5 task ids and cells are unchanged:
pitch = face + 1 cm gap, front faces at the wall front, bottom row on the ground. DEFAULT_BOX_DIMS
reproduces the legacy literals bit for bit. Box yaw is not part of this contract.
"""
import copy
from functools import lru_cache
import math

import numpy as np

DEFAULT_BOX_DIMS = (0.30, 0.40, 0.40)
BOX_GAP_M = 0.01
CONTAINER_HALF_WIDTH_M = 1.02
DEPTH_RANGE_M = (0.20, 0.50)
WIDTH_RANGE_M = (0.25, 0.40)  # 5 columns of 0.40 + 4 gaps fill the 2.04 m container
HEIGHT_RANGE_M = (0.25, 0.47)  # 5 rows + 4 gaps stay below the 2.40 m ceiling face
EXTRACTION_M = 0.35
EXTRACTION_MARGIN_M = 0.05
EXTRACTION_STEP_M = 0.01
# Inside the contact envelope of the current suction bar: box width >= 0.37 m for side and top grasp (adjacent
# fillers); single-arm pools self-collide at height <= 0.30 m and are sparse at 0.35 m, see
# motion306-m0a/size-envelope-00{1,2,3}.
SIZE_LEVELS = {
    "S30x40x40": (0.30, 0.40, 0.40),
    "S40x40x45": (0.40, 0.40, 0.45),
    "S30x37x40": (0.30, 0.37, 0.40),
    "S35x38x42": (0.35, 0.38, 0.42),
    "S40x37x45": (0.40, 0.37, 0.45),
}
SPHERE_MODEL_NAME = "cuboid_voxel_edge_corner_full_v1"
VOXEL_REFERENCE_SPHERES = 20  # voxel pitch (V / 20)^(1/3) of the frozen 6bb184b model


def box_dims(value=None):
    dims = DEFAULT_BOX_DIMS if value is None else tuple(float(item) for item in value)
    if len(dims) != 3 or not all(math.isfinite(item) for item in dims):
        raise ValueError("box dims must be three finite lengths in metres")
    depth, width, height = dims
    if not DEPTH_RANGE_M[0] - 1e-9 <= depth <= DEPTH_RANGE_M[1] + 1e-9:
        raise ValueError(f"box depth {depth} m outside {DEPTH_RANGE_M}")
    for name, value, bounds in (("width", width, WIDTH_RANGE_M), ("height", height, HEIGHT_RANGE_M)):
        if not bounds[0] - 1e-9 <= value <= bounds[1] + 1e-9:
            raise ValueError(f"box {name} {value} m outside {bounds}")
    return dims


def pitch(dims=DEFAULT_BOX_DIMS):
    _, width, height = box_dims(dims)
    return round(width + BOX_GAP_M, 6), round(height + BOX_GAP_M, 6)


def wall_center(front, distance, box_id, dims=DEFAULT_BOX_DIMS):
    depth, _, height = box_dims(dims)
    pitch_y, pitch_z = pitch(dims)
    return np.array([front + distance + depth / 2, (box_id % 5 - 2) * pitch_y, height / 2 + (box_id // 5) * pitch_z])


def extraction_distance_m(dims=DEFAULT_BOX_DIMS):
    """Straight pull that clears the box depth from its neighbours; 0.35 m unchanged up to 0.30 m deep."""
    needed = round(box_dims(dims)[0] + EXTRACTION_MARGIN_M, 6)
    return EXTRACTION_M if needed <= EXTRACTION_M else needed


def extraction_steps(dims=DEFAULT_BOX_DIMS):
    distance = extraction_distance_m(dims)
    return np.linspace(EXTRACTION_STEP_M, distance, int(round(distance / EXTRACTION_STEP_M)))


def _voxel_radius(dims):
    return 0.5 * (float(np.prod(dims)) / VOXEL_REFERENCE_SPHERES) ** (1 / 3)


EDGE_RADIUS_FRACTION = 0.021 / _voxel_radius(DEFAULT_BOX_DIMS)
CORNER_RADIUS_FRACTION = 0.0135 / _voxel_radius(DEFAULT_BOX_DIMS)


@lru_cache(maxsize=None)
def _sphere_model(dims):
    """Inscribed spheres: full voxel grid (radius = distance to nearest face), 12 edge midpoints, 8 corners.

    The frozen model keeps 20 of the 27 default grid voxels (seeded subset); the full grid is a superset.
    """
    half = np.asarray(dims) / 2
    voxel = _voxel_radius(dims)
    axes = []
    for size, extent in zip(dims, half):
        count = math.ceil(size / (2 * voxel) - 1e-9)
        axes.append([0.0] if count <= 1 else np.linspace(-(extent - voxel), extent - voxel, count).tolist())
    centers, radii, groups = [], [], []
    for x in axes[0]:
        for y in axes[1]:
            for z in axes[2]:
                center = np.array([x, y, z])
                centers.append(center)
                radii.append(float(np.min(half - np.abs(center))))
                groups.append("voxel")
    edge = voxel * EDGE_RADIUS_FRACTION
    for axis in range(3):
        first, second = [index for index in range(3) if index != axis]
        for sign_first in (-1, 1):
            for sign_second in (-1, 1):
                center = np.zeros(3)
                center[first] = sign_first * (half[first] - edge)
                center[second] = sign_second * (half[second] - edge)
                centers.append(center)
                radii.append(edge)
                groups.append("edge")
    corner = voxel * CORNER_RADIUS_FRACTION
    for sign_x in (-1, 1):
        for sign_y in (-1, 1):
            for sign_z in (-1, 1):
                centers.append(np.array([sign_x, sign_y, sign_z]) * (half - corner))
                radii.append(corner)
                groups.append("corner")
    model = dict(name=SPHERE_MODEL_NAME, description_commit="6bb184b", dimensions_m=list(dims),
                 parameters=dict(interior_method="full_voxel_grid", voxel_radius_m=voxel,
                                 voxel_reference_spheres=VOXEL_REFERENCE_SPHERES, edge_radius_m=edge,
                                 corner_radius_m=corner, require_inside_faces=True),
                 actual_spheres=len(radii), sphere_groups=groups,
                 centers=[[float(value) for value in center] for center in centers], radii=radii)
    model["metrics"] = sphere_model_metrics(model, samples=20000, seed=20261028)
    return model


def box_sphere_model(dims):
    return copy.deepcopy(_sphere_model(box_dims(dims)))


def sphere_model_metrics(model, samples=60000, seed=0):
    """Monte-Carlo volume coverage, protrusion and surface gaps, as reported by the frozen model."""
    half = np.asarray(model["dimensions_m"], dtype=float) / 2
    centers, radii = np.asarray(model["centers"], dtype=float), np.asarray(model["radii"], dtype=float)
    rng = np.random.default_rng(seed)
    points = rng.uniform(-half, half, (samples, 3))
    covered = np.zeros(samples, dtype=bool)
    for center, radius in zip(centers, radii):
        covered |= np.sum((points - center) ** 2, axis=1) <= radius * radius
    face = rng.integers(0, 6, samples)
    surface = rng.uniform(-half, half, (samples, 3))
    surface[np.arange(samples), face % 3] = np.where(face < 3, -1, 1) * half[face % 3]
    gap = np.full(samples, np.inf)
    for center, radius in zip(centers, radii):
        gap = np.minimum(gap, np.sqrt(np.sum((surface - center) ** 2, axis=1)) - radius)
    gap = np.maximum(gap, 0.)
    raw = np.max(np.abs(centers) + radii[:, None] - half, axis=1)
    protruding = np.where(raw > 1e-12, raw, 0.)  # face-touching spheres differ from the face by round-off only
    return dict(coverage=float(covered.mean()), protrusion=float(protruding.max()),
                protrusion_p95_mm=float(np.percentile(protruding, 95) * 1000),
                surface_gap_p95_mm=float(np.percentile(gap, 95) * 1000), max_gap_mm=float(gap.max() * 1000))
