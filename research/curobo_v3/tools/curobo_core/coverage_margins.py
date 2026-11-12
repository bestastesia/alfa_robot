"""Per-sphere coverage margins of a sphere model against its URDF collision meshes (MOTION-429).

The frozen MorphIt spheres trade coverage against protrusion: parts of every link's mesh lie
outside all of its spheres (arm links up to ~2 cm). A sphere's margin is the growth that makes it
contain every sampled mesh point assigned to it, an assignment that gives each point to the
sphere needing the least growth; growing all spheres of a link by their margins covers its mesh.
"""
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh


def spheres_sha256(kinematics):
    """Identity of the sphere model itself, independent of absolute paths in the robot YAML."""
    canonical = json.dumps(kinematics['collision_spheres'], sort_keys=True, separators=(',', ':'))
    return hashlib.sha256(canonical.encode()).hexdigest()


def link_samples(link, spacing, max_points, rng):
    """Surface samples (vertices plus area-uniform points) of a link's collision meshes, link frame."""
    points, sources = [], []
    for collision in link.collisions:
        source = collision.geometry.mesh
        if source is None:
            raise ValueError(f'coverage margins require collision meshes: {link.name}')
        mesh = trimesh.load(source.filename, force='mesh')
        if source.scale is not None:
            mesh.apply_scale(source.scale)
        origin = np.eye(4) if collision.origin is None else np.asarray(collision.origin)
        count = int(min(max_points, max(1000, mesh.area/spacing**2)))
        sampled, _ = trimesh.sample.sample_surface(mesh, count, seed=int(rng.integers(1 << 31)))
        local = np.vstack([mesh.vertices, sampled])
        points.append(local@origin[:3, :3].T+origin[:3, 3])
        sources.append(dict(path=Path(source.filename).name,
                            sha256=hashlib.sha256(Path(source.filename).read_bytes()).hexdigest(),
                            effective_spacing_m=float(np.sqrt(mesh.area/count))))
    return np.vstack(points), sources


def sphere_margins(points, spheres, extra):
    """Growth per sphere (plus `extra`) so every point lies in a grown sphere, and each point's least growth."""
    need = np.empty(len(points))
    owner = np.empty(len(points), dtype=int)
    for start in range(0, len(points), 65536):
        growth = (np.linalg.norm(points[start:start+65536, None, :]-spheres[None, :, :3], axis=-1)
                  - spheres[None, :, 3])
        owner[start:start+len(growth)] = growth.argmin(1)
        need[start:start+len(growth)] = growth.min(1)
    margins = np.zeros(len(spheres))
    np.maximum.at(margins, owner, np.clip(need, 0., None))
    return margins+extra, need


def coverage_margins(kinematics, urdf, links, spacing=.0015, max_points=200000, extra=.002, seed=20261109):
    """{link: per-sphere margins (m)} for `links`, aligned with kinematics['collision_spheres'][link]."""
    rng = np.random.default_rng(seed)
    output = {}
    for name in links:
        spheres = np.array([[*item['center'], item['radius']] for item in kinematics['collision_spheres'][name]])
        points, _ = link_samples(urdf.link_map[name], spacing, max_points, rng)
        output[name] = sphere_margins(points, spheres, extra)[0]
    return output
