#!/usr/bin/env python3
"""CPU-only audit of link-local mesh reuse and coverage by the frozen robot spheres."""
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh
import yaml
import yourdfpy


def meshes(link):
    parts, signatures = [], []
    for collision in link.collisions:
        geometry = collision.geometry.mesh
        if geometry is None:
            raise ValueError('this audit expects the pinned mesh-based description')
        path = Path(geometry.filename)
        mesh = trimesh.load(path, force='mesh', process=False)
        if geometry.scale is not None:
            mesh.apply_scale(geometry.scale)
        mesh.apply_transform(collision.origin)
        parts.append(mesh)
        signatures.append({'file_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                           'origin': collision.origin.tolist(),
                           'scale': None if geometry.scale is None else list(geometry.scale)})
    return parts, signatures


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--runtime-root', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    root = args.runtime_root
    analytic_cfg = yaml.safe_load((root/'generated/v3_analytic_071cb95/alfa_v322_suction_final.yml').read_text())['kinematics']
    frozen_cfg = yaml.safe_load((root/'generated/v3_suction_v322_6bb184b/frozen_collision_model/alfa_v322_suction_final.yml').read_text())['kinematics']
    analytic = yourdfpy.URDF.load(analytic_cfg['urdf_path'], load_meshes=False)
    frozen = yourdfpy.URDF.load(frozen_cfg['urdf_path'], load_meshes=False)
    assert analytic_cfg['collision_spheres'] == frozen_cfg['collision_spheres'], 'sphere tables differ'
    report = {'scope': 'link-local mesh/coordinate reuse and vertex samples, NOT solid-volume coverage or trajectory acceptance',
              'robot_spheres_identical_to_frozen_6bb184b': True, 'links': {}}
    for name, rows in analytic_cfg['collision_spheres'].items():
        new_parts, new_signature = meshes(analytic.link_map[name])
        old_parts, old_signature = meshes(frozen.link_map[name])
        assert new_signature == old_signature, f'link-local meshes/origins/scales differ: {name}'
        vertices = np.vstack([m.vertices for m in new_parts])
        active = [s for s in rows if s['radius'] > 0]
        centers = np.array([s['center'] for s in active]); radii = np.array([s['radius'] for s in active])
        gap = (np.linalg.norm(vertices[:, None, :]-centers[None, :, :], axis=2)-radii[None, :]).min(axis=1)
        report['links'][name] = {'mesh_parts': len(new_parts), 'loaded_vertices': len(vertices), 'spheres': len(active),
            'link_local_geometry_identical': True, 'vertex_uncovered_fraction': float(np.mean(gap > 1e-6)),
            'vertex_gap_p95_m': float(np.quantile(gap, .95)), 'maximum_vertex_gap_m': float(gap.max()),
            'mesh_bounds': [vertices.min(axis=0).tolist(), vertices.max(axis=0).tolist()],
            'sphere_union_aabb': [(centers-radii[:, None]).min(axis=0).tolist(), (centers+radii[:, None]).max(axis=0).tolist()],
            'mesh_sources': new_signature}
    report['total_spheres'] = sum(x['spheres'] for x in report['links'].values())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    for name in ('arm_carriage', 'left_link5', 'base_link', 'right_link2', 'right_link4', 'right_link7'):
        r = report['links'][name]
        print(name, 'parts',r['mesh_parts'],'spheres',r['spheres'],'vertex_gap_max_mm',round(r['maximum_vertex_gap_m']*1000,3),
              'outside_vertex_fraction',round(r['vertex_uncovered_fraction'],3))
    print('PASS: all robot sphere tables and link-local meshes/origins/scales reused consistently; coverage is approximate.')


if __name__ == '__main__':
    main()
