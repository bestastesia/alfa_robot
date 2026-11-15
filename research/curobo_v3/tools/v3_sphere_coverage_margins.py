#!/usr/bin/env python3
"""Report per-sphere coverage margins of a robot sphere model against its URDF collision meshes.

Writes, per link, the largest gap between the mesh and the union of its spheres, the fraction of
sampled surface outside every sphere and the per-sphere margins the door planner grows moving arms
by in free space (curobo_core.coverage_margins; MOTION-429). Read-only with respect to the model.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import yaml
import yourdfpy

from curobo_core.coverage_margins import link_samples, sphere_margins, spheres_sha256


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--robot-config', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--spacing-m', type=float, default=.0015)
    p.add_argument('--max-points', type=int, default=200000)
    p.add_argument('--margin-m', type=float, default=.002, help='added for the sampling resolution')
    p.add_argument('--seed', type=int, default=20261109)
    a = p.parse_args()
    kinematics = yaml.safe_load(a.robot_config.read_text())['kinematics']
    robot = yourdfpy.URDF.load(kinematics['urdf_path'], load_meshes=False, build_scene_graph=True)
    rng = np.random.default_rng(a.seed)
    links = {}
    for name, items in kinematics['collision_spheres'].items():
        spheres = np.array([[*item['center'], item['radius']] for item in items])
        points, sources = link_samples(robot.link_map[name], a.spacing_m, a.max_points, rng)
        margins, need = sphere_margins(points, spheres, a.margin_m)
        links[name] = dict(spheres=len(spheres), samples=len(points), meshes=sources,
                           max_gap_m=float(max(0., need.max())), uncovered_fraction=float((need > 0).mean()),
                           sphere_margins_m=[round(float(value), 6) for value in margins])
        print(f"{name:14s} spheres={len(spheres):3d} max_gap={links[name]['max_gap_m']*1000:6.1f}mm "
              f"uncovered={links[name]['uncovered_fraction']*100:5.1f}% margin max/median="
              f"{margins.max()*1000:.1f}/{np.median(margins)*1000:.1f}mm", flush=True)
    a.output.parent.mkdir(parents=True, exist_ok=True)
    a.output.write_text(json.dumps(dict(schema=1, robot_config=str(a.robot_config),
                                        robot_spheres_sha256=spheres_sha256(kinematics), spacing_m=a.spacing_m,
                                        max_points=a.max_points, margin_m=a.margin_m, seed=a.seed, links=links),
                                   indent=1, ensure_ascii=False)+'\n')


if __name__ == '__main__':
    main()
