#!/usr/bin/env python3
"""MOTION-326 step 3 (base): add-only cuboids for the base cuboid layer.

The planner checks arm and payload spheres against four frozen base cuboids, not against base_link
spheres (BallFullCyclePlanner, BaseCuboidLayer). The relevant base surface (cached by sphere_patch.py:
points that a carried box or arm links 4-7 come within 50 mm of in the audited transport frames,
mirrored in y) lying more than --tolerance outside the cuboid union is clustered; each cluster gets
one axis-aligned cuboid spanning its points and their closest points on the frozen cuboids, so the
new cuboid reaches from the mesh surface to the frozen model without a seam. With --max-excess, a
cuboid whose surface sticks out of the base mesh by more than that (P95) is split at the median of
its points along the axis that helps most, recursively (sloped faces become stairs). The frozen
cuboids are kept. Writes cuboids.json (frozen + added, the format BaseCuboidLayer reads) and patch.json (added
cuboids, coverage before/after, how far the added cuboids stick out of the base mesh).
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import connected_components
from scipy.spatial import cKDTree
import trimesh
import yaml
import yourdfpy

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from experiments.motion326.base_attribution import link_meshes
from experiments.motion326.cpu_spheres import base_cuboids, box_distance
from experiments.motion326.sphere_patch import MOBILE, describe, mirrored
from v3_research_assets import CUBOID_PATH


def closest_on_boxes(points, boxes):
    """Closest point on the nearest box (the point itself if inside one)."""
    distance = np.stack([box_distance(points, pose, half) for _, pose, half in boxes])
    nearest = distance.argmin(axis=0)
    result = points.copy()
    for index, (_, pose, half) in enumerate(boxes):
        mask = nearest == index
        local = np.clip((points[mask] - pose[:3, 3]) @ pose[:3, :3], -half, half)
        result[mask] = local @ pose[:3, :3].T + pose[:3, 3]
    return result


def union_gap(points, boxes):
    return np.maximum(np.min([box_distance(points, pose, half) for _, pose, half in boxes], axis=0), 0.)


def excess(low, high, union, samples):
    """P95 and max of how far the surface of the box [low, high] lies outside the base mesh (m)."""
    box = trimesh.creation.box(extents=np.maximum(high - low, 1e-3), transform=trimesh.transformations.translation_matrix((low + high) / 2))
    outside = np.maximum(-trimesh.proximity.signed_distance(union, trimesh.sample.sample_surface(box, samples, seed=1)[0]), 0.)
    return float(np.percentile(outside, 95)), float(outside.max())


def fit(members, frozen, union, max_excess, min_points, depth=0, max_depth=4):
    """Axis-aligned boxes [low, high] covering members down to the frozen cuboids, split while they stick out too far."""
    span = np.vstack([members, closest_on_boxes(members, frozen)])
    low, high = span.min(axis=0), span.max(axis=0)
    if max_excess is None or depth >= max_depth or len(members) < 2 * min_points or excess(low, high, union, 1000)[0] <= max_excess:
        return [(low, high, members)]
    best = None
    for axis in range(3):
        cut = np.median(members[:, axis])
        parts = [members[members[:, axis] <= cut], members[members[:, axis] > cut]]
        if min(len(part) for part in parts) < min_points:
            continue
        score = max(excess(*fit(part, frozen, union, None, min_points)[0][:2], union, 1000)[0] for part in parts)
        if best is None or score < best[0]:
            best = (score, parts)
    if best is None:
        return [(low, high, members)]
    return [box for part in best[1] for box in fit(part, frozen, union, max_excess, min_points, depth + 1, max_depth)]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--relevance", type=Path, required=True, help="relevance.npz written by sphere_patch.py")
    parser.add_argument("--tolerance", type=float, default=0.005, help="relevant points farther outside get covered (m)")
    parser.add_argument("--link-distance", type=float, default=0.03, help="clustering radius (m)")
    parser.add_argument("--spacing", type=float, default=0.008, help="surface sample spacing of the relevance cache (m)")
    parser.add_argument("--max-excess", type=float, help="split cuboids sticking out of the mesh by more than this (P95, m)")
    parser.add_argument("--min-points", type=int, default=10, help="smallest point group a split may leave")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    stored = np.load(args.relevance)
    points, flags = stored["base_link/points"], stored["base_link/flags"]
    relevant = mirrored(points, flags, args.spacing)
    frozen = base_cuboids()
    gap = union_gap(points, frozen)
    selected = np.flatnonzero(relevant & (gap > args.tolerance))
    pairs = cKDTree(points[selected]).query_pairs(args.link_distance, output_type="ndarray")
    count, labels = connected_components(coo_matrix((np.ones(len(pairs)), (pairs[:, 0], pairs[:, 1])),
                                                    shape=(len(selected), len(selected))), directed=False)
    urdf = yourdfpy.URDF.load(yaml.safe_load(MOBILE.read_text())["kinematics"]["urdf_path"], load_meshes=False)
    union = trimesh.util.concatenate(link_meshes(urdf, "base_link"))
    added = []
    for cluster in sorted(range(count), key=lambda c: -(labels == c).sum()):
        for low, high, members in fit(points[selected[labels == cluster]], frozen, union, args.max_excess, args.min_points):
            center, dimensions = (low + high) / 2, np.maximum(high - low, 1e-3)
            added.append(dict(name=f"base_patch_{len(added) + 1:03d}", center=center.round(4).tolist(),
                              dimensions=dimensions.round(4).tolist(), rotation_deg=[0., 0., 0.],
                              pose=[*center.round(4).tolist(), 1., 0., 0., 0.], dims=dimensions.round(4).tolist(),
                              relevant_points=int(len(members)), cluster=int(cluster),
                              gap_max_mm=float(union_gap(members, frozen).max() * 1000)))
    definition = json.loads(Path(CUBOID_PATH).read_text())
    definition["cuboids"] = definition["cuboids"] + [{key: item[key] for key in ("name", "center", "dimensions", "rotation_deg",
                                                                                  "pose", "dims")} for item in added]
    definition["patch"] = dict(source=str(CUBOID_PATH), added=[item["name"] for item in added], script="cuboid_patch.py")
    (args.output / "cuboids.json").write_text(json.dumps(definition, indent=1) + "\n")
    patched = base_cuboids(args.output / "cuboids.json")
    for item, (_, pose, half) in zip(added, patched[len(frozen):]):
        box = trimesh.creation.box(extents=2 * half, transform=pose)
        surface = trimesh.sample.sample_surface(box, 4000, seed=1)[0]
        excess = np.maximum(-trimesh.proximity.signed_distance(union, surface), 0.)
        item.update(excess_p95_mm=float(np.percentile(excess, 95) * 1000), excess_max_mm=float(excess.max() * 1000))
    after = union_gap(points, patched)
    report = dict(parameters={key: (str(value) if isinstance(value, Path) else value) for key, value in vars(args).items()},
                  frozen_cuboids=len(frozen), added_cuboids=added,
                  relevant_points=int(relevant.sum()), surface_points=int(len(points)),
                  relevant_gap_before=describe(gap[relevant]), relevant_gap_after=describe(after[relevant]),
                  surface_gap_before=describe(gap), surface_gap_after=describe(after))
    (args.output / "patch.json").write_text(json.dumps(report, indent=1) + "\n")
    for item in added:
        print(f"{item['name']}: center {item['center']} dims {item['dimensions']} | points {item['relevant_points']} "
              f"gap max {item['gap_max_mm']:.1f} mm | outside mesh p95 {item['excess_p95_mm']:.1f} max {item['excess_max_mm']:.1f} mm")
    print(f"relevant gap p95/max {report['relevant_gap_before']['p95_mm']:.1f}/{report['relevant_gap_before']['max_mm']:.1f} -> "
          f"{report['relevant_gap_after']['p95_mm']:.1f}/{report['relevant_gap_after']['max_mm']:.1f} mm; whole surface "
          f"{report['surface_gap_before']['p95_mm']:.1f}/{report['surface_gap_before']['max_mm']:.1f} -> "
          f"{report['surface_gap_after']['p95_mm']:.1f}/{report['surface_gap_after']['max_mm']:.1f} mm")


if __name__ == "__main__":
    main()
