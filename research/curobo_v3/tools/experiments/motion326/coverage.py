#!/usr/bin/env python3
"""MOTION-326 step 2d: how much of each collision shape the GPU spheres leave uncovered.

Surface points of a shape are sampled in its own frame; a point's gap is its distance outside
the union of the link's collision spheres (0 if covered). Robot links use the URDF collision
meshes and the frozen sphere set; the payload uses the box and the 40-sphere box fit. Contact
points recorded by contact_points.py are mapped into the same frames to read the gap there.
"""
import argparse
import collections
import json
from pathlib import Path
import sys

import numpy as np
import trimesh
import yaml
import yourdfpy

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from v3_research_assets import ROOT


def gaps(points, centers, radii):
    distance = np.linalg.norm(points[:, None, :] - centers[None], axis=2) - radii[None]
    return np.maximum(distance.min(axis=1), 0.)


def describe(values):
    return dict(covered_fraction=float((values <= 1e-4).mean()), p50_mm=float(np.median(values) * 1000),
                p95_mm=float(np.percentile(values, 95) * 1000), max_mm=float(values.max() * 1000))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--links", default="arm_carriage,base_link,right_link5,left_link7,right_link7")
    parser.add_argument("--contacts", type=Path, help="contact-points.jsonl to read gaps at contact points")
    parser.add_argument("--samples", type=int, default=20000)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    fixed = yaml.safe_load((ROOT / "generated/v3_analytic_071cb95/alfa_v322_suction_final.yml").read_text())
    kinematics = fixed.get("robot_cfg", fixed)["kinematics"]
    spheres = kinematics["collision_spheres"]
    urdf = yourdfpy.URDF.load(kinematics["urdf_path"], load_meshes=False)
    report = dict(links={}, payload={}, contacts={})
    mesh_frames = {}
    for name in args.links.split(","):
        balls = [ball for ball in spheres.get(name, []) if ball["radius"] > 0]
        centers = np.array([ball["center"] for ball in balls]) if balls else np.zeros((0, 3))
        radii = np.array([ball["radius"] for ball in balls]) if balls else np.zeros(0)
        report["links"][name] = dict(spheres=len(balls), meshes={})
        for index, collision in enumerate(urdf.link_map[name].collisions):
            mesh = trimesh.load(collision.geometry.mesh.filename, force="mesh", process=False)
            origin = collision.origin if collision.origin is not None else np.eye(4)
            mesh_frames[f"{name}:{index}"] = (origin, centers, radii)
            points = trimesh.transform_points(trimesh.sample.sample_surface(mesh, args.samples, seed=3)[0], origin)
            values = gaps(points, centers, radii) if len(radii) else np.full(len(points), np.inf)
            worst = points[np.argsort(values)[-200:]]
            report["links"][name]["meshes"][index] = dict(**describe(values), worst_region_link_frame=dict(
                min=worst.min(axis=0).round(3).tolist(), max=worst.max(axis=0).round(3).tolist()))
    fit = json.loads((ROOT / "generated/v3_suction_v322_6bb184b/frozen_collision_model/box_voxel_edge_corner_final.json").read_text())
    box = trimesh.creation.box(extents=fit["dimensions_m"])
    points = trimesh.sample.sample_surface(box, args.samples, seed=3)[0]
    box_centers, box_radii = np.asarray(fit["centers"]), np.asarray(fit["radii"])
    values = gaps(points, box_centers, box_radii)
    half = np.asarray(fit["dimensions_m"]) / 2
    feature = (np.abs(np.abs(points) - half) <= 0.01).sum(axis=1)
    report["payload"] = dict(spheres=len(box_radii), all=describe(values),
                             **{name: describe(values[feature == count]) for name, count in (("face", 1), ("edge", 2), ("corner", 3))
                                if (feature == count).any()})
    if args.contacts:
        collected = collections.defaultdict(list)
        for line in args.contacts.read_text().splitlines():
            record = json.loads(line)
            for side in record["sides"]:
                kind, name = side["name"].split(":", 1)
                local = np.asarray(side["local"])
                if kind == "payload":
                    collected["payload"].append(float(gaps(local[None], box_centers, box_radii)[0]))
                elif kind == "robot" and name in mesh_frames:
                    origin, centers, radii = mesh_frames[name]
                    if len(radii):
                        link_point = (origin @ np.append(local, 1.))[:3]
                        collected[name].append(float(gaps(link_point[None], centers, radii)[0]))
        report["contacts"] = {name: describe(np.asarray(values)) | dict(n=len(values)) for name, values in collected.items()}
    args.output.write_text(json.dumps(report, indent=1) + "\n")
    for name, link in report["links"].items():
        for index, mesh in link["meshes"].items():
            print(f"{name}:{index} spheres {link['spheres']:2d} | covered {mesh['covered_fraction']:.2f} | gap p95 {mesh['p95_mm']:6.1f} max {mesh['max_mm']:6.1f} mm | worst region {mesh['worst_region_link_frame']}")
    print("payload:", json.dumps({k: (round(v['p95_mm'], 1), round(v['max_mm'], 1), round(v['covered_fraction'], 2)) for k, v in report['payload'].items() if isinstance(v, dict)}))
    for name, value in report["contacts"].items():
        print(f"gap at contact points {name:22s} n {value['n']:5d} | median {value['p50_mm']:5.1f} p95 {value['p95_mm']:5.1f} max {value['max_mm']:5.1f} mm")


if __name__ == "__main__":
    main()
