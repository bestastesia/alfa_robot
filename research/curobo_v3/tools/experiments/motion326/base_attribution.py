#!/usr/bin/env python3
"""MOTION-326 step 2e: why the planner misses contacts with the base.

The planner does not check arm or payload spheres against base_link spheres: BallFullCyclePlanner
ignores those pairs (replace_arm_base_pairs) and checks every sphere except those of base_link and
arm_carriage against four frozen base cuboids (BaseCuboidLayer). This script measures how the
cuboids cover the base_link collision meshes, and for every recorded base contact (the deepest
frame per run and pair from true_depth_inventory.py, plus optional extra frames) compares, in the
base_link frame:
  spheres_cuboids  partner spheres vs the cuboids (what the GPU checks; > 0 means accepted),
  spheres_mesh     partner spheres vs the real base meshes (< 0: a mesh base model would reject it),
  mesh_cuboids     partner mesh surface vs the cuboids (< 0: exact partner geometry would hit the cuboids),
  true_depth       mesh vs mesh (from the records).
The partner is an arm link (frozen spheres, collision meshes) or the carried box (payload spheres, box).
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

from curobo_core.adapter import pose_matrix
from curobo_core.contracts import PlannerAssets
from curobo_core.scene import SceneSnapshot
from experiments.motion326.cpu_spheres import ChainFK, base_cuboids, box_distance, sphere_groups
from v3_research_assets import ROOT


def link_meshes(urdf, link):
    meshes = []
    for collision in urdf.link_map[link].collisions:
        mesh = trimesh.load(collision.geometry.mesh.filename, force="mesh")
        if collision.geometry.mesh.scale is not None:
            mesh.apply_scale(collision.geometry.mesh.scale)
        mesh.apply_transform(collision.origin if collision.origin is not None else np.eye(4))
        meshes.append(mesh)
    return meshes


def cuboid_distance(points, boxes):
    return np.min([box_distance(points, pose, half) for _, pose, half in boxes], axis=0)


def coverage(base_meshes, boxes, samples):
    """Base mesh surface outside the cuboids (gap) and cuboid surface outside the mesh (excess), per mesh."""
    report = dict(meshes={}, cuboids={})
    union = trimesh.util.concatenate(base_meshes)
    for index, mesh in enumerate(base_meshes):
        points = trimesh.sample.sample_surface(mesh, samples, seed=3)[0]
        gap = np.maximum(cuboid_distance(points, boxes), 0.)
        bands = {}
        for low, high in ((-0.4, 0.13), (0.13, 0.3), (0.3, 0.5), (0.5, 0.8), (0.8, 1.2), (1.2, 1.8)):
            mask = (points[:, 2] >= low) & (points[:, 2] < high)
            if mask.any():
                bands[f"{low:+.2f}..{high:+.2f}"] = dict(points=int(mask.sum()), p95_mm=float(np.percentile(gap[mask], 95) * 1000),
                                                         max_mm=float(gap[mask].max() * 1000))
        worst = points[np.argsort(gap)[-200:]]
        report["meshes"][index] = dict(covered_fraction=float((gap <= 1e-4).mean()), p95_mm=float(np.percentile(gap, 95) * 1000),
                                       max_mm=float(gap.max() * 1000), z_bands=bands,
                                       worst_region=dict(low=worst.min(0).round(3).tolist(), high=worst.max(0).round(3).tolist()))
    for name, pose, half in boxes:
        box = trimesh.creation.box(extents=2 * half, transform=pose)
        points = trimesh.sample.sample_surface(box, samples // 4, seed=1)[0]
        excess = np.maximum(-trimesh.proximity.signed_distance(union, points), 0.)
        report["cuboids"][name] = dict(inside_mesh_fraction=float((excess <= 1e-4).mean()),
                                       excess_p95_mm=float(np.percentile(excess, 95) * 1000), excess_max_mm=float(excess.max() * 1000))
    return report


class Frames:
    """Link poses of recorded frames (root frame of the mobile URDF) and the payload of each run."""

    def __init__(self, directories):
        self.directories = directories
        config = yaml.safe_load((ROOT / "generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml").read_text())
        self.mobile = config["kinematics"]
        self.urdf = yourdfpy.URDF.load(self.mobile["urdf_path"], load_meshes=False)
        self.fk = ChainFK(self.urdf)
        self.results = {}

    def load(self, label, run):
        key = (label, run)
        if key not in self.results:
            result = json.loads((self.directories[label] / "full" / f"{run}.json").read_text())
            event = next(item for item in result["predicted_scene_events"] if item["phase"] == "attach")
            self.results[key] = (result, SceneSnapshot.from_dict(event["snapshot"]))
            if len(self.results) > 8:
                self.results.pop(next(iter(self.results)))
        return self.results[key]

    def poses(self, label, run, phase, index):
        result, snapshot = self.load(label, run)
        rows = [row for row, name in zip(result["frames"], result["phases"]) if name == phase]
        values = {**dict(self.mobile.get("lock_joints", {})), **dict(zip(snapshot.state.joint_names, snapshot.state.positions)),
                  **dict(zip(result["joint_names"], rows[index]))}
        values["base_x"] = values["base_x"] - float(result.get("base_advance_m", 0.))
        return {link: pose[0] for link, pose in self.fk.transforms(values, 1).items()}, snapshot


def partner_geometry(partner, poses, snapshot, fixed, box_fit, urdf, samples, cache):
    """Partner spheres (centers, radii) and surface points, both in the root frame."""
    if partner.startswith("payload:"):
        attachment = next(item for item in snapshot.attachments if item.object_id == partner[8:])
        side = attachment.parent_link.split("_")[0]
        group = next(g for g in sphere_groups(fixed, box_fit, snapshot.attachments, {}) if g[0] == "payload:" + side)
        link_pose = poses[group[1]]
        box_pose = poses[attachment.parent_link] @ pose_matrix(attachment.tool_to_object)
        box = trimesh.creation.box(extents=attachment.dimensions_m)
        points = trimesh.transform_points(trimesh.sample.sample_surface(box, samples, seed=5)[0], box_pose)
        centers, radii = group[2], group[3]
    else:
        link = partner.split(":")[1]
        if link not in cache:
            cache[link] = np.concatenate([trimesh.sample.sample_surface(mesh, samples, seed=5)[0] for mesh in link_meshes(urdf, link)])
        link_pose = poses[link]
        points = trimesh.transform_points(cache[link], link_pose)
        balls = [ball for ball in fixed["collision_spheres"][link] if ball["radius"] > 0]
        centers, radii = np.array([ball["center"] for ball in balls]), np.array([ball["radius"] for ball in balls])
    return trimesh.transform_points(centers, link_pose), radii, points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--set", nargs=2, action="append", metavar=("LABEL", "RUN_DIR"), required=True)
    parser.add_argument("--true-depth", type=Path, required=True, help="true-depth-records.jsonl (transport frames)")
    parser.add_argument("--extra", type=Path, help="JSONL of extra frames: set, run, phase, frame, pair, true_depth_mm")
    parser.add_argument("--samples", type=int, default=20000)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    assets = PlannerAssets()
    fixed = yaml.safe_load(assets.robot_config.read_text())["kinematics"]
    box_fit = json.loads(assets.box_fit.read_text())
    boxes = base_cuboids()
    frames = Frames({label: Path(path) for label, path in args.set})
    base_meshes = link_meshes(frames.urdf, "base_link")
    base_union = trimesh.util.concatenate(base_meshes)
    report = dict(coverage=coverage(base_meshes, boxes, args.samples), contacts=[])
    items = []
    for record in map(json.loads, args.true_depth.read_text().splitlines()):
        if record["set"] not in frames.directories:
            continue
        for pair, value in record["pairs"].items():
            if "robot:base_link" in pair and value["true_depth_mm"] > 0.:
                items.append(dict(set=record["set"], run=record["run"], arm=record["arm"], kind=record["kind"], phase="transport",
                                  frame=value["frame"], pair=pair, true_depth_mm=value["true_depth_mm"]))
    if args.extra:
        items += [json.loads(line) for line in args.extra.read_text().splitlines() if line.strip()]
    cache = {}
    for item in items:
        poses, snapshot = frames.poses(item["set"], item["run"], item["phase"], item["frame"])
        partner = next(name for name in item["pair"].split(" | ") if not name.startswith("robot:base_link"))
        centers, radii, points = partner_geometry(partner, poses, snapshot, fixed, box_fit, frames.urdf, 3000, cache)
        to_base = np.linalg.inv(poses["base_link"])
        centers, points = trimesh.transform_points(centers, to_base), trimesh.transform_points(points, to_base)
        mesh_inside = trimesh.proximity.signed_distance(base_union, centers)  # positive inside the base mesh
        deepest = points[np.argmin(cuboid_distance(points, boxes))]
        report["contacts"].append(dict(item, partner=partner,
                                       spheres_cuboids_mm=float((cuboid_distance(centers, boxes) - radii).min() * 1000),
                                       spheres_mesh_mm=float((-mesh_inside - radii).min() * 1000),
                                       mesh_cuboids_mm=float(cuboid_distance(points, boxes).min() * 1000),
                                       mesh_cuboids_point_base_frame=deepest.round(3).tolist()))
        print(json.dumps({k: (round(v, 1) if isinstance(v, float) else v) for k, v in report["contacts"][-1].items()
                          if k not in ("kind",)}), flush=True)
    summary = collections.defaultdict(list)
    for contact in report["contacts"]:
        partner = "payload" if contact["partner"].startswith("payload:") else contact["partner"].split(":")[1]
        group = "joint_rrt" if contact["arm"] == "joint_rrt" else "cartesian"
        summary[f"{contact['phase']} {group} {partner}"].append(contact)
    report["summary"] = {}
    for key, contacts in sorted(summary.items()):
        cuboid_cause = [c["spheres_mesh_mm"] < 0 for c in contacts]
        sphere_cause = [c["mesh_cuboids_mm"] < 0 for c in contacts]
        report["summary"][key] = dict(
            records=len(contacts), true_depth_max_mm=max(c["true_depth_mm"] for c in contacts),
            gpu_accepts=sum(c["spheres_cuboids_mm"] > 0 for c in contacts),
            caught_with_mesh_base=sum(cuboid_cause), caught_with_exact_partner=sum(sphere_cause),
            caught_by_neither=sum(not a and not b for a, b in zip(cuboid_cause, sphere_cause)),
            spheres_cuboids_median_mm=float(np.median([c["spheres_cuboids_mm"] for c in contacts])),
            mesh_cuboids_median_mm=float(np.median([c["mesh_cuboids_mm"] for c in contacts])))
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=1) + "\n")
    for index, mesh in report["coverage"]["meshes"].items():
        print(f"base mesh {index}: covered {mesh['covered_fraction']:.2f} | gap p95 {mesh['p95_mm']:.1f} max {mesh['max_mm']:.1f} mm "
              f"| worst {mesh['worst_region']}")
    for name, cuboid in report["coverage"]["cuboids"].items():
        print(f"{name}: surface inside mesh {cuboid['inside_mesh_fraction']:.2f} | excess p95 {cuboid['excess_p95_mm']:.1f} "
              f"max {cuboid['excess_max_mm']:.1f} mm")
    for key, row in report["summary"].items():
        print(f"{key:32s} records {row['records']:3d} max {row['true_depth_max_mm']:5.1f} mm | GPU accepts {row['gpu_accepts']} "
              f"| mesh base catches {row['caught_with_mesh_base']} | exact partner catches {row['caught_with_exact_partner']} "
              f"| neither {row['caught_by_neither']} | median sphere-cuboid clearance {row['spheres_cuboids_median_mm']:.1f}, "
              f"partner mesh into cuboids {row['mesh_cuboids_median_mm']:.1f} mm")


if __name__ == "__main__":
    main()
