#!/usr/bin/env python3
"""MOTION-326 step 3: add-only collision spheres for the arm carriage and the base link.

The relevant surface of a patched link is the part that a moving body (carried box, arm links 4-7
of either arm) comes within --near of in the audited transport frames, mirrored in y because the
robot is left/right symmetric. Sphere candidates sit inside the link's watertight collision meshes
(surface points pushed inward along the normal); radius = depth below the surface + --protrusion,
so no added sphere sticks out of the mesh by more than --protrusion. Spheres are added greedily
until every relevant point lies within --tolerance of the sphere union or --budget is reached.
The frozen spheres are kept; patched copies of both robot configs are written to --output.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np
from scipy.spatial import cKDTree
import trimesh
import yaml
import yourdfpy

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from curobo_core.adapter import pose_matrix
from curobo_core.scene import SceneSnapshot
from experiments.motion326.coverage import gaps
from v3_research_assets import ROOT

FIXED = ROOT / "generated/v3_analytic_071cb95/alfa_v322_suction_final.yml"
MOBILE = ROOT / "generated/v3_analytic_071cb95/mobile18/alfa_v322_suction_mobile18.yml"
MOVING_LINKS = tuple(f"{side}_link{index}" for side in ("left", "right") for index in range(4, 8))
DEPTHS_M = (0.005, 0.01, 0.02, 0.04, 0.08, 0.16)


def link_meshes(robot, name):
    """Welded collision meshes of a link, in the link frame."""
    meshes = []
    for collision in robot.link_map[name].collisions:
        mesh = trimesh.load(collision.geometry.mesh.filename, force="mesh", process=False)
        if collision.geometry.mesh.scale is not None:
            mesh.apply_scale(collision.geometry.mesh.scale)
        vertices, inverse = np.unique(mesh.vertices, axis=0, return_inverse=True)
        mesh = trimesh.Trimesh(vertices=vertices, faces=inverse[mesh.faces], process=False)
        meshes.append(mesh.apply_transform(collision.origin if collision.origin is not None else np.eye(4)))
    return meshes


def surface(meshes, spacing, seed):
    points, normals = [], []
    for mesh in meshes:
        samples, faces = trimesh.sample.sample_surface(mesh, max(16, int(mesh.area / spacing ** 2)), seed=seed)
        points.append(samples)
        normals.append(mesh.face_normals[faces])
    return np.vstack(points), np.vstack(normals)


class Body:
    """Surface samples of a rigid body in its own frame, for distance queries from another frame."""

    def __init__(self, meshes, spacing):
        self.points, _ = surface(meshes, spacing, seed=5)
        self.tree = cKDTree(self.points)
        self.center = (self.points.max(axis=0) + self.points.min(axis=0)) / 2
        self.half = (self.points.max(axis=0) - self.points.min(axis=0)) / 2

    def mark(self, relative, points, flags, near):
        """relative: body frame -> target frame. Sets flags of target points within `near` of the body."""
        middle = relative[:3, :3] @ self.center + relative[:3, 3]
        extent = np.abs(relative[:3, :3]) @ self.half + near
        indices = np.flatnonzero(np.all(np.abs(points - middle) <= extent, axis=1))
        if len(indices):
            distance, _ = self.tree.query((points[indices] - relative[:3, 3]) @ relative[:3, :3], distance_upper_bound=near)
            flags[indices[np.isfinite(distance)]] = True


def relevance(robot, config, sets, links, spacing, near, stride):
    targets = {}
    for name in links:
        points, normals = surface(link_meshes(robot, name), spacing, seed=11)
        targets[name] = (points, normals, np.zeros(len(points), dtype=bool))
    bodies = {name: Body(link_meshes(robot, name), spacing) for name in MOVING_LINKS}
    boxes = {}
    locked = dict(config.get("lock_joints", {}))
    for label, directory in sets:
        directory = Path(directory)
        for run in json.loads((directory / "fcl-audit.json").read_text())["runs"]:
            result = json.loads((directory / "full" / f"{run['run']}.json").read_text())
            event = next(item for item in result["predicted_scene_events"] if item["phase"] == "attach")
            snapshot = SceneSnapshot.from_dict(event["snapshot"])
            payload = []
            for item in snapshot.attachments:
                key = tuple(np.round(item.dimensions_m, 4))
                if key not in boxes:
                    boxes[key] = Body([trimesh.creation.box(extents=item.dimensions_m)], spacing)
                payload.append((item.parent_link, pose_matrix(item.tool_to_object), boxes[key]))
            base = {**locked, **dict(zip(snapshot.state.joint_names, snapshot.state.positions))}
            advance = float(result.get("base_advance_m", 0.))
            rows = [row for row, phase in zip(result["frames"], result["phases"]) if phase == "transport"]
            for row in rows[::stride]:
                state = {**base, **dict(zip(result["joint_names"], row))}
                state["base_x"] -= advance
                robot.update_cfg({name: state[name] for name in robot.actuated_joint_names})
                poses = {name: robot.get_transform(name, robot.base_link) for name in (*links, *MOVING_LINKS,
                                                                                       *(p for p, _, _ in payload))}
                for name, (points, _, flags) in targets.items():
                    inverse = np.linalg.inv(poses[name])
                    for moving, body in bodies.items():
                        body.mark(inverse @ poses[moving], points, flags, near)
                    for parent, tool_to_box, body in payload:
                        body.mark(inverse @ poses[parent] @ tool_to_box, points, flags, near)
            print(f"{label} {run['run']}: " + ", ".join(f"{n} {int(f.sum())}" for n, (_, _, f) in targets.items()), flush=True)
    return targets


def mirrored(points, flags, spacing):
    distance, index = cKDTree(points).query(points[flags] * np.array([1., -1., 1.]), distance_upper_bound=1.5 * spacing)
    result = flags.copy()
    result[index[np.isfinite(distance)]] = True
    return result


def inside_depth(meshes, points):
    return np.max([trimesh.proximity.signed_distance(mesh, points) for mesh in meshes if mesh.is_watertight], axis=0)


def greedy(meshes, points, normals, centers, radii, tolerance, protrusion, budget, min_gain):
    remaining = gaps(points, centers, radii) > tolerance
    seeds = np.flatnonzero(remaining)
    if not len(seeds):
        return []
    candidates = (points[seeds, None, :] - normals[seeds, None, :] * np.asarray(DEPTHS_M)[None, :, None]).reshape(-1, 3)
    depth = inside_depth(meshes, candidates)
    candidates, depth = candidates[depth > 1e-3], depth[depth > 1e-3]
    radius = depth + protrusion
    cover = [np.asarray(item, dtype=int) for item in cKDTree(points).query_ball_point(candidates, radius + tolerance)]
    added = []
    for _ in range(budget):
        gain = np.array([remaining[item].sum() for item in cover])
        best = int(gain.argmax())
        if gain[best] < min_gain:
            break
        remaining[cover[best]] = False
        added.append(dict(center=candidates[best].round(4).tolist(), radius=round(float(radius[best]), 4),
                          depth_m=round(float(depth[best]), 4), covered_points=int(gain[best])))
    return added


def describe(values):
    return dict(n=int(len(values)), p50_mm=float(np.median(values) * 1000), p95_mm=float(np.percentile(values, 95) * 1000),
                max_mm=float(values.max() * 1000))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--set", nargs=2, action="append", metavar=("LABEL", "RUN_DIR"), required=True)
    parser.add_argument("--links", default="arm_carriage,base_link")
    parser.add_argument("--near", type=float, default=0.05)
    parser.add_argument("--tolerance", type=float, default=0.005)
    parser.add_argument("--protrusion", type=float, default=0.01)
    parser.add_argument("--budget", type=int, default=40, help="maximum added spheres per link")
    parser.add_argument("--min-gain", type=int, default=5, help="stop when a new sphere covers fewer relevant points")
    parser.add_argument("--spacing", type=float, default=0.008, help="surface sample spacing (m)")
    parser.add_argument("--stride", type=int, default=16)
    parser.add_argument("--relevance", type=Path, help="cached relevance .npz (written if missing)")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    fixed, mobile = yaml.safe_load(FIXED.read_text()), yaml.safe_load(MOBILE.read_text())
    config = mobile["kinematics"]
    robot = yourdfpy.URDF.load(config["urdf_path"], load_meshes=False, build_scene_graph=True)
    links = args.links.split(",")
    cache = args.relevance or args.output / "relevance.npz"
    if cache.exists():
        stored = np.load(cache)
        targets = {name: (stored[f"{name}/points"], stored[f"{name}/normals"], stored[f"{name}/flags"]) for name in links}
    else:
        targets = relevance(robot, config, args.set, links, args.spacing, args.near, args.stride)
        np.savez_compressed(cache, **{f"{name}/{key}": value for name, values in targets.items()
                                      for key, value in zip(("points", "normals", "flags"), values)})
    spheres = fixed["kinematics"]["collision_spheres"]
    report = dict(parameters={key: (str(value) if isinstance(value, Path) else value) for key, value in vars(args).items()}
                  | dict(moving_links=list(MOVING_LINKS), candidate_depths_m=list(DEPTHS_M)), links={})
    patched = {name: list(balls) for name, balls in spheres.items()}
    for name in links:
        points, normals, flags = targets[name]
        relevant = mirrored(points, flags, args.spacing)
        meshes = link_meshes(robot, name)
        frozen = [ball for ball in spheres[name] if ball["radius"] > 0]
        centers, radii = np.array([b["center"] for b in frozen]), np.array([b["radius"] for b in frozen])
        added = greedy(meshes, points[relevant], normals[relevant], centers, radii, args.tolerance, args.protrusion,
                       args.budget, args.min_gain)
        new_centers = np.vstack([centers, *[np.asarray(b["center"])[None] for b in added]])
        new_radii = np.concatenate([radii, [b["radius"] for b in added]])
        shell = trimesh.creation.icosphere(subdivisions=3).vertices
        protrusion = max((float(-inside_depth(meshes, np.asarray(b["center"]) + b["radius"] * shell).min()) for b in added),
                         default=0.)
        report["links"][name] = dict(
            surface_samples=int(len(points)), relevant_direct=int(flags.sum()), relevant_mirrored=int(relevant.sum()),
            frozen_spheres=len(frozen), added_spheres=added, max_protrusion_mm=protrusion * 1000,
            relevant_gap_before=describe(gaps(points[relevant], centers, radii)),
            relevant_gap_after=describe(gaps(points[relevant], new_centers, new_radii)),
            surface_gap_before=describe(gaps(points, centers, radii)),
            surface_gap_after=describe(gaps(points, new_centers, new_radii)))
        patched[name] += [dict(center=b["center"], radius=b["radius"]) for b in added]
    for document, target in ((fixed, args.output / FIXED.name), (mobile, args.output / "mobile18" / MOBILE.name)):
        target.parent.mkdir(parents=True, exist_ok=True)
        document["kinematics"]["collision_spheres"] = patched
        target.write_text(yaml.safe_dump(document, sort_keys=False))
    if not (args.output / "mobile18" / "model").exists():
        (args.output / "mobile18" / "model").symlink_to(MOBILE.parent / "model")
    (args.output / "patch.json").write_text(json.dumps(report, indent=1) + "\n")
    for name, value in report["links"].items():
        print(f"{name}: relevant {value['relevant_direct']} -> {value['relevant_mirrored']} of {value['surface_samples']} | "
              f"added {len(value['added_spheres'])} (protrusion <= {value['max_protrusion_mm']:.1f} mm) | relevant gap p95/max "
              f"{value['relevant_gap_before']['p95_mm']:.1f}/{value['relevant_gap_before']['max_mm']:.1f} -> "
              f"{value['relevant_gap_after']['p95_mm']:.1f}/{value['relevant_gap_after']['max_mm']:.1f} mm")


if __name__ == "__main__":
    main()
