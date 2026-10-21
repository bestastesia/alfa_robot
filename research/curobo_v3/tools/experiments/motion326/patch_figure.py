#!/usr/bin/env python3
"""MOTION-326: render a sphere patch (frozen spheres grey, appended spheres red) over the link meshes."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import yaml
import yourdfpy

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from curobo_core.contracts import PlannerAssets
from experiments.motion326.sphere_patch import link_meshes

VIEWS = (("front", 0, 0), ("side", 0, 90), ("top", 90, -90))


def sphere(ax, center, radius, color, alpha):
    u, v = np.mgrid[0:2 * np.pi:16j, 0:np.pi:9j]
    ax.plot_wireframe(center[0] + radius * np.cos(u) * np.sin(v), center[1] + radius * np.sin(u) * np.sin(v),
                      center[2] + radius * np.cos(v), color=color, alpha=alpha, linewidth=0.4)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--patch", type=Path, required=True)
    parser.add_argument("--box", type=float, nargs=6, metavar=("XMIN", "XMAX", "YMIN", "YMAX", "ZMIN", "ZMAX"),
                        action="append", help="per-link crop in the link frame (same order as the patch links)")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    fixed = yaml.safe_load(PlannerAssets().robot_config.read_text())["kinematics"]
    robot = yourdfpy.URDF.load(fixed["urdf_path"], load_meshes=False, build_scene_graph=True)
    patch = json.loads(args.patch.read_text())["links"]
    figure = plt.figure(figsize=(6 * len(VIEWS), 5.5 * len(patch)))
    for row, (link, value) in enumerate(patch.items()):
        crop = np.asarray(args.box[row]).reshape(3, 2) if args.box else None
        inside = (lambda c: np.all((c >= crop[:, 0]) & (c <= crop[:, 1]))) if crop is not None else (lambda c: True)
        for column, (name, elevation, azimuth) in enumerate(VIEWS):
            ax = figure.add_subplot(len(patch), len(VIEWS), row * len(VIEWS) + column + 1, projection="3d")
            for mesh in link_meshes(robot, link):
                faces = mesh.vertices[mesh.faces]
                keep = [f for f in faces if inside(f.mean(axis=0))]
                if keep:
                    ax.add_collection3d(Poly3DCollection(keep, facecolor=(0.6, 0.6, 0.6, 0.25), edgecolor=(0.3, 0.3, 0.3, 0.15),
                                                         linewidth=0.2))
            for ball in fixed["collision_spheres"][link]:
                if ball["radius"] > 0 and inside(np.asarray(ball["center"])):
                    sphere(ax, np.asarray(ball["center"]), ball["radius"], "tab:blue", 0.25)
            for ball in value["added_spheres"]:
                if inside(np.asarray(ball["center"])):
                    sphere(ax, np.asarray(ball["center"]), ball["radius"], "tab:red", 0.7)
            limits = crop if crop is not None else np.vstack([m.bounds for m in link_meshes(robot, link)]).T[:, [0, -1]]
            ax.set_xlim(*limits[0]); ax.set_ylim(*limits[1]); ax.set_zlim(*limits[2])
            ax.set_box_aspect(np.ptp(limits, axis=1))
            ax.view_init(elev=elevation, azim=azimuth)
            ax.set_title(f"{link} {name}: frozen {len(fixed['collision_spheres'][link])} (blue) + {len(value['added_spheres'])} (red)",
                         fontsize=9)
            ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    figure.tight_layout()
    figure.savefig(args.output, dpi=110)


if __name__ == "__main__":
    main()
