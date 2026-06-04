#!/usr/bin/env python3
"""Visualize 9-orientation reachability CSV results with the real robot model.

This loads the actual URDF (via xacro) and renders every visual STL at its
real FK pose. The displayed robot matches RViz/MoveIt exactly, so orientation,
mounting position and arm geometry can be trusted for reachability analysis.

Conventions (read this before interpreting the plot):
  - World frame at origin, X-axis red, Y-axis green, Z-axis blue.
  - Robot front      = +X (red arrow points forward)
  - Robot left       = +Y (green arrow points to the robot's left side)
  - Robot up         = +Z (blue arrow points up)
  - Left arm (blue)  mounts at y > 0, right arm (orange) at y < 0.

The robot pose is the URDF home pose (all joints at 0). updown is at its
lower limit (column lowest), which matches the default MoveIt state.
"""

from __future__ import annotations

import argparse
import csv
import os
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np


# ── CSV loading (unchanged from previous version) ─────────────────────

@dataclass
class PointSummary:
    x: float
    y: float
    z: float
    n_success: int
    n_total: int
    point_reachable: bool


def load_summaries(csv_path: Path) -> list[PointSummary]:
    summaries = []
    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("orient_label") == "SUMMARY" or row.get("orient_idx") == "-1":
                try:
                    summaries.append(PointSummary(
                        x=float(row["x"]),
                        y=float(row["y"]),
                        z=float(row["z"]),
                        n_success=int(row["n_success"]),
                        n_total=int(row["n_total"]),
                        point_reachable=bool(int(row["point_reachable"])),
                    ))
                except (ValueError, KeyError):
                    continue
    return summaries


def categorize_points(summaries: list[PointSummary], n_orient: int = 9):
    reachable, partial, unreachable, partial_ratios = [], [], [], []
    for s in summaries:
        pt = [s.x, s.y, s.z]
        total = s.n_total if s.n_total > 0 else n_orient
        ratio = s.n_success / total
        if s.n_success == total:
            reachable.append(pt)
        elif s.n_success == 0:
            unreachable.append(pt)
        else:
            partial.append(pt)
            partial_ratios.append(ratio)
    return (
        np.asarray(reachable, dtype=np.float64),
        np.asarray(unreachable, dtype=np.float64),
        np.asarray(partial, dtype=np.float64),
        np.asarray(partial_ratios, dtype=np.float64),
    )


def make_cloud(points: np.ndarray, color):
    import open3d as o3d
    cloud = o3d.geometry.PointCloud()
    if len(points) > 0:
        cloud.points = o3d.utility.Vector3dVector(points)
        cloud.paint_uniform_color(color)
    return cloud


def make_partial_cloud(points: np.ndarray, ratios: np.ndarray):
    import open3d as o3d
    cloud = o3d.geometry.PointCloud()
    if len(points) == 0:
        return cloud
    cloud.points = o3d.utility.Vector3dVector(points)
    colors = np.zeros((len(points), 3))
    for i, r in enumerate(ratios):
        if r < 0.5:
            colors[i] = [1.0, r * 2.0, 0.0]
        else:
            colors[i] = [2.0 * (1.0 - r), 1.0, 0.0]
    cloud.colors = o3d.utility.Vector3dVector(colors)
    return cloud


# ── Real robot loading via URDF + STL ─────────────────────────────────

def find_description_share() -> Path:
    """Locate alfa_robot_description share/ directory (install preferred, src fallback)."""
    here = Path(__file__).resolve()
    candidates = [
        # ros2_ws/install/alfa_robot_description/share/alfa_robot_description
        here.parents[3] / "install/alfa_robot_description/share/alfa_robot_description",
        # project-root/install/alfa_robot_description/share/alfa_robot_description
        here.parents[4] / "install/alfa_robot_description/share/alfa_robot_description",
        # ros2_ws/src/alfa_robot_description
        here.parents[2] / "alfa_robot_description",
    ]
    for c in candidates:
        if (c / "meshes").exists():
            return c
    raise FileNotFoundError(
        "Cannot locate alfa_robot_description. Searched:\n  "
        + "\n  ".join(str(c) for c in candidates)
    )


def compile_xacro_to_urdf(xacro_path: Path) -> Path:
    """Run xacro to produce a flat URDF file. Returns path to temp URDF."""
    out = Path(tempfile.mkstemp(suffix=".urdf", prefix="alfa_robot_")[1])
    try:
        result = subprocess.run(
            ["xacro", str(xacro_path)],
            capture_output=True, text=True, check=True,
        )
    except FileNotFoundError:
        raise RuntimeError("xacro not found. Source your ROS 2 environment first.")
    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"xacro failed: {e.stderr}")
    out.write_text(result.stdout)
    return out


def make_filename_handler(description_share: Path):
    """Return a callable mapping `package://alfa_robot_description/...` to a real path.

    yourdfpy passes the filename as keyword arg `fname`; accept both forms.
    """
    prefix = "package://alfa_robot_description/"

    def handler(fname: Optional[str] = None, filename: Optional[str] = None) -> str:
        name = fname if fname is not None else filename
        if name is None:
            return ""
        if name.startswith(prefix):
            return str(description_share / name[len(prefix):])
        return name
    return handler


def load_robot_geometries(verbose: bool = False) -> list:
    """Load real URDF, compute FK at home pose, return list of Open3D meshes."""
    import open3d as o3d
    import yourdfpy

    description_share = find_description_share()
    xacro_path = description_share / "urdf" / "alfa_robot.urdf.xacro"
    if not xacro_path.exists():
        # Fall back to source tree (install copies usually include the xacro too)
        alt = Path(__file__).resolve().parents[4] / "src/alfa_robot_description/urdf/alfa_robot.urdf.xacro"
        if alt.exists():
            xacro_path = alt
        else:
            raise FileNotFoundError(f"xacro not found at {xacro_path}")

    if verbose:
        print(f"  description share: {description_share}")
        print(f"  xacro:            {xacro_path}")

    urdf_path = compile_xacro_to_urdf(xacro_path)
    handler = make_filename_handler(description_share)

    robot = yourdfpy.URDF.load(
        str(urdf_path),
        filename_handler=handler,
        load_meshes=True,
        build_scene_graph=True,
    )

    # Home pose: zeros everywhere. yourdfpy's default cfg is already zeros.
    # Compute FK for every visual link.
    geometries: list = []
    for link_name, link in robot.link_map.items():
        if not link.visuals:
            continue
        try:
            T_world_link = robot.get_transform(link_name)
        except Exception:
            continue
        for visual in link.visuals:
            geom = visual.geometry
            if geom.mesh is None:
                continue
            mesh_path = handler(geom.mesh.filename)
            if not os.path.exists(mesh_path):
                if verbose:
                    print(f"  skip missing mesh: {mesh_path}")
                continue
            mesh = o3d.io.read_triangle_mesh(mesh_path)
            if mesh.is_empty():
                continue
            # Apply mesh scale if any
            scale = geom.mesh.scale
            if scale is not None:
                S = np.eye(4)
                S[0, 0], S[1, 1], S[2, 2] = float(scale[0]), float(scale[1]), float(scale[2])
                mesh.transform(S)
            # Visual origin offset (inside the link frame)
            if visual.origin is not None:
                mesh.transform(np.asarray(visual.origin))
            # Then the link's world transform
            mesh.transform(np.asarray(T_world_link))
            mesh.compute_vertex_normals()
            # Color: prefer URDF material, else fall back to neutral grey
            color = None
            if visual.material is not None and visual.material.color is not None:
                rgba = visual.material.color.rgba
                color = [float(rgba[0]), float(rgba[1]), float(rgba[2])]
            if color is None:
                if "left" in link_name:
                    color = [0.30, 0.55, 0.95]
                elif "right" in link_name:
                    color = [0.95, 0.55, 0.25]
                else:
                    color = [0.55, 0.55, 0.58]
            mesh.paint_uniform_color(color)
            geometries.append(mesh)
            if verbose:
                origin = np.asarray(T_world_link)[:3, 3]
                print(f"  loaded {link_name:22s} at ({origin[0]:+.3f}, {origin[1]:+.3f}, {origin[2]:+.3f})  mesh={Path(mesh_path).name}")

    try:
        urdf_path.unlink()
    except OSError:
        pass

    return geometries


def make_world_axes(size: float = 0.4):
    """World frame axes at origin: X red, Y green, Z blue. Robot front = +X."""
    import open3d as o3d
    return o3d.geometry.TriangleMesh.create_coordinate_frame(size=size, origin=[0, 0, 0])


def make_axis_labels(size: float = 0.4):
    """Annotate axes with small colored arrows extending beyond the world frame."""
    import open3d as o3d
    labels = []
    # +X arrow (forward, red)
    x_arrow = o3d.geometry.TriangleMesh.create_arrow(
        cylinder_radius=0.01, cone_radius=0.025,
        cylinder_height=size * 1.5, cone_height=0.06,
    )
    R = o3d.geometry.get_rotation_matrix_from_xyz([0, np.pi / 2, 0])
    x_arrow.rotate(R, center=[0, 0, 0])
    x_arrow.paint_uniform_color([0.9, 0.1, 0.1])
    labels.append(x_arrow)
    return labels


def main():
    parser = argparse.ArgumentParser(description="Visualize 9-orient reachability CSV with real robot model")
    parser.add_argument("csv", type=Path, help="Reachability CSV path")
    parser.add_argument("--show-unreachable", action="store_true", help="Also show 0/9 unreachable points")
    parser.add_argument("--point-size", type=float, default=4.0, help="Open3D point size")
    parser.add_argument("--no-robot", action="store_true", help="Skip robot model rendering")
    parser.add_argument("--n-orient", type=int, default=9, help="Expected orientations per point")
    parser.add_argument("--axes-size", type=float, default=0.4, help="World frame axes length")
    parser.add_argument("--verbose", action="store_true", help="Print mesh load progress")
    args = parser.parse_args()

    print(f"Loading reachability CSV: {args.csv}")
    summaries = load_summaries(args.csv)
    print(f"  point summaries: {len(summaries)}")
    if not summaries:
        print("No SUMMARY rows in CSV, nothing to display.")
        return

    reachable, unreachable, partial, partial_ratios = categorize_points(summaries, args.n_orient)
    print(f"  fully reachable (n/n): {len(reachable)}")
    print(f"  partial:               {len(partial)}")
    print(f"  unreachable (0/n):     {len(unreachable)}")

    if len(reachable) + len(partial) + len(unreachable) == 0:
        print("Nothing to display.")
        return

    import open3d as o3d

    geometries = []

    # World axes — robot front is +X (red), left is +Y (green), up is +Z (blue)
    geometries.append(make_world_axes(args.axes_size))

    # Real robot model (URDF + STL meshes at home pose)
    if not args.no_robot:
        print("Loading robot model from URDF…")
        try:
            geometries.extend(load_robot_geometries(verbose=args.verbose))
            print("  robot model loaded")
        except Exception as e:
            print(f"  WARNING: failed to load robot model: {e}")
            print("  Showing reachability points without robot.")

    # Reachability point clouds
    if len(reachable) > 0:
        geometries.append(make_cloud(reachable, [0.1, 0.85, 0.2]))
    if len(partial) > 0:
        geometries.append(make_partial_cloud(partial, partial_ratios))
    if args.show_unreachable and len(unreachable) > 0:
        geometries.append(make_cloud(unreachable, [0.9, 0.1, 0.1]))

    title = f"9-Orient Reachability: {args.csv.name}    |   front=+X (red)  left=+Y (green)  up=+Z (blue)"
    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name=title, width=1500, height=950)
    for g in geometries:
        vis.add_geometry(g)

    opt = vis.get_render_option()
    opt.point_size = args.point_size
    opt.background_color = np.asarray([0.05, 0.05, 0.07])
    opt.mesh_show_back_face = True
    opt.light_on = True

    # Initial view: look from +X / -Y / +Z (in front, slightly right, slightly above)
    ctr = vis.get_view_control()
    ctr.set_front([0.6, -0.4, -0.5])
    ctr.set_up([0, 0, 1])
    ctr.set_lookat([0.0, 0.0, 0.9])
    ctr.set_zoom(0.55)

    print()
    print("Controls: drag = rotate, right-drag = pan, scroll = zoom, Q/Esc = close")
    print("World frame at origin:  X-axis (red) = robot front   Y-axis (green) = robot left   Z-axis (blue) = up")
    vis.run()
    vis.destroy_window()


if __name__ == "__main__":
    main()
