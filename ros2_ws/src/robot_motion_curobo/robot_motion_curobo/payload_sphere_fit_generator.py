"""Generate reproducible cuRobo payload sphere fits for visualization.

Run this module with the Python environment that can import the local cuRobo
checkout and access CUDA.  The resulting JSON is intentionally independent of
Torch/cuRobo so that the normal ROS Python environment can render it in Rerun.
"""

from __future__ import annotations

import argparse
import json
import platform
import random
from dataclasses import asdict
from pathlib import Path
from typing import Sequence

import numpy as np


DEFAULT_BOXES = {
    "top_suction": (0.3, 0.4, 0.4),
    "front": (0.4, 0.4, 0.3),
}
DEFAULT_MODES = ("morphit", "voxel", "surface")


def exact_axis_protrusion(
    centers: np.ndarray, radii: np.ndarray, size: Sequence[float]
) -> dict[str, object]:
    """Return exact per-axis/AABB protrusion beyond an axis-aligned box."""
    centers = np.asarray(centers, dtype=float).reshape(-1, 3)
    radii = np.asarray(radii, dtype=float).reshape(-1)
    half = 0.5 * np.asarray(size, dtype=float)
    if len(centers) != len(radii):
        raise ValueError("sphere center/radius count mismatch")
    if not len(centers):
        return {
            "per_axis_m": [0.0, 0.0, 0.0],
            "maximum_m": 0.0,
            "sphere_aabb_min": [0.0, 0.0, 0.0],
            "sphere_aabb_max": [0.0, 0.0, 0.0],
        }
    per_axis = np.maximum(np.abs(centers) + radii[:, None] - half[None, :], 0.0).max(axis=0)
    lower = (centers - radii[:, None]).min(axis=0)
    upper = (centers + radii[:, None]).max(axis=0)
    return {
        "per_axis_m": per_axis.tolist(),
        "maximum_m": float(per_axis.max()),
        "sphere_aabb_min": lower.tolist(),
        "sphere_aabb_max": upper.tolist(),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Fit MORPHIT/VOXEL/SURFACE payload spheres with real cuRobo."
    )
    parser.add_argument("--output", required=True)
    parser.add_argument("--num-spheres", type=int, default=64)
    parser.add_argument("--surface-radius", type=float, default=0.005)
    parser.add_argument("--morphit-iterations", type=int, default=200)
    parser.add_argument("--modes", default=",".join(DEFAULT_MODES))
    parser.add_argument("--seed", type=int, default=20260720)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.num_spheres <= 0:
        raise ValueError("--num-spheres must be positive")
    if args.surface_radius <= 0.0:
        raise ValueError("--surface-radius must be positive")

    import torch
    import trimesh
    from curobo._src.geom.sphere_fit.fit_spheres import fit_spheres_to_mesh
    from curobo._src.geom.sphere_fit.types import SphereFitType
    from curobo._src.types.device_cfg import DeviceCfg

    if not torch.cuda.is_available():
        raise RuntimeError("CUDA is required for the real cuRobo sphere fit")
    modes = tuple(value.strip().lower() for value in args.modes.split(",") if value.strip())
    unsupported = [value for value in modes if value not in DEFAULT_MODES]
    if unsupported:
        raise ValueError(f"unsupported fit modes: {unsupported}")

    output = {
        "schema_version": 1,
        "generator": "robot_motion_curobo.payload_sphere_fit_generator",
        "seed": args.seed,
        "num_spheres_requested": args.num_spheres,
        "surface_radius_m": args.surface_radius,
        "morphit_iterations": args.morphit_iterations,
        "environment": {
            "python": platform.python_version(),
            "torch": torch.__version__,
            "cuda_device": torch.cuda.get_device_name(0),
            "trimesh": trimesh.__version__,
        },
        "boxes": {},
    }
    device_cfg = DeviceCfg()
    mode_types = {
        "morphit": SphereFitType.MORPHIT,
        "voxel": SphereFitType.VOXEL,
        "surface": SphereFitType.SURFACE,
    }

    for box_index, (box_name, size) in enumerate(DEFAULT_BOXES.items()):
        mesh = trimesh.creation.box(extents=size)
        box_record = {"size_xyz": list(size), "fits": {}}
        for mode_index, mode in enumerate(modes):
            fit_seed = args.seed + box_index * 100 + mode_index
            random.seed(fit_seed)
            np.random.seed(fit_seed)
            torch.manual_seed(fit_seed)
            torch.cuda.manual_seed_all(fit_seed)
            result = fit_spheres_to_mesh(
                mesh,
                num_spheres=args.num_spheres,
                surface_radius=args.surface_radius,
                fit_type=mode_types[mode],
                iterations=args.morphit_iterations,
                compute_metrics=True,
                device_cfg=device_cfg,
            )
            centers = result.centers.detach().cpu().numpy()
            radii = result.radii.detach().cpu().numpy()
            metrics = asdict(result.metrics) if result.metrics is not None else None
            box_record["fits"][mode] = {
                "seed": fit_seed,
                "sphere_count": int(result.num_spheres),
                "fit_time_s": float(result.fit_time_s or 0.0),
                "debug_info": result.debug_info,
                "metrics": metrics,
                "exact_axis_protrusion": exact_axis_protrusion(centers, radii, size),
                "centers": centers.tolist(),
                "radii": radii.tolist(),
            }
            print(
                f"{box_name}/{mode}: count={result.num_spheres} "
                f"fit={result.fit_time_s:.3f}s "
                f"coverage={metrics['coverage']:.4f} "
                f"protrusion={metrics['protrusion']:.4f} "
                f"max_axis_out={box_record['fits'][mode]['exact_axis_protrusion']['maximum_m']:.6f}m"
            )
        output["boxes"][box_name] = box_record

    output_path = Path(args.output).expanduser().resolve()
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(output, indent=2, ensure_ascii=False) + "\n")
    print(f"saved: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
