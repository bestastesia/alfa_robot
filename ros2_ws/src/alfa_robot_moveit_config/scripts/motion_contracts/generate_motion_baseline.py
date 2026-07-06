#!/usr/bin/env python3
from __future__ import annotations

import argparse
import hashlib
import json
import subprocess
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

REPO_ROOT = Path(__file__).resolve().parents[5]
DEFAULT_MANIFEST = REPO_ROOT / "ros2_ws/src/alfa_robot_moveit_config/config/motion_baselines/current_motion_baseline.yaml"

HASH_KEYS = (
    ("robot_model", "description_xacro"),
    ("robot_model", "moveit_wrapper_xacro"),
    ("robot_model", "srdf"),
    ("robot_model", "joint_limits"),
    ("robot_model", "ros2_control"),
)


def load_yaml(path: Path) -> dict[str, Any]:
    if yaml is None:
        raise RuntimeError("PyYAML is required: python3 -m pip install pyyaml")
    return yaml.safe_load(path.read_text()) or {}


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def git_value(args: list[str]) -> str:
    try:
        return subprocess.check_output(args, cwd=REPO_ROOT, text=True, stderr=subprocess.DEVNULL).strip()
    except Exception:
        return "unknown"


def path_from_manifest(value: str) -> Path:
    path = Path(value)
    return path if path.is_absolute() else REPO_ROOT / path


def add_file_hashes(manifest: dict[str, Any]) -> dict[str, Any]:
    files: dict[str, dict[str, str]] = {}
    for section, key in HASH_KEYS:
        raw = str(manifest.get(section, {}).get(key, ""))
        if not raw:
            continue
        path = path_from_manifest(raw)
        files[f"{section}.{key}"] = {
            "path": raw,
            "exists": str(path.exists()).lower(),
            "sha256": file_sha256(path) if path.exists() else "missing",
        }
    manifest["tracked_files"] = files
    return manifest


def canonical_json(data: dict[str, Any]) -> str:
    return json.dumps(data, ensure_ascii=False, sort_keys=True, separators=(",", ":"))


def compute_baseline_id(manifest: dict[str, Any]) -> str:
    data = dict(manifest)
    data.pop("baseline_id", None)
    digest = hashlib.sha256(canonical_json(data).encode("utf-8")).hexdigest()[:16]
    return f"motion-baseline-{digest}"


def build_baseline(manifest_path: Path) -> dict[str, Any]:
    manifest = load_yaml(manifest_path)
    manifest["manifest_path"] = str(manifest_path.relative_to(REPO_ROOT) if manifest_path.is_relative_to(REPO_ROOT) else manifest_path)
    manifest["git"] = {
        "commit": git_value(["git", "rev-parse", "HEAD"]),
        "branch": git_value(["git", "branch", "--show-current"]),
        "dirty": "true" if git_value(["git", "status", "--porcelain"]) else "false",
    }
    add_file_hashes(manifest)
    manifest["baseline_id"] = compute_baseline_id(manifest)
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description="Generate ALFA motion baseline JSON from the current manifest and key file hashes.")
    parser.add_argument("--manifest", type=Path, default=DEFAULT_MANIFEST)
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--print-id", action="store_true")
    args = parser.parse_args()

    baseline = build_baseline(args.manifest)
    text = json.dumps(baseline, ensure_ascii=False, indent=2, sort_keys=True)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n")
    if args.print_id:
        print(baseline["baseline_id"])
    else:
        print(text)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
