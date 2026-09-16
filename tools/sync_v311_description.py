#!/usr/bin/env python3
"""Sync the V3.1.1 Kkozia profile from the authoritative description repository."""

import argparse
import hashlib
import json
from pathlib import Path
import subprocess


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
PACKAGE_ROOT = REPOSITORY_ROOT / "ros2_ws/src/alfa_robot_description"
DEFAULT_SOURCE_REPOSITORY = Path("/mnt/mydisk/ALFA/SevenovaHangzhou/robot_description")
DEFAULT_SOURCE_REF = "origin/robot_v3_suction_chassis"
EXPECTED_MODEL_REVISION = "robot_v3.1.1-hybrid"
SOURCE_MANIFEST = "config/v3_1_1_integration_manifest.json"
SOURCE_XACROS = (
    "urdf/robot_v3_1_1.xacro",
    "urdf/robot_v3_chassis.xacro",
    "urdf/robot_v3_end_effectors.xacro",
    "urdf/robot_v3_head.xacro",
)
SOURCE_CONFIGS = (
    "config/initial_positions.yaml",
    "config/initial_positions_gripper.yaml",
    "config/initial_positions_suction.yaml",
    "config/named_poses.yaml",
    "config/named_poses_gripper.yaml",
    "config/named_poses_suction.yaml",
    "config/joint_limits.yaml",
    "config/joint_limits_gripper.yaml",
    "config/joint_limits_suction.yaml",
)
MANAGED_MESH_DIRECTORIES = (
    "meshes/active_suspension",
    "meshes/chassis",
    "meshes/head",
    "meshes/robot_v3",
    "meshes/robot_v3_1_1",
)


def git_output(repository, *arguments):
    return subprocess.run(
        ["git", "-C", str(repository), *arguments],
        check=True,
        capture_output=True,
    ).stdout


def digest(data):
    return hashlib.sha256(data).hexdigest()


def source_blob(repository, source_ref, relative_path):
    return git_output(repository, "show", f"{source_ref}:{relative_path}")


def source_paths(repository, source_ref, prefix):
    output = git_output(repository, "ls-tree", "-r", "--name-only", source_ref, "--", prefix)
    return tuple(line for line in output.decode().splitlines() if line)


def transform_xacro(data):
    return data.replace(
        b"package://robot_description/meshes/",
        b"package://alfa_robot_description/meshes/",
    ).replace(
        b"$(find robot_description)/urdf/",
        b"$(find alfa_robot_description)/urdf/alfa_robot/",
    )


def destination_path(source_path):
    relative = Path(source_path)
    if relative.parts[0] == "urdf":
        return PACKAGE_ROOT / "urdf/alfa_robot" / relative.name
    return PACKAGE_ROOT / relative


def expected_snapshot(repository, source_ref):
    commit = git_output(repository, "rev-parse", f"{source_ref}^{{commit}}").decode().strip()
    manifest_bytes = source_blob(repository, source_ref, SOURCE_MANIFEST)
    manifest = json.loads(manifest_bytes)
    if manifest.get("model_revision") != EXPECTED_MODEL_REVISION:
        raise ValueError(
            f"{source_ref} provides {manifest.get('model_revision')}, expected {EXPECTED_MODEL_REVISION}"
        )

    source_files = list(SOURCE_XACROS) + list(SOURCE_CONFIGS)
    for directory in MANAGED_MESH_DIRECTORIES:
        source_files.extend(source_paths(repository, source_ref, directory))
    source_files = sorted(set(source_files))

    files = {}
    source_hashes = {}
    for relative_path in source_files:
        data = source_blob(repository, source_ref, relative_path)
        source_hashes[relative_path] = digest(data)
        if relative_path.endswith(".xacro"):
            data = transform_xacro(data)
        files[destination_path(relative_path)] = data

    files[PACKAGE_ROOT / "config/upstream_description_manifest.json"] = manifest_bytes
    origin_url = git_output(repository, "remote", "get-url", "origin").decode().strip()
    lock = {
        "schema_version": 2,
        "model_revision": EXPECTED_MODEL_REVISION,
        "profile": "suction",
        "upstream_repository": origin_url,
        "upstream_branch": "robot_v3_suction_chassis",
        "upstream_ref_used_for_sync": source_ref,
        "upstream_commit": commit,
        "upstream_manifest_sha256": digest(manifest_bytes),
        "package_uri_rewrite": {
            "from": "package://robot_description/meshes/",
            "to": "package://alfa_robot_description/meshes/",
        },
        "managed_source_files": source_hashes,
        "managed_destination_files": {
            str(path.relative_to(PACKAGE_ROOT)): digest(data)
            for path, data in sorted(files.items(), key=lambda item: str(item[0]))
        },
        "movable_joint_counts": {"suction": 26, "gripper": 28},
        "integration_status": "moveit_demo_consumer_snapshot",
    }
    lock_bytes = json.dumps(lock, ensure_ascii=False, indent=2, sort_keys=True).encode() + b"\n"
    files[PACKAGE_ROOT / "config/upstream_description.lock.json"] = lock_bytes
    return files


def check_snapshot(files, allow_equivalent_source_ref=False):
    mismatches = []
    for path, expected in files.items():
        if not path.is_file():
            mismatches.append(f"missing: {path.relative_to(REPOSITORY_ROOT)}")
            continue
        if path.read_bytes() == expected:
            continue
        if path.name == "upstream_description.lock.json" and allow_equivalent_source_ref:
            actual_lock = json.loads(path.read_text())
            expected_lock = json.loads(expected)
            actual_lock.pop("upstream_ref_used_for_sync", None)
            expected_lock.pop("upstream_ref_used_for_sync", None)
            # A reviewed PR may add a merge/squash commit without changing any
            # managed description bytes. The per-file hashes remain authoritative.
            actual_lock.pop("upstream_commit", None)
            expected_lock.pop("upstream_commit", None)
            if actual_lock == expected_lock:
                continue
        mismatches.append(f"content differs: {path.relative_to(REPOSITORY_ROOT)}")
    if mismatches:
        raise SystemExit("V3.1.1 description snapshot mismatch:\n" + "\n".join(mismatches))


def check_local_snapshot():
    lock_path = PACKAGE_ROOT / "config/upstream_description.lock.json"
    manifest_path = PACKAGE_ROOT / "config/upstream_description_manifest.json"
    if not lock_path.is_file() or not manifest_path.is_file():
        raise SystemExit("V3.1.1 local snapshot lock or manifest is missing")
    lock = json.loads(lock_path.read_text())
    if lock.get("model_revision") != EXPECTED_MODEL_REVISION:
        raise SystemExit("V3.1.1 local snapshot lock has the wrong model revision")
    if digest(manifest_path.read_bytes()) != lock["upstream_manifest_sha256"]:
        raise SystemExit("V3.1.1 local manifest does not match its lock")
    for relative_path, expected_hash in lock["managed_destination_files"].items():
        path = PACKAGE_ROOT / relative_path
        if not path.is_file() or digest(path.read_bytes()) != expected_hash:
            raise SystemExit(f"V3.1.1 managed file differs from its lock: {relative_path}")
    print(
        f"Local consumer snapshot matches {EXPECTED_MODEL_REVISION} "
        f"at {lock['upstream_commit'][:10]}."
    )


def write_snapshot(files):
    expected_paths = set(files)
    for directory in MANAGED_MESH_DIRECTORIES:
        target_directory = PACKAGE_ROOT / directory
        if not target_directory.exists():
            continue
        for path in target_directory.glob("*.stl"):
            if path not in expected_paths:
                path.unlink()
    for path, data in files.items():
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(data)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-repository", type=Path, default=DEFAULT_SOURCE_REPOSITORY)
    parser.add_argument("--source-ref", default=DEFAULT_SOURCE_REF)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--check", action="store_true", help="compare against the source repository")
    mode.add_argument("--check-local", action="store_true", help="verify the pinned local snapshot only")
    arguments = parser.parse_args()
    if arguments.check_local:
        check_local_snapshot()
        return
    source_repository = arguments.source_repository.resolve()
    files = expected_snapshot(source_repository, arguments.source_ref)
    if arguments.check:
        check_snapshot(files, allow_equivalent_source_ref=True)
        print("V3.1.1 consumer snapshot matches the authoritative description commit.")
        return
    write_snapshot(files)
    check_snapshot(files)
    lock = json.loads(files[PACKAGE_ROOT / "config/upstream_description.lock.json"])
    print(
        f"Synced {lock['model_revision']} profile={lock['profile']} "
        f"at {lock['upstream_commit']} with {len(lock['managed_destination_files'])} files."
    )


if __name__ == "__main__":
    main()
