#!/usr/bin/env python3
"""Fail when the installed MoveIt demo is not the pinned V3.1.1 consumer snapshot."""

import hashlib
import argparse
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import yaml


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_DESCRIPTION = REPOSITORY_ROOT / "ros2_ws/src/alfa_robot_description"
SOURCE_MOVEIT = REPOSITORY_ROOT / "ros2_ws/src/alfa_robot_moveit_config"
INSTALL_ROOT = REPOSITORY_ROOT / "ros2_ws/install"
INSTALL_DESCRIPTION = INSTALL_ROOT / "alfa_robot_description/share/alfa_robot_description"
INSTALL_MOVEIT = INSTALL_ROOT / "alfa_robot_moveit_config/share/alfa_robot_moveit_config"
EXPECTED_REVISION = "robot_v3.1.1-hybrid"


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def fail(message):
    print(f"V3 demo install mismatch: {message}", file=sys.stderr)
    raise SystemExit(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--description-only", action="store_true")
    arguments = parser.parse_args()
    source_lock_path = SOURCE_DESCRIPTION / "config/upstream_description.lock.json"
    installed_lock_path = INSTALL_DESCRIPTION / "config/upstream_description.lock.json"
    if not source_lock_path.is_file() or not installed_lock_path.is_file():
        fail("source or installed description lock is missing")
    source_lock = json.loads(source_lock_path.read_text())
    installed_lock = json.loads(installed_lock_path.read_text())
    if source_lock != installed_lock or source_lock.get("model_revision") != EXPECTED_REVISION:
        fail("installed description revision differs from the source lock")
    active_relative_path = "urdf/alfa_robot/robot_v3_1_1.xacro"
    expected_xacro_hash = source_lock["managed_destination_files"][active_relative_path]
    for package_root in (SOURCE_DESCRIPTION, INSTALL_DESCRIPTION):
        active_xacro = package_root / active_relative_path
        if not active_xacro.is_file() or digest(active_xacro) != expected_xacro_hash:
            fail(f"active model snapshot differs: {active_xacro}")

    if arguments.description_only:
        print(
            f"V3 description install matches {EXPECTED_REVISION} at "
            f"{source_lock['upstream_commit'][:10]}."
        )
        return

    source_limits_path = SOURCE_MOVEIT / "config/joint_limits.yaml"
    installed_limits_path = INSTALL_MOVEIT / "config/joint_limits.yaml"
    if not installed_limits_path.is_file() or source_limits_path.read_bytes() != installed_limits_path.read_bytes():
        fail("installed MoveIt joint limits differ from source")
    limits = yaml.safe_load(source_limits_path.read_text())["joint_limits"]
    updown = limits["updown"]
    if updown["min_position"] != -1.0 or updown["max_position"] != 0.0:
        fail("MoveIt updown range is not V3.1.1 [-1, 0] m")
    source_initial = yaml.safe_load(
        (SOURCE_MOVEIT / "config/initial_positions.yaml").read_text()
    )["initial_positions"]
    description_home = yaml.safe_load(
        (SOURCE_DESCRIPTION / "config/named_poses.yaml").read_text()
    )["named_poses"]["home"]
    if source_initial != description_home:
        fail("MoveIt initial positions differ from description named pose home")

    source_srdf_path = SOURCE_MOVEIT / "config/alfa_robot.srdf"
    installed_srdf_path = INSTALL_MOVEIT / "config/alfa_robot.srdf"
    if not installed_srdf_path.is_file() or source_srdf_path.read_bytes() != installed_srdf_path.read_bytes():
        fail("installed SRDF differs from source")
    groups = {group.get("name"): group for group in ET.parse(source_srdf_path).getroot().findall("group")}
    if groups["left_arm"].find("chain").get("tip_link") != "left_tool0":
        fail("left arm does not end at the Kkozia Tool0 frame")
    if groups["right_arm"].find("chain").get("tip_link") != "right_tool0":
        fail("right arm does not end at the Kkozia Tool0 frame")
    print(
        f"V3 demo install matches {EXPECTED_REVISION} at "
        f"{source_lock['upstream_commit'][:10]}; updown=[-1, 0] m."
    )


if __name__ == "__main__":
    main()
