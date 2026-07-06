#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import Any

try:
    import yaml
except ImportError:  # pragma: no cover
    yaml = None

REPO_ROOT = Path(__file__).resolve().parents[5]
EXECUTION_BRIDGE = REPO_ROOT / "ros2_ws/src/alfa_robot_execution_bridge"
if str(EXECUTION_BRIDGE) not in sys.path:
    sys.path.insert(0, str(EXECUTION_BRIDGE))

from alfa_robot_execution_bridge.joints import (  # noqa: E402
    DEFAULT_DIRECTION_SIGNS,
    DEFAULT_JOINT_NAMES,
    EXECUTION_JOINT_NAMES,
    FLIPPED_JOINT_NAMES,
    REAL_CONTROLLER_JOINT_NAMES,
)

DEFAULT_CONFIG = REPO_ROOT / "ros2_ws/src/alfa_robot_execution_bridge/config/execution_bridge.yaml"


def load_yaml(path: Path) -> dict[str, Any]:
    if yaml is None:
        raise RuntimeError("PyYAML is required: python3 -m pip install pyyaml")
    return yaml.safe_load(path.read_text()) or {}


def fail(message: str) -> None:
    raise SystemExit(f"ERROR: {message}")


def main() -> int:
    parser = argparse.ArgumentParser(description="Check ALFA execution joint naming and direction contract.")
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--require-real-signs", action="store_true", help="Fail if apply_direction_signs is false.")
    args = parser.parse_args()

    data = load_yaml(args.config)
    params = data.get("alfa_execution_bridge", {}).get("ros__parameters", {})
    yaml_names = [str(name) for name in params.get("joint_names", [])]
    raw_yaml_signs = params.get("direction_signs")
    yaml_signs = (
        [float(value) for value in raw_yaml_signs]
        if raw_yaml_signs is not None
        else list(DEFAULT_DIRECTION_SIGNS)
    )
    apply_direction_signs = bool(params.get("apply_direction_signs", False))

    if yaml_names != list(DEFAULT_JOINT_NAMES):
      fail(f"execution_bridge.yaml joint_names != DEFAULT_JOINT_NAMES\nyaml={yaml_names}\ncode={list(DEFAULT_JOINT_NAMES)}")
    if yaml_names != list(EXECUTION_JOINT_NAMES):
      fail("DEFAULT_JOINT_NAMES must equal EXECUTION_JOINT_NAMES")
    if yaml_signs != list(DEFAULT_DIRECTION_SIGNS):
      fail(f"execution_bridge.yaml direction_signs != DEFAULT_DIRECTION_SIGNS\nyaml={yaml_signs}\ncode={list(DEFAULT_DIRECTION_SIGNS)}")
    if sorted(REAL_CONTROLLER_JOINT_NAMES) != sorted(EXECUTION_JOINT_NAMES):
      fail("REAL_CONTROLLER_JOINT_NAMES must be a permutation of EXECUTION_JOINT_NAMES")
    if args.require_real_signs and not apply_direction_signs:
      fail("apply_direction_signs is false, but real execution requires ROS->EtherCAT sign mapping")

    print("joint contract OK")
    print(f"  joint_count={len(DEFAULT_JOINT_NAMES)}")
    print(f"  flipped_joints={','.join(FLIPPED_JOINT_NAMES) if FLIPPED_JOINT_NAMES else 'none'}")
    print(f"  apply_direction_signs={apply_direction_signs}")
    print(f"  direction_signs_source={'yaml' if raw_yaml_signs is not None else 'code_default'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
