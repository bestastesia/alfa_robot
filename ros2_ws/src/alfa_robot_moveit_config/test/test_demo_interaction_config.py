#!/usr/bin/env python3

from pathlib import Path

import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    runtime_kinematics = yaml.safe_load(
        (PACKAGE_ROOT / "config/kinematics.yaml").read_text(encoding="utf-8")
    )
    assert runtime_kinematics == {}

    demo_kinematics = yaml.safe_load(
        (PACKAGE_ROOT / "config/kinematics.demo.yaml").read_text(encoding="utf-8")
    )
    for group in ("left_arm", "right_arm"):
        assert demo_kinematics[group]["kinematics_solver"] == (
            "kdl_kinematics_plugin/KDLKinematicsPlugin"
        )

    demo_launch = (PACKAGE_ROOT / "launch/demo.launch.py").read_text(encoding="utf-8")
    assert "move_group_demo.launch.py" in demo_launch
    assert "moveit_rviz_demo.launch.py" in demo_launch

    rviz = (PACKAGE_ROOT / "config/moveit.rviz").read_text(encoding="utf-8")
    assert "Query Goal State: true" in rviz
    assert "Query Start State: false" in rviz
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
