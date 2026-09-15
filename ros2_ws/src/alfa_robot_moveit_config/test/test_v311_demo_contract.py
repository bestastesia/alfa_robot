#!/usr/bin/env python3

import json
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


MOVEIT_ROOT = Path(__file__).resolve().parents[1]
DESCRIPTION_ROOT = MOVEIT_ROOT.parent / "alfa_robot_description"
ACTIVE_JOINTS = {
    "updown",
    "head_joint",
    "head_pitch_joint",
    *(f"{side}_joint{index}" for side in ("left", "right") for index in range(1, 8)),
}


def controller_joints(path):
    payload = yaml.safe_load(path.read_text())
    if "controller_manager" in payload:
        return payload["dual_arm_controller"]["ros__parameters"]["joints"]
    return payload["moveit_simple_controller_manager"]["dual_arm_controller"]["joints"]


def main():
    lock = json.loads((DESCRIPTION_ROOT / "config/upstream_description.lock.json").read_text())
    assert lock["model_revision"] == "robot_v3.1.1-hybrid"
    assert lock["profile"] == "suction"
    assert lock["upstream_commit"] == "30313fb54999f9e09a1245c9f77d8e0243e1d11c"

    initial = yaml.safe_load((MOVEIT_ROOT / "config/initial_positions.yaml").read_text())["initial_positions"]
    limits = yaml.safe_load((MOVEIT_ROOT / "config/joint_limits.yaml").read_text())["joint_limits"]
    assert initial["updown"] == 0.0
    assert limits["updown"]["min_position"] == -1.0
    assert limits["updown"]["max_position"] == 0.0

    ros2_control = controller_joints(MOVEIT_ROOT / "config/ros2_controllers.yaml")
    moveit = controller_joints(MOVEIT_ROOT / "config/moveit_controllers.yaml")
    assert len(ros2_control) == len(set(ros2_control)) == 17
    assert set(ros2_control) == set(moveit) == ACTIVE_JOINTS

    srdf = ET.parse(MOVEIT_ROOT / "config/alfa_robot.srdf").getroot()
    groups = {group.get("name"): group for group in srdf.findall("group")}
    required_groups = {
        "left_arm",
        "right_arm",
        "dual_arm",
        "head",
        "left_arm_with_updown",
        "right_arm_with_updown",
        "dual_arm_with_updown",
        "whole_body",
        "left_tool_group",
        "right_tool_group",
    }
    assert required_groups <= set(groups)
    assert groups["left_arm"].find("chain").get("tip_link") == "left_tool0"
    assert groups["right_arm"].find("chain").get("tip_link") == "right_tool0"
    passive = {joint.get("name") for joint in srdf.findall("passive_joint")}
    assert passive == {
        "active_suspension_joint",
        *(f"{kind}{index:02d}_joint" for kind in ("caster", "wheel") for index in range(1, 5)),
    }


if __name__ == "__main__":
    main()
