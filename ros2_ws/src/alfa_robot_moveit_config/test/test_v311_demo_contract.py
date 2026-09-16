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
    assert lock["upstream_commit"] == "62662f48548fa475340b55f8ed251064d5c02cba"

    initial = yaml.safe_load((MOVEIT_ROOT / "config/initial_positions.yaml").read_text())["initial_positions"]
    named = yaml.safe_load((DESCRIPTION_ROOT / "config/named_poses.yaml").read_text())["named_poses"]
    limits = yaml.safe_load((MOVEIT_ROOT / "config/joint_limits.yaml").read_text())["joint_limits"]
    assert initial == named["home"]
    assert initial["updown"] == -0.3
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
    states = {
        (state.get("group"), state.get("name")): {
            joint.get("name"): float(joint.get("value"))
            for joint in state.findall("joint")
        }
        for state in srdf.findall("group_state")
    }
    arm_names = {
        f"{side}_joint{index}"
        for side in ("left", "right") for index in range(1, 8)
    }
    for pose_name in ("home", "unloading"):
        pose = named[pose_name]
        expected_arm = {name: pose[name] for name in arm_names}
        assert states[("dual_arm", pose_name)] == expected_arm
        assert states[("dual_arm_with_updown", pose_name)] == {
            "updown": pose["updown"], **expected_arm
        }
        assert states[("whole_body", pose_name)] == {
            "updown": pose["updown"],
            "head_joint": 0.0,
            "head_pitch_joint": 0.0,
            **expected_arm,
        }
    passive = {joint.get("name") for joint in srdf.findall("passive_joint")}
    assert passive == {
        "active_suspension_joint",
        *(f"{kind}{index:02d}_joint" for kind in ("caster", "wheel") for index in range(1, 5)),
    }


if __name__ == "__main__":
    main()
