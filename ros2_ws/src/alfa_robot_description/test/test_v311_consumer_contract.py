import hashlib
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def render(end_effector):
    result = subprocess.run(
        [
            "xacro",
            str(PACKAGE_ROOT / "urdf/alfa_robot.urdf.xacro"),
            f"end_effector:={end_effector}",
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    return ET.fromstring(result.stdout)


def test_demo_consumes_pinned_v311_kkozia_profile():
    lock = json.loads((PACKAGE_ROOT / "config/upstream_description.lock.json").read_text())
    assert lock["model_revision"] == "robot_v3.1.1-hybrid"
    assert lock["profile"] == "suction"
    assert lock["upstream_branch"] == "robot_v3_suction_chassis"
    assert lock["upstream_commit"] == "62662f48548fa475340b55f8ed251064d5c02cba"
    assert lock["upstream_ref_used_for_sync"] == "feat/motion-94-v311-named-poses"
    for profile in ("", "_gripper", "_suction"):
        assert f"config/named_poses{profile}.yaml" in lock["managed_destination_files"]

    for relative_path, expected_hash in lock["managed_destination_files"].items():
        path = PACKAGE_ROOT / relative_path
        assert path.is_file(), relative_path
        assert hashlib.sha256(path.read_bytes()).hexdigest() == expected_hash

    entrypoint = (PACKAGE_ROOT / "urdf/alfa_robot.urdf.xacro").read_text()
    assert "robot_v3_1_1.xacro" in entrypoint
    assert "robot_v3_0_9.xacro" not in entrypoint


def test_suction_and_gripper_profiles_match_control_inventory():
    for profile, expected_count in (("suction", 26), ("gripper", 28)):
        robot = render(profile)
        joints = {joint.get("name"): joint for joint in robot.findall("joint")}
        movable = {name for name, joint in joints.items() if joint.get("type") != "fixed"}
        control = robot.find("ros2_control")
        control_joints = {joint.get("name") for joint in control.findall("joint")}
        assert len(movable) == expected_count
        assert movable == control_joints
        assert ("left_moving_jaw_joint" in movable) == (profile == "gripper")

        updown = joints["updown"].find("limit")
        assert float(updown.get("lower")) == -1.0
        assert float(updown.get("upper")) == 0.0
        assert joints["left_tool0_fixed"].find("parent").get("link") == "left_joint7"
        assert joints["right_tool0_fixed"].find("parent").get("link") == "right_joint7"
        assert joints["base_footprint_to_base_link"].find("origin").get("xyz") == (
            "0.190000002779484 -0.0000442724271391554 0.40000250599116"
        )
