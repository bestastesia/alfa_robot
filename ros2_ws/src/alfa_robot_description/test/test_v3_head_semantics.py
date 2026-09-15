import xml.etree.ElementTree as ET

import pytest

from test_v3_chassis_semantics import joint_transform, mesh_points_in_footprint, transform_point
from test_v3_description_semantics import render_urdf


@pytest.mark.parametrize("variant", ("gripper", "suction"))
def test_head_has_two_source_axes_and_replaces_the_legacy_head(variant):
    robot = ET.fromstring(render_urdf(variant))
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    mount = joints["head_mount_fixed"]
    yaw = joints["head_joint"]
    pitch = joints["head_pitch_joint"]
    assert mount.attrib["type"] == "fixed"
    assert mount.find("parent").attrib["link"] == "arm_carriage"
    assert mount.find("child").attrib["link"] == "head_mount"
    assert mount.find("origin").attrib == {"xyz": "0.226 0 1.3806412", "rpy": "0 0 0"}
    assert yaw.find("parent").attrib["link"] == "head_mount"
    assert yaw.find("child").attrib["link"] == "head_yaw"
    assert pitch.find("parent").attrib["link"] == "head_yaw"
    assert pitch.find("child").attrib["link"] == "head"
    assert yaw.find("origin").attrib == {"xyz": "0 0 0.005", "rpy": "0 0 0"}
    assert pitch.find("origin").attrib == {
        "xyz": "0.04 0.005 0.05", "rpy": "-1.5707963 0 0"
    }
    for joint in (yaw, pitch):
        assert joint.attrib["type"] == "revolute"
        assert joint.find("axis").attrib["xyz"] == "0 0 1"
        assert joint.find("limit").attrib == {
            "lower": "-1.57", "upper": "1.57", "effort": "10", "velocity": "5"
        }
    for name, mass in (("head_mount", 0.11917571), ("head_yaw", 0.15476448), ("head", 0.19975472)):
        assert float(links[name].find("inertial/mass").attrib["value"]) == mass
        for mesh in links[name].findall(".//mesh"):
            assert mesh.attrib["filename"].startswith("package://alfa_robot_description/meshes/head/")
    assert len(links["head"].findall("visual")) == 3
    assert not any("part_021_solid_021.stl" in mesh.attrib["filename"] for mesh in robot.findall(".//mesh"))


@pytest.mark.parametrize("variant", ("gripper", "suction"))
def test_head_mount_contacts_the_carriage_plate(variant):
    robot = ET.fromstring(render_urdf(variant))
    base = list(mesh_points_in_footprint(robot, "head_mount", "part_004_part.stl"))
    plate = list(mesh_points_in_footprint(robot, "arm_carriage", "part_002_solid_002.stl"))
    assert abs(min(p[2] for p in base) - max(p[2] for p in plate)) < 2e-6
    for axis in (0, 1):
        assert min(p[axis] for p in plate) <= min(p[axis] for p in base)
        assert max(p[axis] for p in base) <= max(p[axis] for p in plate)


def test_head_yaw_and_pitch_rotate_about_orthogonal_physical_axes():
    robot = ET.fromstring(render_urdf())
    yaw = robot.find("joint[@name='head_joint']")
    pitch = robot.find("joint[@name='head_pitch_joint']")
    yaw_rotation, _ = joint_transform(yaw)
    pitch_rotation, _ = joint_transform(pitch)
    assert transform_point((0, 0, 1), yaw_rotation, (0, 0, 0)) == pytest.approx((0, 0, 1), abs=1e-7)
    assert transform_point((0, 0, 1), pitch_rotation, (0, 0, 0)) == pytest.approx((0, 1, 0), abs=1e-7)
    # Both a point on the rotating plate and the sensor payload turn with yaw;
    # changing pitch only moves the sensor payload, leaving the yaw plate fixed.
    camera = "part_003_ZED_X_Wide.stl"
    zero_camera = next(mesh_points_in_footprint(robot, "head", camera))
    yaw_camera = next(mesh_points_in_footprint(robot, "head", camera, {"head_joint": 0.5}))
    pitch_camera = next(mesh_points_in_footprint(robot, "head", camera, {"head_pitch_joint": 0.5}))
    assert yaw_camera != pytest.approx(zero_camera)
    assert pitch_camera != pytest.approx(zero_camera)
    yaw_plate_zero = next(mesh_points_in_footprint(robot, "head_yaw", "part_001_part.stl"))
    yaw_plate_pitch = next(mesh_points_in_footprint(robot, "head_yaw", "part_001_part.stl", {"head_pitch_joint": 0.5}))
    assert yaw_plate_zero == yaw_plate_pitch
