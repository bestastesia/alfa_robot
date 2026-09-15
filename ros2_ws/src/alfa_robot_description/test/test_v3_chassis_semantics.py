import math
import struct
import xml.etree.ElementTree as ET

import pytest

from test_v3_description_semantics import PACKAGE_ROOT, render_urdf, rpy_matrix


def joint_transform(joint, position=0.0):
    origin = joint.find("origin")
    translation = tuple(float(v) for v in origin.attrib["xyz"].split())
    rotation = rpy_matrix(tuple(float(v) for v in origin.attrib["rpy"].split()))
    if position:
        assert joint.find("axis").attrib["xyz"] == "0 0 1"
        if joint.attrib["type"] == "prismatic":
            local_motion = (0.0, 0.0, position)
            translation = tuple(
                translation[i] + sum(rotation[i][k] * local_motion[k] for k in range(3))
                for i in range(3)
            )
        else:
            motion = rpy_matrix((0.0, 0.0, position))
            rotation = tuple(
                tuple(sum(rotation[i][k] * motion[k][j] for k in range(3)) for j in range(3)
                )
                for i in range(3)
            )
    return rotation, translation


def transform_point(point, rotation, translation):
    return tuple(
        sum(rotation[i][k] * point[k] for k in range(3)) + translation[i]
        for i in range(3)
    )


def mesh_points_in_footprint(robot, link_name, mesh_name, positions=None):
    positions = positions or {}
    parents = {joint.find("child").attrib["link"]: joint for joint in robot.findall("joint")}
    link = robot.find(f"link[@name='{link_name}']")
    visual = next(
        visual for visual in link.findall("visual")
        if visual.find("geometry/mesh").attrib["filename"].endswith(mesh_name)
    )
    mesh = visual.find("geometry/mesh")
    path = PACKAGE_ROOT / mesh.attrib["filename"].removeprefix("package://alfa_robot_description/")
    data = path.read_bytes()
    count = struct.unpack_from("<I", data, 80)[0]
    assert len(data) == 84 + 50 * count
    scale = tuple(float(v) for v in mesh.attrib["scale"].split())
    transforms = [joint_transform(visual)]
    while link_name != "base_footprint":
        joint = parents[link_name]
        transforms.append(joint_transform(joint, positions.get(joint.attrib["name"], 0.0)))
        link_name = joint.find("parent").attrib["link"]
    for triangle in struct.iter_unpack("<12fH", data[84:]):
        for offset in (3, 6, 9):
            point = tuple(triangle[offset + i] * scale[i] for i in range(3))
            for rotation, translation in transforms:
                point = transform_point(point, rotation, translation)
            yield point


def link_origin_in_footprint(robot, link_name, positions=None):
    positions = positions or {}
    parents = {joint.find("child").attrib["link"]: joint for joint in robot.findall("joint")}
    point = (0.0, 0.0, 0.0)
    while link_name != "base_footprint":
        joint = parents[link_name]
        rotation, translation = joint_transform(joint, positions.get(joint.attrib["name"], 0.0))
        point = transform_point(point, rotation, translation)
        link_name = joint.find("parent").attrib["link"]
    return point


@pytest.mark.parametrize("variant", ("gripper", "suction"))
def test_chassis_tree_preserves_active_suspension_topology(variant):
    robot = ET.fromstring(render_urdf(variant))
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    assert joints["base_to_chassis"].find("parent").attrib["link"] == "base_link"
    assert joints["base_to_chassis"].find("child").attrib["link"] == "chassis_base"

    suspension = joints["active_suspension_joint"]
    assert suspension.attrib["type"] == "prismatic"
    assert suspension.find("parent").attrib["link"] == "chassis_base"
    assert suspension.find("child").attrib["link"] == "active_suspension_carriage"
    assert suspension.find("axis").attrib["xyz"] == "0 0 1"
    assert float(suspension.find("limit").attrib["lower"]) == -0.15
    assert float(suspension.find("limit").attrib["upper"]) == 0.0

    caster_parents = {
        1: "chassis_base",
        2: "active_suspension_carriage",
        3: "active_suspension_carriage",
        4: "chassis_base",
    }
    for index in range(1, 5):
        caster = joints[f"caster{index:02d}_joint"]
        wheel = joints[f"wheel{index:02d}_joint"]
        assert caster.find("parent").attrib["link"] == caster_parents[index]
        assert caster.find("child").attrib["link"] == f"caster{index:02d}"
        assert wheel.find("parent").attrib["link"] == f"caster{index:02d}"
        assert wheel.find("child").attrib["link"] == f"wheel{index:02d}"
        for joint in (caster, wheel):
            assert joint.attrib["type"] == "revolute"
            assert joint.find("axis").attrib["xyz"] == "0 0 1"
            assert float(joint.find("limit").attrib["lower"]) == -3.14159
            assert float(joint.find("limit").attrib["upper"]) == 3.14159
            assert float(joint.find("limit").attrib["effort"]) == 10.0
            assert float(joint.find("limit").attrib["velocity"]) == 1.0


@pytest.mark.parametrize("variant", ("gripper", "suction"))
def test_wheels_ground_and_column_mounting_faces_meet(variant):
    robot = ET.fromstring(render_urdf(variant))
    wheel_centers = [link_origin_in_footprint(robot, f"wheel{index:02d}") for index in range(1, 5)]
    average_center = tuple(sum(point[axis] for point in wheel_centers) / 4.0 for axis in range(3))
    assert abs(average_center[0]) < 1e-9
    assert abs(average_center[1]) < 1e-9
    assert abs(average_center[2] - 0.1) < 1e-9
    for index in range(1, 5):
        mesh = f"wheel{index:02d}.stl"
        for steering in (0.0, 0.7):
            points = list(mesh_points_in_footprint(
                robot, f"wheel{index:02d}", mesh, {f"caster{index:02d}_joint": steering}
            ))
            assert abs(min(p[2] for p in points)) < 5e-6
            assert abs(max(p[2] for p in points) - 0.2) < 2e-5
    for index in (2, 3):
        points = list(mesh_points_in_footprint(
            robot, f"wheel{index:02d}", f"wheel{index:02d}.stl",
            {"active_suspension_joint": -0.15},
        ))
        assert abs(min(p[2] for p in points) + 0.15) < 5e-6
    plate = list(mesh_points_in_footprint(robot, "chassis_base", "part_001_solid_001.stl"))
    column = list(mesh_points_in_footprint(robot, "model_base", "part_001_solid_001.stl"))
    assert abs(min(p[2] for p in plate) - 0.32000250599116) < 2e-6
    assert abs(max(p[2] for p in plate) - min(p[2] for p in column)) < 2e-6


@pytest.mark.parametrize("variant", ("gripper", "suction"))
def test_legacy_chassis_is_replaced_without_mesh_or_mass_duplication(variant):
    robot = ET.fromstring(render_urdf(variant))
    upper = robot.find("link[@name='model_base']")
    assert len(upper.findall("visual")) == len(upper.findall("collision")) == 10
    for mesh in upper.findall(".//mesh"):
        assert int(mesh.attrib["filename"].split("part_")[1][:3]) < 38
    assert float(upper.find("inertial/mass").attrib["value"]) == pytest.approx(30.1449868854)
    chassis_mass = 0.0
    names = ["chassis_base", "active_suspension_carriage"]
    names += [f"{kind}{i:02d}" for kind in ("caster", "wheel") for i in range(1, 5)]
    for name in names:
        link = robot.find(f"link[@name='{name}']")
        chassis_mass += float(link.find("inertial/mass").attrib["value"])
        for mesh in link.findall(".//mesh"):
            expected = "meshes/chassis/" if name == "chassis_base" else "meshes/active_suspension/"
            assert mesh.attrib["filename"].startswith(f"package://alfa_robot_description/{expected}")
    assert chassis_mass == pytest.approx(55.40394687)
