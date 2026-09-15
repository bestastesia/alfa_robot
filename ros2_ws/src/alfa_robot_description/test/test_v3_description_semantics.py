import math
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def rpy_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = ((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr))
    ry = ((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp))
    rz = ((cy, -sy, 0.0), (sy, cy, 0.0), (0.0, 0.0, 1.0))
    return matrix_multiply(matrix_multiply(rz, ry), rx)


def matrix_multiply(lhs, rhs):
    return tuple(
        tuple(sum(lhs[row][k] * rhs[k][column] for k in range(3)) for column in range(3))
        for row in range(3)
    )


def assert_matrix_close(actual, expected, tolerance=1e-7):
    for row in range(3):
        for column in range(3):
            assert abs(actual[row][column] - expected[row][column]) < tolerance


def render_urdf(end_effector="gripper"):
    result = subprocess.run(
        [
            "xacro", str(PACKAGE_ROOT / "urdf" / "alfa_robot.urdf.xacro"),
            f"end_effector:={end_effector}",
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    assert result.returncode == 0, result.stderr
    return result.stdout


@pytest.mark.parametrize("end_effector", ("gripper", "suction"))
def test_v3_side_zero_and_group_semantics(end_effector):
    urdf_text = render_urdf(end_effector)
    robot = ET.fromstring(urdf_text)
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}

    footprint = joints["base_footprint_to_base_link"]
    assert footprint.find("parent").attrib["link"] == "base_footprint"
    assert footprint.find("child").attrib["link"] == "base_link"
    assert footprint.find("origin").attrib["xyz"] == (
        "0.190000002779484 -0.0000442724271391554 0.40000250599116"
    )

    # ROS base coordinates use +Y as robot left.
    mount_rotation = rpy_matrix((0.0, 0.0, math.pi / 2.0))
    left_origin = tuple(float(value) for value in joints["left_joint1"].find("origin").attrib["xyz"].split())
    right_origin = tuple(float(value) for value in joints["right_joint1"].find("origin").attrib["xyz"].split())
    left_in_carriage = tuple(sum(mount_rotation[row][column] * left_origin[column] for column in range(3)) for row in range(3))
    right_in_carriage = tuple(sum(mount_rotation[row][column] * right_origin[column] for column in range(3)) for row in range(3))
    assert left_in_carriage[1] > 0.0
    assert right_in_carriage[1] < 0.0

    updown = joints["updown"]
    assert updown.find("origin").attrib["xyz"] == "-6.50744829443e-09 -0.64829997 0.19"
    assert float(updown.find("limit").attrib["lower"]) == -1.0
    assert float(updown.find("limit").attrib["upper"]) == 0.0

    exact_joint2_limit = math.radians(105.0)
    for side in ("left", "right"):
        limit = joints[f"{side}_joint2"].find("limit")
        assert abs(float(limit.attrib["lower"]) + exact_joint2_limit) < 1e-15
        assert abs(float(limit.attrib["upper"]) - exact_joint2_limit) < 1e-15

    assert joints["left_joint5"].find("origin").attrib == {
        "xyz": "0.22457775 0.060175428 0.068",
        "rpy": "1.5707963 0 1.8325957",
    }
    assert joints["left_joint7"].find("origin").attrib == {
        "xyz": "0.0993 0 -0.0615",
        "rpy": "0 -1.5707963 3.1415927",
    }
    for name in ("left_joint5", "left_joint7"):
        limit = joints[name].find("limit")
        assert float(limit.attrib["lower"]) == -3.14159265
        assert float(limit.attrib["upper"]) == 3.14159265

    suffix = f"_{end_effector}"
    initial_positions = yaml.safe_load(
        (PACKAGE_ROOT / "config" / f"initial_positions{suffix}.yaml").read_text()
    )["initial_positions"]
    expected_initial_positions = {
        "updown": 0.0,
        "head_joint": 0.0,
        "head_pitch_joint": 0.0,
        "left_joint1": math.radians(150.0),
        "left_joint2": math.radians(90.0),
        "left_joint3": math.radians(-5.0),
        "left_joint4": math.radians(120.0),
        "left_joint5": 0.0,
        "left_joint6": 0.0,
        "left_joint7": 0.0,
        "right_joint1": math.radians(-150.0),
        "right_joint2": math.radians(-90.0),
        "right_joint3": math.radians(5.0),
        "right_joint4": math.radians(-120.0),
        "right_joint5": 0.0,
        "right_joint6": 0.0,
        "right_joint7": 0.0,
    }
    expected_initial_positions.update({
        f"{kind}{index:02d}_joint": 0.0
        for kind in ("caster", "wheel") for index in range(1, 5)
    })
    expected_initial_positions["active_suspension_joint"] = 0.0
    if end_effector == "gripper":
        expected_initial_positions.update({
            "left_moving_jaw_joint": 0.0,
            "right_moving_jaw_joint": 0.0,
        })
    assert set(initial_positions) == set(expected_initial_positions)
    for joint_name, expected in expected_initial_positions.items():
        assert abs(float(initial_positions[joint_name]) - expected) < 1e-10

    with tempfile.NamedTemporaryFile("w", suffix=".urdf") as output:
        output.write(urdf_text)
        output.flush()
        checked = subprocess.run(
            ["check_urdf", output.name], capture_output=True, text=True, check=False
        )
    assert checked.returncode == 0, checked.stderr


@pytest.mark.parametrize("side", ("left", "right"))
def test_dual_gripper_attachment_and_travel(side):
    robot = ET.fromstring(render_urdf())
    joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
    links = {link.attrib["name"]: link for link in robot.findall("link")}
    jaw = joints[f"{side}_moving_jaw_joint"]

    # Source gripper names are reversed relative to the ROS physical sides.
    assert jaw.attrib["type"] == "prismatic"
    assert jaw.find("parent").attrib["link"] == f"{side}_joint7"
    assert jaw.find("child").attrib["link"] == f"{side}_moving_jaw"
    assert jaw.find("origin").attrib == {"xyz": "0 0 0", "rpy": "0 0 0"}
    assert jaw.find("axis").attrib["xyz"] == "1 0 0"
    assert float(jaw.find("limit").attrib["lower"]) == 0.0
    assert float(jaw.find("limit").attrib["upper"]) == 0.080

    # Opening the jaw must not move the existing arm planning tip.
    tool = joints[f"{side}_tool0_fixed"]
    assert tool.attrib["type"] == "fixed"
    assert tool.find("parent").attrib["link"] == f"{side}_joint7"
    assert tool.find("origin").attrib == {"xyz": "0 0 0.13585", "rpy": "0 0 0"}

    fixed_body = links[f"{side}_joint7"]
    moving_jaw = links[f"{side}_moving_jaw"]
    assert float(fixed_body.find("inertial/mass").attrib["value"]) == 4.750
    assert float(moving_jaw.find("inertial/mass").attrib["value"]) == 0.342
    for link, mesh_name in ((fixed_body, "ee_fixed_body"), (moving_jaw, "moving_jaw")):
        for geometry in ("visual", "collision"):
            assert len(link.findall(geometry)) == 1
            assert link.find(f"{geometry}/geometry/mesh").attrib["filename"] == (
                f"package://alfa_robot_description/meshes/robot_v3/{mesh_name}_{geometry}.stl"
            )
        assert link.find("visual/origin").attrib == {"xyz": "0 0 0", "rpy": "0 0 0"}
    assert fixed_body.find("collision/origin").attrib == {
        "xyz": "-0.040201 -0.071594 0", "rpy": "0 0 0"
    }
    assert moving_jaw.find("collision/origin").attrib == {"xyz": "0 0 0", "rpy": "0 0 0"}


@pytest.mark.parametrize("end_effector", ("gripper", "suction"))
def test_all_movable_joints_have_consistent_configuration(end_effector):
    robot = ET.fromstring(render_urdf(end_effector))
    movable = {
        joint.attrib["name"]: joint
        for joint in robot.findall("joint")
        if joint.attrib["type"] != "fixed"
    }
    suffix = f"_{end_effector}"
    positions = yaml.safe_load(
        (PACKAGE_ROOT / "config" / f"initial_positions{suffix}.yaml").read_text()
    )["initial_positions"]
    limits = yaml.safe_load(
        (PACKAGE_ROOT / "config" / f"joint_limits{suffix}.yaml").read_text()
    )["joints"]
    assert len(movable) == (28 if end_effector == "gripper" else 26)
    assert set(movable) == set(positions) == set(limits)
    for name, joint in movable.items():
        unit = "m" if joint.attrib["type"] == "prismatic" else "rad"
        lower = float(joint.find("limit").attrib["lower"])
        upper = float(joint.find("limit").attrib["upper"])
        assert limits[name]["type"] == joint.attrib["type"]
        assert limits[name][f"lower_position_{unit}"] == lower
        assert limits[name][f"upper_position_{unit}"] == upper
        assert lower <= positions[name] <= upper


@pytest.mark.parametrize("end_effector", ("gripper", "suction"))
def test_mesh_resources_resolve_with_millimeter_scale(end_effector):
    robot = ET.fromstring(render_urdf(end_effector))
    prefix = "package://alfa_robot_description/"
    for mesh in robot.findall(".//mesh"):
        filename = mesh.attrib["filename"]
        assert filename.startswith(prefix)
        path = PACKAGE_ROOT / filename.removeprefix(prefix)
        assert path.is_file(), filename
        assert path.stat().st_size > 0, filename
        assert mesh.attrib["scale"] == "0.001 0.001 0.001"


@pytest.mark.parametrize("side", ("left", "right"))
def test_suction_replaces_gripper_with_source_mass_and_collision_offset(side):
    robot = ET.fromstring(render_urdf("suction"))
    assert not any("moving_jaw" in e.attrib.get("name", "") for e in robot)
    link = robot.find(f"link[@name='{side}_joint7']")
    assert float(link.find("inertial/mass").attrib["value"]) == 2.618
    assert link.find("inertial/origin").attrib == {
        "xyz": "-0.000330 -0.000125 0.070303", "rpy": "0 0 0"
    }
    for geometry in ("visual", "collision"):
        assert len(link.findall(geometry)) == 1
        assert link.find(f"{geometry}/geometry/mesh").attrib["filename"] == (
            f"package://alfa_robot_description/meshes/robot_v3/suction_{geometry}.stl"
        )
    assert link.find("visual/origin").attrib == {"xyz": "0 0 0", "rpy": "0 0 0"}
    assert link.find("collision/origin").attrib == {
        "xyz": "-0.158443333 -0.088374996 0", "rpy": "0 0 0"
    }
    tool = robot.find(f"joint[@name='{side}_tool0_fixed']")
    assert tool.attrib["type"] == "fixed"
    assert tool.find("parent").attrib["link"] == f"{side}_joint7"
    assert tool.find("origin").attrib == {"xyz": "0 0 0.13585", "rpy": "0 0 0"}


def test_end_effector_switch_preserves_common_robot():
    def common_elements(variant):
        robot = ET.fromstring(render_urdf(variant))
        return {
            (element.tag, element.attrib["name"]): ET.canonicalize(ET.tostring(element), strip_text=True)
            for element in robot
            if element.tag != "ros2_control"
            and "moving_jaw" not in element.attrib["name"]
            and not (element.tag == "link" and element.attrib["name"] in ("left_joint7", "right_joint7"))
        }

    assert common_elements("suction") == common_elements("gripper")


def test_variant_entrypoints_and_branch_defaults():
    for variant in ("gripper", "suction"):
        result = subprocess.run(
            ["xacro", str(PACKAGE_ROOT / "urdf" / f"alfa_robot_dual_{variant}.urdf.xacro")],
            capture_output=True, text=True, check=True,
        )
        assert ET.canonicalize(result.stdout, strip_text=True) == ET.canonicalize(
            render_urdf(variant), strip_text=True
        )

    positions = yaml.safe_load((PACKAGE_ROOT / "config" / "initial_positions.yaml").read_text())
    variant = positions["metadata"]["end_effector"]
    for stem in ("initial_positions", "joint_limits"):
        default = yaml.safe_load((PACKAGE_ROOT / "config" / f"{stem}.yaml").read_text())
        explicit = yaml.safe_load((PACKAGE_ROOT / "config" / f"{stem}_{variant}.yaml").read_text())
        assert default == explicit
    rendered = subprocess.run(
        ["xacro", str(PACKAGE_ROOT / "urdf" / "alfa_robot.urdf.xacro")],
        capture_output=True, text=True, check=True,
    ).stdout
    assert ET.canonicalize(rendered, strip_text=True) == ET.canonicalize(
        render_urdf(variant), strip_text=True
    )


def test_unknown_end_effector_is_rejected():
    result = subprocess.run(
        ["xacro", str(PACKAGE_ROOT / "urdf" / "alfa_robot.urdf.xacro"), "end_effector:=invalid"],
        capture_output=True, text=True, check=False,
    )
    assert result.returncode != 0
