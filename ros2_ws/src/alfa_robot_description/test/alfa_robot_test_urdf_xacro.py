import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


def test_urdf_xacro():
    description_path = os.path.join(
        get_package_share_directory("alfa_robot_description"),
        "urdf",
        "alfa_robot.urdf.xacro",
    )
    file_descriptor, output_path = tempfile.mkstemp(suffix=".urdf")
    os.close(file_descriptor)

    try:
        xacro_process = subprocess.run(
            [shutil.which("xacro"), description_path],
            stdout=open(output_path, "w", encoding="utf-8"),
            stderr=subprocess.PIPE,
            text=True,
            check=False,
        )
        assert xacro_process.returncode == 0, xacro_process.stderr

        check_process = subprocess.run(
            [shutil.which("check_urdf"), output_path],
            capture_output=True,
            text=True,
            check=False,
        )
        assert check_process.returncode == 0, check_process.stderr

        robot = ET.parse(output_path).getroot()
        joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
        links = {link.attrib["name"]: link for link in robot.findall("link")}

        base_meshes = links["model_base"].findall("visual/geometry/mesh")
        carriage_meshes = links["arm_carriage"].findall("visual/geometry/mesh")
        assert len(base_meshes) == 17
        assert len(carriage_meshes) == 13
        assert all(
            "/meshes/robot_v3_0_5/" in mesh.attrib["filename"]
            for mesh in base_meshes + carriage_meshes
        )

        visuals = robot.findall(".//visual")
        assert len(visuals) == 44
        assert all(
            visual.find("material/color").attrib["rgba"] == "0.82 0.82 0.82 1"
            for visual in visuals
        )

        model_joint = joints["base_to_model"]
        assert model_joint.find("parent").attrib["link"] == "base_link"
        assert model_joint.find("child").attrib["link"] == "model_base"
        assert model_joint.find("origin").attrib["xyz"] == "0.13899334 0 -0.50669703"
        assert model_joint.find("origin").attrib["rpy"] == "1.57079632679 0.22548136 0"
        assert joints["world_to_base"].find("origin").attrib["xyz"] == "-0.055 0 0"

        carriage_joint = joints["arm_carriage_joint"]
        assert carriage_joint.attrib["type"] == "fixed"
        assert carriage_joint.find("parent").attrib["link"] == "model_base"
        assert carriage_joint.find("child").attrib["link"] == "arm_carriage"
        assert carriage_joint.find("origin").attrib["xyz"] == (
            "0.065760636 -0.66236699 -2.021e-08"
        )
        expected_limits = {
            1: (-3.14159265, 3.14159265),
            2: (-1.83259571, 1.83259571),
            3: (-3.14159265, 3.14159265),
            4: (-2.53072742, 2.53072742),
            5: (-3.14159265, 3.14159265),
            6: (-2.09439510, 2.09439510),
            7: (-3.14159265, 3.14159265),
        }
        for side in ("left", "right"):
            for index in range(1, 8):
                name = f"{side}_joint{index}"
                assert name in joints
                lower, upper = expected_limits[index]
                limit = joints[name].find("limit")
                assert abs(float(limit.attrib["lower"]) - lower) < 1e-8
                assert abs(float(limit.attrib["upper"]) - upper) < 1e-8
                visual_mesh = links[name].find("visual/geometry/mesh")
                collision_mesh = links[name].find("collision/geometry/mesh")
                assert visual_mesh is not None
                assert collision_mesh is not None
                assert "/meshes/robot_v3_0_5/" in visual_mesh.attrib["filename"]
                assert visual_mesh.attrib["filename"] == collision_mesh.attrib["filename"]
                assert visual_mesh.attrib["scale"] == "0.001 0.001 0.001"

            assert joints[f"{side}_joint1"].find("parent").attrib["link"] == (
                "arm_carriage"
            )
            assert joints[f"{side}_tool0_fixed"].find("parent").attrib["link"] == f"{side}_joint7"

        assert joints["left_tool0_fixed"].find("origin").attrib["xyz"] == "0 0 0.1865"
        assert joints["right_tool0_fixed"].find("origin").attrib["xyz"] == "0 0 0.1895"

        ros2_control = robot.find("ros2_control")
        assert ros2_control is not None
        control_joints = [joint.attrib["name"] for joint in ros2_control.findall("joint")]
        assert len(control_joints) == 14
        assert set(control_joints) == {
            f"{side}_joint{index}"
            for side in ("left", "right")
            for index in range(1, 8)
        }
    finally:
        os.remove(output_path)
