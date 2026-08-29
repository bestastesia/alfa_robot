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
        head_meshes = links["head"].findall("visual/geometry/mesh")
        assert len(base_meshes) == 19
        assert len(carriage_meshes) == 8
        assert len(head_meshes) == 1
        assert all(
            "/meshes/robot_v3_0_8/" in mesh.attrib["filename"]
            for mesh in base_meshes + carriage_meshes + head_meshes
        )

        visuals = robot.findall(".//visual")
        collisions = robot.findall(".//collision")
        assert len(visuals) == 42
        assert len(collisions) == 42
        assert all(
            visual.find("material/color").attrib["rgba"] == "0.82 0.82 0.82 1"
            for visual in visuals
        )

        model_joint = joints["base_to_model"]
        assert model_joint.find("parent").attrib["link"] == "base_link"
        assert model_joint.find("child").attrib["link"] == "model_base"
        assert model_joint.find("origin").attrib["xyz"] == (
            "-0.190000017371 6.50500004219e-09 0.648299964909"
        )
        assert model_joint.find("origin").attrib["rpy"] == (
            "1.5707963 0 1.57079632679"
        )
        assert joints["world_to_base"].find("origin").attrib["xyz"] == "0 0 0"

        updown_joint = joints["updown"]
        assert updown_joint.attrib["type"] == "prismatic"
        assert updown_joint.find("parent").attrib["link"] == "model_base"
        assert updown_joint.find("child").attrib["link"] == "arm_carriage"
        assert updown_joint.find("origin").attrib["xyz"] == (
            "-6.505e-09 -0.64829997 0.19"
        )
        assert updown_joint.find("axis").attrib["xyz"] == "0 0 1"
        assert updown_joint.find("limit").attrib["lower"] == "0"
        assert updown_joint.find("limit").attrib["upper"] == "0.7"
        head_joint = joints["head_joint"]
        assert head_joint.attrib["type"] == "revolute"
        assert head_joint.find("parent").attrib["link"] == "arm_carriage"
        assert head_joint.find("child").attrib["link"] == "head"
        assert head_joint.find("axis").attrib["xyz"] == "0 0 1"
        expected_limits = {
            1: (-3.14159265, 3.14159265),
            2: (-1.83259571, 1.83259571),
            3: (-3.14159265, 3.14159265),
            4: (-2.53072742, 2.53072742),
            5: (-3.14159265, 3.14159265),
            6: (-1.91986218, 1.91986218),
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
                assert "/meshes/robot_v3_0_8/" in visual_mesh.attrib["filename"]
                assert visual_mesh.attrib["filename"] == collision_mesh.attrib["filename"]
                assert visual_mesh.attrib["scale"] == "0.001 0.001 0.001"

            arm_mount = joints[f"{side}_arm_mount"]
            assert arm_mount.find("parent").attrib["link"] == "arm_carriage"
            assert arm_mount.find("child").attrib["link"] == f"{side}_arm_base"
            assert joints[f"{side}_joint1"].find("parent").attrib["link"] == f"{side}_arm_base"
            assert joints[f"{side}_tool0_fixed"].find("parent").attrib["link"] == f"{side}_joint7"

        assert joints["left_tool0_fixed"].find("origin").attrib["xyz"] == "0 0 0.13585"
        assert joints["right_tool0_fixed"].find("origin").attrib["xyz"] == "0 0 0.13585"

        left_joint1_origin = joints["left_joint1"].find("origin").attrib["xyz"]
        right_joint1_origin = joints["right_joint1"].find("origin").attrib["xyz"]
        assert left_joint1_origin == "0.181 -0.33054221 1.3032993"
        assert right_joint1_origin == "0.181 0.35372443 1.309511"

        ros2_control = robot.find("ros2_control")
        assert ros2_control is not None
        control_joints = [joint.attrib["name"] for joint in ros2_control.findall("joint")]
        assert len(control_joints) == 16
        assert set(control_joints) == {"updown", "head_joint"} | {
            f"{side}_joint{index}"
            for side in ("left", "right")
            for index in range(1, 8)
        }
        for side in ("left", "right"):
            joint = ros2_control.find(f"joint[@name='{side}_joint6']")
            position_interface = joint.find("command_interface[@name='position']")
            params = {
                param.attrib["name"]: float(param.text)
                for param in position_interface.findall("param")
            }
            assert abs(params["min"] + 1.91986218) < 1e-8
            assert abs(params["max"] - 1.91986218) < 1e-8
    finally:
        os.remove(output_path)
