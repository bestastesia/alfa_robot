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

        cube = links["base_link"].find("visual/geometry/box")
        assert cube is not None
        assert cube.attrib["size"] == "0.4 0.5 0.5"

        expected_mounts = {
            "left": ("0 0.255 0.25", "-1.57079633 -1.57079633 0"),
            "right": ("0 -0.255 0.25", "1.57079633 1.57079633 0"),
        }
        expected_limits = {
            1: (-3.14159265, 3.14159265),
            2: (-1.83259571, 1.83259571),
            3: (-3.14159265, 3.14159265),
            4: (-2.35619449, 2.35619449),
            5: (-3.14159265, 3.14159265),
            6: (-2.09439510, 2.09439510),
            7: (-3.14159265, 3.14159265),
        }
        for side, (expected_xyz, expected_rpy) in expected_mounts.items():
            mount_origin = joints[f"{side}_arm_mount"].find("origin")
            assert mount_origin.attrib["xyz"] == expected_xyz
            assert mount_origin.attrib["rpy"] == expected_rpy

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
                assert "/meshes/robot_v3_0_4/" in visual_mesh.attrib["filename"]
                assert visual_mesh.attrib["filename"] == collision_mesh.attrib["filename"]
                assert visual_mesh.attrib["scale"] == "0.001 0.001 0.001"

            assert joints[f"{side}_tool0_fixed"].find("parent").attrib["link"] == f"{side}_joint7"

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
