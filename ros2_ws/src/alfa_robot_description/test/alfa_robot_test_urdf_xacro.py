import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory


def test_urdf_xacro():
    package_path = get_package_share_directory("alfa_robot_description")
    description_path = os.path.join(package_path, "urdf", "alfa_robot.urdf.xacro")
    positions_path = os.path.join(package_path, "config", "initial_positions.yaml")
    file_descriptor, output_path = tempfile.mkstemp(suffix=".urdf")
    os.close(file_descriptor)

    try:
        with open(output_path, "w", encoding="utf-8") as output_file:
            xacro_process = subprocess.run(
                [shutil.which("xacro"), description_path],
                stdout=output_file,
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
        movable = {name for name, joint in joints.items() if joint.attrib["type"] != "fixed"}

        assert joints["world_to_base"].find("parent").attrib["link"] == "world"
        assert joints["world_to_base"].find("child").attrib["link"] == "base_footprint"
        assert joints["base_footprint_to_base_link"].find("origin").attrib["xyz"] == (
            "0.190000002779484 -0.0000442724271391554 0.40000250599116"
        )
        assert joints["base_to_model"].find("origin").attrib["xyz"] == (
            "-0.190000017371 6.50500004219e-09 0.662499964909"
        )
        assert joints["updown"].find("limit").attrib["lower"] == "-1"
        assert joints["updown"].find("limit").attrib["upper"] == "0"
        assert "head_pitch_joint" in joints
        assert "active_suspension_joint" in joints
        assert all(f"caster{index:02d}_joint" in joints for index in range(1, 5))
        assert all(f"wheel{index:02d}_joint" in joints for index in range(1, 5))
        assert not any("moving_jaw" in name for name in links | joints.keys())

        ros2_control = robot.find("ros2_control")
        assert ros2_control is not None
        control_joints = {joint.attrib["name"] for joint in ros2_control.findall("joint")}
        assert len(movable) == len(control_joints) == 26
        assert movable == control_joints

        with open(positions_path, encoding="utf-8") as positions_file:
            initial_positions = yaml.safe_load(positions_file)["initial_positions"]
        assert set(initial_positions) == movable
        for joint_name, expected_value in initial_positions.items():
            initial_value = ros2_control.find(
                f"joint[@name='{joint_name}']/state_interface[@name='position']/"
                "param[@name='initial_value']"
            )
            assert initial_value is not None
            assert abs(float(initial_value.text) - expected_value) < 1e-12
    finally:
        os.remove(output_path)
