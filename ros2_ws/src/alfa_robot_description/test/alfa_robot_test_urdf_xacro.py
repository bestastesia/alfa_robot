# Copyright (c) 2025, b»robotized
# Copyright (c) 2022 FZI Forschungszentrum Informatik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the FZI Forschungszentrum Informatik nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#
# Source of this file is https://github.com/b-robotized/ros_team_workspace repository.
# Modified from tests in https://github.com/UniversalRobots/Universal_Robots_ROS2_Description
#
# Author: Lukas Sackewitz
# Author (template): Manuel Muth

import os
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory


def test_urdf_xacro():
    # General Arguments
    description_package = "alfa_robot_description"
    description_file = "alfa_robot.urdf.xacro"

    description_file_path = os.path.join(
        get_package_share_directory(description_package), "urdf", description_file
    )

    (_, tmp_urdf_output_file) = tempfile.mkstemp(suffix=".urdf")

    # Compose `xacro` and `check_urdf` command
    xacro_command = (
        f"{shutil.which('xacro')}" f" {description_file_path}" f" > {tmp_urdf_output_file}"
    )
    check_urdf_command = f"{shutil.which('check_urdf')} {tmp_urdf_output_file}"

    # Try to call processes but finally remove the temp file
    try:
        xacro_process = subprocess.run(
            xacro_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
        )

        assert xacro_process.returncode == 0, " --- XACRO command failed ---"

        check_urdf_process = subprocess.run(
            check_urdf_command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True
        )

        assert (
            check_urdf_process.returncode == 0
        ), "\n --- URDF check failed! --- \nYour xacro does not unfold into a proper urdf robot description. Please check your xacro file."

        robot = ET.parse(tmp_urdf_output_file).getroot()
        joints = {joint.attrib["name"]: joint for joint in robot.findall("joint")}
        links = {link.attrib["name"]: link for link in robot.findall("link")}
        for side, mesh_variant in (("left", "left"), ("right", "right")):
            for index in range(1, 8):
                name = f"{side}_joint{index}"
                assert name in joints
                visual_mesh = links[name].find("visual/geometry/mesh")
                collision_mesh = links[name].find("collision/geometry/mesh")
                assert visual_mesh is not None
                assert collision_mesh is not None
                expected_mesh_root = f"/meshes/robot_v3_0_2/{mesh_variant}/"
                assert expected_mesh_root in visual_mesh.attrib["filename"]
                assert expected_mesh_root in collision_mesh.attrib["filename"]
                assert visual_mesh.attrib["filename"] == collision_mesh.attrib["filename"]

            tool_joint = joints[f"{side}_tool0_fixed"]
            assert tool_joint.find("parent").attrib["link"] == f"{side}_joint7"

        expected_initial_positions = {
            "updown": 0.3,
            "left_joint1": 0.73513268,
            "left_joint2": 0.75921822,
            "left_joint3": 1.25332094,
            "left_joint4": -0.02879793,
            "left_joint5": 1.13568574,
            "left_joint6": -0.09058259,
            "left_joint7": -0.23980824,
            "right_joint1": 0.73513268,
            "right_joint2": 0.75921822,
            "right_joint3": 1.25332094,
            "right_joint4": -0.02879793,
            "right_joint5": 1.13568574,
            "right_joint6": -0.09058259,
            "right_joint7": -0.23980824,
        }
        ros2_control = robot.find("ros2_control")
        assert ros2_control is not None
        control_joints = {
            joint.attrib["name"]: joint for joint in ros2_control.findall("joint")
        }
        for name, expected in expected_initial_positions.items():
            initial_value = control_joints[name].find(
                "state_interface[@name='position']/param[@name='initial_value']"
            )
            assert initial_value is not None
            assert abs(float(initial_value.text) - expected) < 1e-9

    finally:
        os.remove(tmp_urdf_output_file)


if __name__ == "__main__":
    test_urdf_xacro()
