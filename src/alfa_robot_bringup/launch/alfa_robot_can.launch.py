# Copyright (c) 2025, b»robotized
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Launch file for real CAN hardware (leftjoint2,3,4).
Uses AlfaRobotHW with 0x92 read / 0xA4 position control protocol.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("alfa_robot_bringup"),
                                "launch",
                                "alfa_robot.launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "use_mock_hardware": "false",
                    "controllers_file": "alfa_robot_controllers_can.yaml",
                    "robot_controller": "forward_position_controller",
                    "can_interface": "can0",
                    "max_speed_dps": "360",
                }.items(),
            )
        ]
    )
