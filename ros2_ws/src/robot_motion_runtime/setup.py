import os
from glob import glob

from setuptools import setup


package_name = "robot_motion_runtime"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sevenova Motion Control Team",
    maintainer_email="motion@example.com",
    description="Runtime service facades, authoritative state source, and dashboard for ALFA robot motion.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motion_state_source_node = robot_motion_runtime.motion_state_source_node:main",
            "motion_scene_source_node = robot_motion_runtime.motion_scene_source_node:main",
            "dual_arm_ik_candidate_service_node = robot_motion_runtime.dual_arm_ik_candidate_service_node:main",
            "box_pair_task_adapter_node = robot_motion_runtime.box_pair_task_adapter_node:main",
            "dual_grasp_task_adapter_node = robot_motion_runtime.dual_grasp_task_adapter_node:main",
            "plan_extract_service_node = robot_motion_runtime.plan_extract_service_node:main",
            "plan_loaded_service_node = robot_motion_runtime.plan_loaded_service_node:main",
            "execute_trajectory_service_node = robot_motion_runtime.execute_trajectory_service_node:main",
            "motion_task_orchestrator_node = robot_motion_runtime.motion_task_orchestrator_node:main",
            "motion_runtime_dashboard_node = robot_motion_runtime.motion_runtime_dashboard_node:main",
            "kinematic_sim_executor_node = robot_motion_runtime.kinematic_sim_executor_node:main",
            "vehicle_pose_source_node = robot_motion_runtime.vehicle_pose_source_node:main",
        ],
    },
)
