import os
from glob import glob

from setuptools import setup


package_name = "robot_motion_curobo"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yml")),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Sevenova Motion Control Team",
    maintainer_email="motion@example.com",
    description="Persistent optional cuRobo joint-segment planning service.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "joint_segment_planner_node = robot_motion_curobo.joint_segment_planner_node:main",
            "segment_benchmark_client = robot_motion_curobo.segment_benchmark_client:main",
            "cspace_tuning_benchmark = robot_motion_curobo.cspace_tuning_benchmark:main",
            "collision_model_visualizer = robot_motion_curobo.collision_model_visualizer:main",
            "snapshot_collision_sphere_visualizer = robot_motion_curobo.snapshot_collision_sphere_visualizer:main",
            "rejected_trajectory_dual_scene_visualizer = robot_motion_curobo.rejected_trajectory_dual_scene_visualizer:main",
            "payload_sphere_fit_visualizer = robot_motion_curobo.payload_sphere_fit_visualizer:main",
        ],
    },
)
