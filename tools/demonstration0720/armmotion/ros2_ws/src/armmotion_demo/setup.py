import os
from glob import glob

from setuptools import find_packages, setup


package_name = "armmotion_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    package_data={"armmotion_demo": ["trajectory_cache/*.json.gz"]},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Sevenova Motion Control Team",
    maintainer_email="motion@example.com",
    description="Isolated two-thread ALFA arm planning and staged execution demo.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "algorithm_thread = armmotion_demo.algorithm_thread:main",
            "controller_interpolated_rerun = armmotion_demo.controller_interpolated_rerun:main",
            "domain_motion_server = armmotion_demo.domain_motion_server:main",
            "manual_domain_task = armmotion_demo.manual_domain_task:main",
            "mock_current_rt_control = armmotion_demo.mock_current_rt_control:main",
            "task_thread = armmotion_demo.task_thread:main",
        ],
    },
)
