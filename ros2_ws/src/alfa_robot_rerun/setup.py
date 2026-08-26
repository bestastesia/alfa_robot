import os
from glob import glob
from setuptools import setup

package_name = 'alfa_robot_rerun'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='dev@example.com',
    description='Shared Rerun URDF/FK visualization module and ROS joint-state viewers for ALFA robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'basic_robot_viewer = alfa_robot_rerun.basic_robot_viewer:main',
            'rerun_joint_state_viewer_node = alfa_robot_rerun.joint_state_viewer_node:main',
            'v3_redundant_solution_family_viewer = alfa_robot_rerun.v3_redundant_solution_family_viewer:main',
            'v3_single_arm_box_extract_viewer = alfa_robot_rerun.v3_single_arm_box_extract_viewer:main',
            'v3_dual_arm_cartesian_box_viewer = alfa_robot_rerun.v3_dual_arm_cartesian_box_viewer:main',
            'visualize_rerun = alfa_robot_rerun.visualize_rerun:main',
        ],
    },
)
