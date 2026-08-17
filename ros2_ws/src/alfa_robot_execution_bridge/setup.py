import os
from glob import glob

from setuptools import setup


package_name = 'alfa_robot_execution_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='li',
    maintainer_email='231055558@qq.com',
    description='Unified ALFA robot trajectory execution bridge with a mock backend.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'execution_bridge_node = alfa_robot_execution_bridge.execution_bridge_node:main',
            'joint_teach_pendant = alfa_robot_execution_bridge.joint_teach_pendant:main',
            'mock_execution_node = alfa_robot_execution_bridge.mock_execution_node:main',
            'send_mock_trajectory = alfa_robot_execution_bridge.send_mock_trajectory:main',
        ],
    },
)
