from glob import glob
from setuptools import find_packages, setup

package_name = 'alfa_robot_plc_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ALFA Robot',
    maintainer_email='231055558@qq.com',
    description='ROS2 bridge between ALFA robot trajectories and the PLC Modbus interface.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plc_bridge_node = alfa_robot_plc_bridge.plc_bridge_node:main',
            'plc_safety_node = alfa_robot_plc_bridge.plc_safety_node:main',
        ],
    },
)
