from setuptools import setup
from glob import glob
import os

package_name = 'rx200_moveit_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Install config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='master26',
    maintainer_email='rishabh.jain.2026@mumail.ie',
    description='RX200',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = rx200_moveit_control.vision_node:main',
            'manipulation_node = rx200_moveit_control.manipulation_node:main',
            'coordinator_node = rx200_moveit_control.coordinator_node:main',
        ],
    },
)
