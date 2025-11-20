from setuptools import find_packages, setup

package_name = 'rx200_moveit_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='James Florin Petri',
    maintainer_email='james.petri@mu.ie',
    description='Package used for RX200 arm control via the moveit interface',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rx200_pick_place_client=rx200_moveit_control.rx200_pick_place_client:main',
            'rx200_moveit_client=rx200_moveit_control.rx200_moveit_action_client:main'
        ],
    },
)
