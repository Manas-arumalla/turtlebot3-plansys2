import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'washbot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manas Reddy Arumalla',
    maintainer_email='manasreddyarumalla@gmail.com',
    description=(
        'Mission controller, world model, action handlers and simulation '
        'tools for WashBot cleaning missions.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_controller = washbot_control.mission_controller:main',
            'fake_nav2_server = washbot_control.sim.fake_nav2_server:main',
            'initial_pose_publisher = washbot_control.sim.initial_pose:main',
            'pose_recorder = washbot_control.sim.pose_recorder:main',
        ],
    },
)
