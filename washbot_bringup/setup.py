import os
from glob import glob

from setuptools import setup

package_name = 'washbot_bringup'

setup(
    name=package_name,
    version='1.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manas Reddy Arumalla',
    maintainer_email='manasreddyarumalla@gmail.com',
    description='Launch files, navigation config, maps and RViz profiles for WashBot.',
    license='Apache-2.0',
)
