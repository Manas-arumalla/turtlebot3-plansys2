import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'washbot_planning'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'pddl'), glob('pddl/*.pddl')),
        (os.path.join('share', package_name, 'pddl', 'problems'), glob('pddl/problems/*.pddl')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Manas Reddy Arumalla',
    maintainer_email='manasreddyarumalla@gmail.com',
    description=(
        'PDDL domains, planner backends, plan validation, problem generation '
        'and benchmarks for the WashBot cleaning missions.'
    ),
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'washbot_plan = washbot_planning.cli:main',
            'planner_client = washbot_planning.planner_client:main',
        ],
    },
)
