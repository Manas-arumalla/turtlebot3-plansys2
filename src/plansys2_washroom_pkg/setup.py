from setuptools import setup, find_packages
import os

package_name = 'plansys2_washroom_pkg'

# include pddl, config, maps and launch files into share/<package>
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# include config/pddl/maps/launch
for subdir in ('config', 'pddl', 'maps', 'launch'):
    dirpath = subdir
    if os.path.isdir(dirpath):
        files = []
        for root, _, filenames in os.walk(dirpath):
            for f in filenames:
                files.append(os.path.join(root, f))
        if files:
            dest = os.path.join('share', package_name, subdir)
            data_files.append((dest, files))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manas',
    maintainer_email='manas@todo.todo',
    description='Washroom PlanSys2 bridge and fake Nav2 servers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plansys2_executor_bridge = plansys2_washroom.plansys2_executor_bridge:main',
            'fake_nav2_server = plansys2_washroom.fake_tools.fake_nav2_server:main',
            'fake_nav2_server_reject_commode = plansys2_washroom.fake_tools.fake_nav2_server_reject_commode:main',
        ],
    },
)

