"""Bring up PlanSys2 with the WashBot domain loaded.

Starts the PlanSys2 stack (domain expert, problem expert, planner, executor)
via its distributed bringup, with the STRIPS or temporal domain preloaded.
With this running, the mission controller can plan through PlanSys2:

    ros2 launch washbot_bringup plansys2.launch.py
    ros2 launch washbot_bringup mission_sim.launch.py planner_backend:=plansys2

Requires the ros-<distro>-plansys2-* packages (see docs/getting_started.md).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    planning_share = get_package_share_directory('washbot_planning')
    plansys2_share = get_package_share_directory('plansys2_bringup')
    default_domain = os.path.join(planning_share, 'pddl', 'domain_strips.pddl')

    return LaunchDescription([
        DeclareLaunchArgument('model_file', default_value=default_domain,
                              description='PDDL domain to load into PlanSys2'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                plansys2_share, 'launch',
                'plansys2_bringup_launch_distributed.py')),
            launch_arguments={
                'model_file': LaunchConfiguration('model_file'),
            }.items(),
        ),
    ])
