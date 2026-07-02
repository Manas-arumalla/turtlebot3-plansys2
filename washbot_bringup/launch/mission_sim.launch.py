"""Fully simulated mission: fake Nav2 server + mission controller.

Runs on any machine with ROS 2 — no robot, no Gazebo, no Nav2 stack — and
exercises the complete plan -> execute -> monitor -> replan loop.

Nominal mission:

    ros2 launch washbot_bringup mission_sim.launch.py

Blocked-corridor recovery scenario (the mission detours via the shower):

    ros2 launch washbot_bringup mission_sim.launch.py fail_edges:='[hall->commode]'

Unreachable-goal scenario (the mission exhausts routes and aborts cleanly):

    ros2 launch washbot_bringup mission_sim.launch.py fail_locations:='[commode]'
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_share = get_package_share_directory('washbot_control')
    bringup_share = get_package_share_directory('washbot_bringup')
    default_world = os.path.join(control_share, 'config', 'world.yaml')
    default_rviz = os.path.join(bringup_share, 'rviz', 'mission.rviz')

    world_file = LaunchConfiguration('world_file')

    return LaunchDescription([
        DeclareLaunchArgument('world_file', default_value=default_world,
                              description='World model YAML (waypoints, edges)'),
        DeclareLaunchArgument('goals', default_value='[all]',
                              description="Fixtures to clean, or [all]"),
        DeclareLaunchArgument('planner_backend', default_value='auto',
                              description='auto | internal-gbfs | internal-astar '
                                          '| popf | plansys2'),
        DeclareLaunchArgument('domain_mode', default_value='strips',
                              description='strips | temporal'),
        DeclareLaunchArgument('fail_edges', default_value="['']",
                              description="Blocked passages, e.g. [hall->commode]"),
        DeclareLaunchArgument('fail_locations', default_value="['']",
                              description='Unreachable waypoints, e.g. [commode]'),
        DeclareLaunchArgument('fail_mode', default_value='abort',
                              description='abort | reject'),
        DeclareLaunchArgument('fail_times', default_value='-1',
                              description='Failures before success; -1 = always'),
        DeclareLaunchArgument('use_rviz', default_value='false'),

        Node(
            package='washbot_control',
            executable='fake_nav2_server',
            name='fake_nav2_server',
            output='screen',
            parameters=[{
                'world_file': world_file,
                'fail_edges': LaunchConfiguration('fail_edges'),
                'fail_locations': LaunchConfiguration('fail_locations'),
                'fail_mode': LaunchConfiguration('fail_mode'),
                'fail_times': LaunchConfiguration('fail_times'),
            }],
        ),
        Node(
            package='washbot_control',
            executable='mission_controller',
            name='mission_controller',
            output='screen',
            parameters=[{
                'world_file': world_file,
                'goals': LaunchConfiguration('goals'),
                'planner_backend': LaunchConfiguration('planner_backend'),
                'domain_mode': LaunchConfiguration('domain_mode'),
            }],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', default_rviz],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
        ),
    ])
