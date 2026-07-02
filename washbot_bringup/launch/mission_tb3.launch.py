"""Mission on a TurtleBot3 driven by the real Nav2 stack.

Expects the robot (or its Gazebo simulation) to be up already:

    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py    # terminal 1
    ros2 launch washbot_bringup mission_tb3.launch.py           # terminal 2

This launch starts Nav2 on the washroom map, seeds AMCL with the dock pose,
and starts the mission controller (delayed so localization settles first).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('washbot_bringup')
    control_share = get_package_share_directory('washbot_control')
    default_world = os.path.join(control_share, 'config', 'world.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('world_file', default_value=default_world),
        DeclareLaunchArgument('goals', default_value='[all]'),
        DeclareLaunchArgument('planner_backend', default_value='auto'),
        DeclareLaunchArgument('domain_mode', default_value='strips'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bringup_share, 'launch', 'nav2_tb3.launch.py')),
            launch_arguments={
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),

        # Seed AMCL with the dock pose once the stack has had time to start
        # (the publisher repeats for ~8 s, covering slow activations).
        TimerAction(period=8.0, actions=[
            Node(
                package='washbot_control',
                executable='initial_pose_publisher',
                name='initial_pose_publisher',
                output='screen',
                parameters=[{'x': -2.05, 'y': -0.55, 'yaw': 0.0,
                             'use_sim_time': LaunchConfiguration('use_sim_time')}],
            ),
        ]),

        # Start the mission only after localization has settled; a navigation
        # goal sent into a half-initialized stack reads as a blocked passage.
        TimerAction(period=20.0, actions=[
            Node(
                package='washbot_control',
                executable='mission_controller',
                name='mission_controller',
                output='screen',
                parameters=[{
                    'world_file': LaunchConfiguration('world_file'),
                    'goals': LaunchConfiguration('goals'),
                    'planner_backend': LaunchConfiguration('planner_backend'),
                    'domain_mode': LaunchConfiguration('domain_mode'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    # Jazzy TurtleBot3 bridges cmd_vel as TwistStamped.
                    'stamped_cmd_vel': True,
                }],
            ),
        ]),
    ])
