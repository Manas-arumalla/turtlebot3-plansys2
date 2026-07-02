"""Localization + navigation for a TurtleBot3 on the washroom map.

Starts a deliberately minimal, fully self-contained Nav2 stack — map server,
AMCL, planner, controller, behaviors, BT navigator — configured by
``config/nav2_params.yaml``. Start the robot (or Gazebo) first, e.g.:

    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

then:

    ros2 launch washbot_bringup nav2_tb3.launch.py use_sim_time:=true
    ros2 run washbot_control initial_pose_publisher --ros-args \\
        -p x:=-2.05 -p y:=-0.55 -p yaw:=0.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('washbot_bringup')
    default_map = os.path.join(bringup_share, 'maps', 'washroom_world.yaml')
    default_params = os.path.join(bringup_share, 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    common = {'use_sim_time': use_sim_time}

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),

        Node(package='nav2_map_server', executable='map_server',
             name='map_server', output='screen',
             parameters=[params_file, common, {'yaml_filename': map_yaml}]),
        Node(package='nav2_amcl', executable='amcl',
             name='amcl', output='screen',
             parameters=[params_file, common]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_localization', output='screen',
             parameters=[{'autostart': True,
                          'node_names': ['map_server', 'amcl'],
                          **{'use_sim_time': use_sim_time}}]),

        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[params_file, common]),
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[params_file, common]),
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[params_file, common]),
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[params_file, common]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_navigation', output='screen',
             parameters=[{'autostart': True,
                          'node_names': ['controller_server', 'planner_server',
                                         'behavior_server', 'bt_navigator'],
                          **{'use_sim_time': use_sim_time}}]),
    ])
