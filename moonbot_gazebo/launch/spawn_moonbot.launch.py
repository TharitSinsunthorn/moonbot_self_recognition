#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource

# this is the function launch  system will look for


def generate_launch_description():

    world_file_name = LaunchConfiguration('world_file_name')
    urdf_file = LaunchConfiguration('urdf_file')

    world_file_name_arg = DeclareLaunchArgument(
        'world_file_name',
        default_value='box_bot_empty.world'
    )

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='moonbotX.urdf'
    )


    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('moonbot_gazebo'), 'launch'),
            '/start_world.launch.py']),
    launch_arguments={'world_file_name': world_file_name}.items(),
    )

    launch_ros2_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('moonbot_control'), 'launch'),
            '/test_control.launch.py'])
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('moonbot_description'), 'launch'),
            '/urdf_visualize.launch.py']),
    launch_arguments={'use_joint_state_gui': 'False',
                      'urdf_file': urdf_file}.items(),
    )


    # create and return launch description object
    return LaunchDescription(
        [
            world_file_name_arg,
            urdf_file_arg,

            start_world,
            launch_ros2_control,
            spawn_robot
        ]
    )

