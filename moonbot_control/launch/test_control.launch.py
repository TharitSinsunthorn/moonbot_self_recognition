#!/usr/bin/python3
# -*- coding: utf-8 -*-
import random

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource

# this is the function launch  system will look for


def generate_launch_description():


    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.5]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]
    # Base Name or robot
    robot_base_name = "Moonbot"


    entity_name = robot_base_name+"-"+str(int(random.random()*100000))

    # Spawn ROBOT Set Gazebo
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        output='screen',
        arguments=['-entity',
                   entity_name,
                   '-x', str(position[0]), '-y', str(position[1]
                                                     ), '-z', str(position[2]),
                   '-R', str(orientation[0]), '-P', str(orientation[1]
                                                        ), '-Y', str(orientation[2]),
                   '-topic', '/robot_description'
                   ]
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster",
                   "--controller-manager", "/controller_manager"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["position_trajectory_controller", "-c", "/controller_manager"],
    )

    # robot_controller = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[/robot_description, <your_controllers_yaml_file_path>]
    # )

    RF_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["RFposition_trajectory_controller", "-c", "/controller_manager"],
    )

    LF_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["LFposition_trajectory_controller", "-c", "/controller_manager"],
    )

    LR_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["LRposition_trajectory_controller", "-c", "/controller_manager"],
    )

    RR_robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["RRposition_trajectory_controller", "-c", "/controller_manager"],
    )

    # create and return launch description object
    return LaunchDescription(
        [
            RegisterEventHandler(
                event_handler=OnProcessExit(
                  target_action=spawn_robot,
                  on_exit=[joint_state_broadcaster_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessExit(
                  target_action=joint_state_broadcaster_spawner,
                  on_exit=[RF_robot_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                  target_action=RF_robot_controller_spawner,
                  on_start=[LF_robot_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                  target_action=LF_robot_controller_spawner,
                  on_start=[LR_robot_controller_spawner],
                )
            ),
            RegisterEventHandler(
                event_handler=OnProcessStart(
                  target_action=LR_robot_controller_spawner,
                  on_start=[RR_robot_controller_spawner],
                )
            ),
            spawn_robot,
        ]
    )