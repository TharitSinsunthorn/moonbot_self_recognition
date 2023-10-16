import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    launch_file = "limb_visualize.launch.py"
    launch_file2 = "urdf_visualize.launch.py"
    package_description = "moonbot_description"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    launch_dir = os.path.join(
        get_package_share_directory(package_description), "launch", launch_file
    )

    launch_dir2 = os.path.join(
        get_package_share_directory(package_description), "launch", launch_file2
    )


    limb1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_dir)
        # launch_arguments={
        #     "namespace": TextSubstitution(text="limb1"),
        #     },
        
        )

    limb2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_dir2)
        # launch_arguments={
        #     "namespace": TextSubstitution(text="limb1"),
        #     },
        
        )

    # limb1_ns = GroupAction(
    #     actions=[
    #         PushRosNamespace('limb1'),
    #         limb1,
    #     ])

    limb2_ns = GroupAction(
        actions=[
            PushRosNamespace('limb2'),
            limb2,
        ])

    limb1_ns = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moonbot_description'),
                'launch',
                'urdf_visualize.launch.py'
                ])
            ]),
            launch_arguments={
                'phantom_ns': 'phantom1'
            }.items()
        )

    limb2_ns = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('moonbot_description'),
                'launch',
                'urdf_visualize.launch.py'
                ])
            ]),
            launch_arguments={
                'phantom_ns': 'phantom2'
            }.items()
        )



    # ld.add_action(limb1)
    # ld.add_action(limb2)
    # ld.add_action(limb3)
    # ld.add_action(limb4)

    return LaunchDescription([
        limb1_ns,
        limb2_ns
    ])
