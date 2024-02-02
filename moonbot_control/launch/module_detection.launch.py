import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.event_handlers import OnProcessExit
from launch.actions import  RegisterEventHandler, DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit, OnProcessStart
import xacro


def generate_launch_description():

    detection_config = os.path.join(
        get_package_share_directory("moonbot_control"),
        "config",
        "detection_param.yaml"
    )

    rviz_config = os.path.join(
        get_package_share_directory("moonbot_description"), 
        'rviz', 
        'urdf_vis.rviz'
    )

    RF_connection = Node(
        package="moonbot_control",
        executable="RF_con.py",
        output="screen",
        parameters=[detection_config]
    )

    LF_connection = Node(
        package="moonbot_control",
        executable="LF_con.py",
        output="screen",
        parameters=[detection_config]
    )

    LR_connection = Node(
        package="moonbot_control",
        executable="LR_con.py",
        output="screen",
        parameters=[detection_config]
    )

    RR_connection = Node(
        package="moonbot_control",
        executable="RR_con.py",
        output="screen",
        parameters=[detection_config]
    )

    pinging_node = Node(
        package="moonbot_control",
        executable="launch_detection.py",
        output="screen"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output={
            "stdout": "screen",
            "stderr": "log",
        },
    )

    return LaunchDescription([

        RF_connection,

        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=RF_connection,
                on_start=[LF_connection],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=LF_connection,
                on_start=[LR_connection],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=LR_connection,
                on_start=[RR_connection],
            )
        ),

        pinging_node, 
        rviz_node
        
    ])