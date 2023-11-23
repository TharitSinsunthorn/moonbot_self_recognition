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
from launch.substitutions import command, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ####### DATA INPUT ##########

    package_description = "moonbot_camera"

    rs_pkg = "realsense2_camera"
    rs_file = "rs_launch.py"
 
    launch_realsense_dir = os.path.join(
        get_package_share_directory(rs_pkg), "launch", rs_file
    )
    
    ####### DATA INPUT END ##########

    # Realsense camera launch file
    rs_LF = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_realsense_dir)
    )
    # rs_LF = Node(
    #     package=rs_pkg,
    #     executable=rs_file,
    #     namespace='cam_LF',
    #     parameters=[{'camera_name': 141122078145}]

    # )

    # YOLOv8 result launch file
    yolov8 = Node(
        package=package_description,
        executable='yolov8_ros2_pt.py',
        name='moonbot_detector_node'
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(
        get_package_share_directory(package_description), 
        'rviz', 
        'yolo_vis.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )

    return LaunchDescription(
        [
            rs_LF,
            yolov8,
            rviz_node,
        ]
    )
