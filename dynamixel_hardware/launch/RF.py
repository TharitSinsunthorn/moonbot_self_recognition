import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    robot_name = "limbbot"
    urdf_file = "singleHWRF.urdf"
    package_description = "moonbot_description"

    rviz_config = os.path.join(
        get_package_share_directory(package_description), 
        'rviz', 
        'urdf_vis.rviz'
    )

    robot_desc_path = os.path.join(
        get_package_share_directory(package_description), 
        "urdf", 
        urdf_file
    )

    robot_description_config = xacro.process_file(robot_desc_path)

    controller_config = os.path.join(
        get_package_share_directory("dynamixel_hardware"),
        "config",
        "RF_config.yaml"
    )

    return LaunchDescription([
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace = '/RF',
            parameters=[
                {"robot_description": robot_description_config.toxml()}, controller_config],
            output={
                "stdout": "screen",
                "stderr": "screen",
            },
        ),

        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/RF/controller_manager"],
        ),

        # Node(
        #     package="controller_manager",
        #     executable="spawner.py",
        #     arguments=["forward_position_controller",
        #                "-c", "/controller_manager"],
        # ),

        Node(
            package="controller_manager",
            executable="spawner.py",
            arguments=["position_trajectory_controller",
                       "-c", "/RF/controller_manager"],
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace = '/RF',
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen"),

        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz2",
        #     # namespace = '/RF',
        #     arguments=["-d", rviz_config],
        #     output={
        #         "stdout": "screen",
        #         "stderr": "log",
        #     },
        # ),

        # Node(
        #     package="moonbot_gazebo",
        #     executable="RF.py",
        #     output="screen"
        # )

    ])