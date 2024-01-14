import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    robot_name = "limbbot"
    urdf_file = "singleHWRR.urdf"
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
        "RR_config.yaml"
    )

    RR_ns = LaunchConfiguration('RR_ns')

    RR_ns_arg = DeclareLaunchArgument(
        'RR_ns',
        default_value='RR'
    )


    return LaunchDescription([
        RR_ns_arg,
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            namespace = RR_ns,
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
                       "--controller-manager", "/RR/controller_manager"],
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
                       "-c", "/RR/controller_manager"],
        ),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace = RR_ns,
            parameters=[
                {"robot_description": robot_description_config.toxml()}],
            output="screen"),

        # Node(
        #     package="rviz2",
        #     executable="rviz2",
        #     name="rviz2",
        #     # namespace = RR_ns,
        #     arguments=["-d", rviz_config],
        #     output={
        #         "stdout": "screen",
        #         "stderr": "log",
        #     },
        # ),

        # Node(
        #     package="moonbot_gazebo",
        #     executable="RR.py",
        #     output="screen"
        # )

    ])