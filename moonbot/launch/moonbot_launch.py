from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
        package='moonbot',
        executable='dynamixel_control',
        name='dynamixel_control'
        ),
        Node(
        package="moonbot",
        executable="joint_interface",
        name = "joint_interface_l1",
        parameters = [{'limb_num': 1}]
        ),
        Node(
        package="moonbot",
        executable="joint_controller",
        name = "joint_controller_l1",
        parameters= [{'limb_num': 1}]
        ),
        Node(
        package="moonbot",
        executable="body_controller",
        name = "body_controller"
        )
    ])