from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    dynamixel_node = Node(
        package='moonbot',
        executable='dynamixel_control',
        name='dynamixel_control'
        )
    body_controller_node = Node(
        package="moonbot",
        executable="body_controller",
        name = "body_controller"
        )
    
    joint_interface_nodes = [Node(
        package="moonbot",
        executable="joint_interface",
        name = f"joint_interface_l{i + 1}",
        parameters = [{'limb_num': i + 1}]
        ) for i in range(2)]
    
    joint_controller_nodes = [Node(
        package="moonbot",
        executable="joint_controller",
        name = f"joint_controller_l{i + 1}",
        parameters= [{'limb_num': i + 1}]
        ) for i in range(2)]
    
    return LaunchDescription([dynamixel_node, body_controller_node] + joint_controller_nodes + joint_interface_nodes)