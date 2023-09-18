from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    dynamixel_node = [Node(
        package='move_moonbot',
        executable='dynamixel_control',
        name=f'dynamixel_control_d{i+1}',
        parameters = [{'dynamixel_num': i + 1}]
        ) for i in range(2)]

    move_robot_node = [Node(
        package="move_moonbot",
        executable="move_robot",
        name = "move_robot"
        )]
    
    joint_interface_nodes = [Node(
        package="move_moonbot",
        executable="joint_interface",
        name = f"joint_interface_l{i +1 }",
        parameters = [{'limb_num': i +1 }]
        ) for i in range(4)]
    
    # return LaunchDescription(dynamixel_nodes)
    return LaunchDescription(dynamixel_node + joint_interface_nodes + move_robot_node)