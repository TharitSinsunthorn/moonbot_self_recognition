from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trial',
            executable='dynamixel_control.py',
            output='screen'),

    ])