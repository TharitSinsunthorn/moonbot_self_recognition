from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moonbot_gazebo',
            # namespace='turtlesim1',
            executable='RF.py',
            name='RF'
        ),
        Node(
            package='moonbot_gazebo',
            # namespace='turtlesim1',
            executable='LF.py',
            name='LF'
        ),
        Node(
            package='moonbot_gazebo',
            # namespace='turtlesim1',
            executable='LR.py',
            name='LF'
        ),
        Node(
            package='moonbot_gazebo',
            # namespace='turtlesim1',
            executable='RR.py',
            name='LF'
        ),
    ])
