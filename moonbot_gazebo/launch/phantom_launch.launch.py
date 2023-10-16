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
        # Node(
        #     package='turtlesim',
        #     namespace='turtlesim2',
        #     executable='turtlesim_node',
        #     name='sim'
        # ),
        # Node(
        #     package='turtlesim',
        #     executable='mimic',
        #     name='mimic',
        #     remappings=[
        #         ('/input/pose', '/turtlesim1/turtle1/pose'),
        #         ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        #     ]
        # )
    ])
