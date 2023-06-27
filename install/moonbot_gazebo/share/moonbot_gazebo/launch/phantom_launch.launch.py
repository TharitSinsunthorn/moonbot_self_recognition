from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    empty_world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('gazebo_ros'), '/launch/empty_world.launch.py']),
        launch_arguments={
            'world_name': 'worlds/empty.world',
            'paused': 'true'
        }.items()
    )

    robot_description = DeclareLaunchArgument(
        'robot_description',
        default_value='$(find phantomx_description)/urdf/phantomx.urdf',
        description='Path to the robot description file (URDF)'
    )

    spawn_urdf = Node(
        package='gazebo_ros',
        executable='spawn_model',
        arguments=[
            '-file', '$(find phantomx_description)/urdf/phantomx.urdf',
            '-urdf',
            '-model', 'phantomx',
            '-z', '0.2'
        ]
    )

    phantomx_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('phantomx_control'), '/launch/phantomx_control.launch.py'])
    )

    phantomx_walker = Node(
        package='phantomx_gazebo',
        executable='walker.py'
    )

    return LaunchDescription([
        empty_world_launch,
        robot_description,
        spawn_urdf,
        phantomx_control_launch,
        phantomx_walker
    ])


if __name__ == '__main__':
    generate_launch_description()
