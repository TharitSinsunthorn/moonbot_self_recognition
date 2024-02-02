import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition
import xacro

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    # urdf_file = 'moonbotX.urdf'
    package_description = "moonbot_description"

    use_joint_state_gui = LaunchConfiguration('use_joint_state_gui')
    urdf_file = LaunchConfiguration('urdf_file')
    robot_desc_path = LaunchConfiguration('robot_desc_path')

    gui_arg = DeclareLaunchArgument(
        'use_joint_state_gui',
        default_value='True'
    )

    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='moonbotX.urdf'
    )

    robot_desc_path_arg = DeclareLaunchArgument(
        'robot_desc_path',
        default_value=[os.path.join(get_package_share_directory(package_description), 
                                    'urdf'),
                                    '/', 
                                    urdf_file],
        description='URDF file'
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        # parameters=[robot_description],  ## for xacro
        output="screen"
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(use_joint_state_gui)
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(
        get_package_share_directory(package_description), 
        'rviz', 
        'urdf_vis.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        name='rviz_node',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', rviz_config_dir]
    )


    # create and return launch description object
    return LaunchDescription(
        [           
            gui_arg,
            urdf_file_arg,
            robot_desc_path_arg,

            robot_state_publisher_node,
            rviz_node,
            joint_state_publisher_node
        ]
    )
