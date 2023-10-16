import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
import xacro

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'single.urdf'
    package_description = "moonbot_description"

    ####### DATA INPUT END ##########
    print("Fetching URDF ==>")
    robot_desc_path = os.path.join(
        get_package_share_directory(package_description), 
        "urdf", 
        urdf_file
    )

    # namespace = LaunchConfiguration('namespace')

    # declare_namespace_cmd = DeclareLaunchArgument(
    #     'namespace',
    #     default_value='',
    #     description='Top-level namespace')

    

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': True, 'robot_description': Command(['xacro ', robot_desc_path])}],
        # parameters=[robot_description],  ## for xacro
        # namespace = "namespace",
        output="screen"
    )

    # Joint State Publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
        # namespace = "namespace"
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
        # namespace = "namespace"
    )


    # create and return launch description object
    return LaunchDescription(
        [            
            # declare_namespace_cmd,
            robot_state_publisher_node,
            rviz_node,
            joint_state_publisher_node
        ]
    )
