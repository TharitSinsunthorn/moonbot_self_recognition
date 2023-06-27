from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    moonbot_control_params = DeclareLaunchArgument(
        'moonbot_control_params',
        default_value='$(find moonbot_control)/config/moonbot_control.yaml',
        description='Path to the Monbot control parameters file'
    )

    controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        name='controller_spawner',
        output='screen',
        namespace='/phantomx',
        arguments=[
            'j_c1_lf_position_controller', 'j_c1_rf_position_controller', 'j_c1_lm_position_controller',
            'j_c1_rm_position_controller', 'j_c1_lr_position_controller', 'j_c1_rr_position_controller',
            'j_thigh_lf_position_controller', 'j_thigh_rf_position_controller', 'j_thigh_lm_position_controller',
            'j_thigh_rm_position_controller', 'j_thigh_lr_position_controller', 'j_thigh_rr_position_controller',
            'j_tibia_lf_position_controller', 'j_tibia_rf_position_controller', 'j_tibia_lm_position_controller',
            'j_tibia_rm_position_controller', 'j_tibia_lr_position_controller', 'j_tibia_rr_position_controller',
            'joint_state_controller'
        ]
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        remappings=[('/joint_states', '/phantomx/joint_states')]
    )

    return LaunchDescription([
        moonbot_control_params,
        controller_spawner,
        robot_state_publisher
    ])

