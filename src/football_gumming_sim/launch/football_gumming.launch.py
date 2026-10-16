import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('football_gumming_sim')
    gazebo_share = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(
        package_share, 'worlds', 'football_gumming_robot.world'
    )
    xacro_file = os.path.join(
        package_share, 'urdf', 'football_gumming.urdf.xacro'
    )

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r ', world_file]}.items()
    )

    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'football_gumming_machine',
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )

    joint_state_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager',
                   '/controller_manager'],
        output='screen'
    )

    axis_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['axis_position_controller', '--controller-manager',
                   '/controller_manager'],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        state_publisher,
        TimerAction(period=4.0, actions=[spawn_robot]),
        TimerAction(period=10.0, actions=[
            joint_state_spawner,
            axis_controller_spawner
        ])
    ])
