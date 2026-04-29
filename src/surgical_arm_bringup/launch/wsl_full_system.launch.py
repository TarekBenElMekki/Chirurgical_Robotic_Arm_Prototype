from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('surgical_arm_bringup')

    pi_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'pi_robot.launch.py')
        ),
        launch_arguments={
            'use_rviz': 'false',
        }.items()
    )

    moveit_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg, 'launch', 'pc_moveit.launch.py')
                )
            )
        ]
    )

    pwm_mapper = Node(
        package='surgical_arm_bringup',
        executable='pwm_mapper.py',
        name='pwm_mapper',
        output='screen'
    )

    pwm_echo_hint = Node(
        package='surgical_arm_bringup',
        executable='status_monitor.py',
        name='status_monitor',
        output='screen'
    )

    return LaunchDescription([
        pi_robot_launch,
        moveit_launch,
        pwm_mapper,
        pwm_echo_hint,
    ])
