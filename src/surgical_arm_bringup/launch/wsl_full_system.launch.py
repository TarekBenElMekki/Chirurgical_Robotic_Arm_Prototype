from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('surgical_arm_bringup')

    wsl_sim_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg, 'launch', 'wsl_sim_control.launch.py')
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
    return LaunchDescription([
        wsl_sim_control_launch,
        moveit_launch,
        pwm_mapper,
    ])
