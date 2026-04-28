from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='pi_monitor_rviz',
            output='screen',
            arguments=['--ros-args', '--log-level', 'info'],
        )
    ])
