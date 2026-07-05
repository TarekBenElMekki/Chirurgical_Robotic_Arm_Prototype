from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    moveit_demo = Path(
        get_package_share_directory("surgical_arm_moveit_config")
    ) / "launch" / "demo.launch.py"

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(moveit_demo))
        ),

        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="surgical_arm_mouse_control",
                    executable="workspace_2d_interactive_node",
                    name="workspace_2d_interactive_node",
                    output="screen",
                )
            ],
        ),
    ])
