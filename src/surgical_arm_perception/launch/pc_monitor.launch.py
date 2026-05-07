from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro


def generate_launch_description():
    bringup_pkg = get_package_share_directory('surgical_arm_bringup')
    perception_pkg = get_package_share_directory('surgical_arm_perception')

    urdf = os.path.join(bringup_pkg, 'urdf', 'surgical_arm.urdf.xacro')
    rviz_config = os.path.join(perception_pkg, 'rviz', 'pc_robot_monitor.rviz')

    robot_description = {
        'robot_description': xacro.process_file(urdf).toxml()
    }

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='perception_robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='pc_robot_monitor_rviz',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[robot_description]
    )

    return LaunchDescription([
        robot_state_publisher,
        rviz,
    ])
