from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():

    # RVIZ_CAMERA_FORCE_BRIDGE_P6
    rviz_camera_force_bridge = ExecuteProcess(
        cmd=[
            "python3",
            "/home/master_thesis/PFE_Surgical_Arm_Portable_V0_4_2_MANUAL_RVIZ_SYNC_DEV/Runtime/WSL/rviz_camera_force_bridge.py",
            "--ros-args",
            "-p", "stream_url:=http://100.112.77.4:8081/stream.mjpg",
            "-p", "camera_topic:=/camera/image_raw",
            "-p", "force_topic:=/force/newton",
            "-p", "marker_topic:=/force/rviz_marker",
            "-p", "frame_id:=base_link",
            "-p", "publish_fps:=15.0",
            "-p", "force_limit_n:=15.0",
        ],
        output="screen",
    )

    pkg = get_package_share_directory('surgical_arm_bringup')
    urdf = os.path.join(pkg, 'urdf', 'surgical_arm.urdf.xacro')
    controllers = os.path.join(pkg, 'config', 'ros2_controllers.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'pi_monitor.rviz')

    robot_description = {'robot_description': xacro.process_file(urdf).toxml()}
    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        rviz_camera_force_bridge,
        DeclareLaunchArgument('use_rviz', default_value='false'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            remappings=[('/joint_states', '/raw_joint_states')],
            parameters=[robot_description, controllers],
            output='screen'
        ),

        TimerAction(period=4.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]),

        TimerAction(period=5.0, actions=[
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['hardware_arm_controller', '--controller-manager', '/controller_manager'],
                output='screen'
            )
        ]),

        Node(
            package='surgical_arm_bringup',
            executable='joint_state_freshener.py',
            output='screen'
        ),

        Node(
            package='surgical_arm_bringup',
            executable='trajectory_gateway.py',
            output='screen'
        ),

        Node(
            package='surgical_arm_bringup',
            executable='status_monitor.py',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='pi_monitor_rviz',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(use_rviz)
        ),
    ])
