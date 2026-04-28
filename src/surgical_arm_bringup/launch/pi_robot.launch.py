from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro

def generate_launch_description():
    pkg = get_package_share_directory('surgical_arm_bringup')
    urdf = os.path.join(pkg, 'urdf', 'surgical_arm.urdf.xacro')
    controllers = os.path.join(pkg, 'config', 'ros2_controllers.yaml')
    robot_description = {'robot_description': xacro.process_file(urdf).toxml()}

    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[robot_description], output='screen'),

        Node(package='controller_manager', executable='ros2_control_node',
             parameters=[robot_description, controllers], output='screen'),

        TimerAction(period=2.0, actions=[
            ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                                'joint_state_broadcaster'], output='screen')
        ]),

        TimerAction(period=3.0, actions=[
            ExecuteProcess(cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
                                'hardware_arm_controller'], output='screen')
        ]),

        Node(package='v4l2_camera', executable='v4l2_camera_node',
             name='camera', output='screen',
             parameters=[{'video_device': '/dev/video0', 'image_size': [640, 480]}]),

        Node(package='surgical_arm_bringup', executable='status_monitor.py',
             output='screen'),

        Node(package='surgical_arm_bringup', executable='trajectory_gateway.py',
             output='screen'),

        Node(package='surgical_arm_bringup', executable='joint_state_freshener.py',
             output='screen'),
    ])
