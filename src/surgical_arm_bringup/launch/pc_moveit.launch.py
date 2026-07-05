from launch.actions import ExecuteProcess
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro, yaml

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

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
    srdf = os.path.join(pkg, 'config', 'surgical_arm.srdf')

    robot_description = {'robot_description': xacro.process_file(urdf).toxml()}
    robot_description_semantic = {'robot_description_semantic': open(srdf).read()}

    kinematics = load_yaml(os.path.join(pkg, 'config', 'kinematics.yaml'))
    joint_limits = load_yaml(os.path.join(pkg, 'config', 'joint_limits.yaml'))
    controllers = load_yaml(os.path.join(pkg, 'config', 'moveit_controllers.yaml'))
    planning = load_yaml(os.path.join(pkg, 'config', 'planning.yaml'))

    common_params = [
        
        robot_description,
        robot_description_semantic,
        kinematics,
        joint_limits,
        controllers,
        planning,

        # Correct parameters (ONLY ONCE)
        {'joint_state_topic': '/joint_states_fresh'},

        {'moveit_manage_controllers': True},

        {'trajectory_execution.allowed_execution_duration_scaling': 100.0},
        {'trajectory_execution.allowed_goal_duration_margin': 10.0},
        {'trajectory_execution.allowed_start_tolerance': 0.5},
        {'trajectory_execution.execution_duration_monitoring': False},
    ]

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=common_params
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg, 'rviz', 'moveit.rviz')],
        parameters=common_params
    )

    return LaunchDescription([
        rviz_camera_force_bridge,move_group, rviz])
