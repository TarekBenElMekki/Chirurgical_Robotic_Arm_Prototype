from launch.actions import ExecuteProcess
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


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

    moveit_config = (
        MoveItConfigsBuilder(
            "surgical_arm_primitive",
            package_name="surgical_arm_moveit_config"
        )
        .to_moveit_configs()
    )

    rviz_config = os.path.join(
        get_package_share_directory("surgical_arm_moveit_config"),
        "config",
        "real_feedback_live_robot.rviz"
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    matrix_node = Node(
        package="surgical_arm_mouse_control",
        executable="workspace_2d_interactive_node",
        output="screen",
    )

    return LaunchDescription([
        rviz_camera_force_bridge,
        robot_state_publisher,
        move_group,
        rviz,
        matrix_node,
    ])
