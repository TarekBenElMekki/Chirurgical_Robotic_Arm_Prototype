from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import xacro
import yaml


def load_yaml(path):
    with open(path, "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    desc_pkg = get_package_share_directory("v2_description")
    moveit_pkg = get_package_share_directory("v2_moveit_config")
    viz_pkg = get_package_share_directory("v2_visualization")

    urdf_path = os.path.join(desc_pkg, "urdf", "surgical_arm.urdf.xacro")
    srdf_path = os.path.join(desc_pkg, "config", "surgical_arm.srdf")
    rviz_config = os.path.join(viz_pkg, "rviz", "dashboard.rviz")

    robot_description = {
        "robot_description": xacro.process_file(urdf_path).toxml()
    }

    robot_description_semantic = {
        "robot_description_semantic": open(srdf_path).read()
    }

    kinematics = load_yaml(os.path.join(desc_pkg, "config", "kinematics.yaml"))
    joint_limits = load_yaml(os.path.join(desc_pkg, "config", "joint_limits.yaml"))
    controllers = load_yaml(os.path.join(moveit_pkg, "config", "moveit_controllers.yaml"))
    planning = load_yaml(os.path.join(moveit_pkg, "config", "planning.yaml"))

    moveit_params = [
        robot_description,
        robot_description_semantic,
        kinematics,
        joint_limits,
        controllers,
        planning,
        {"joint_state_topic": "/joint_states_fresh"},
        {"moveit_manage_controllers": True},
        {"trajectory_execution.allowed_execution_duration_scaling": 100.0},
        {"trajectory_execution.allowed_goal_duration_margin": 10.0},
        {"trajectory_execution.allowed_start_tolerance": 0.5},
        {"trajectory_execution.execution_duration_monitoring": False},
    ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="pc_robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_pkg, "launch", "pc_moveit.launch.py")
        )
    )

    dashboard_markers = Node(
        package="v2_visualization",
        executable="dashboard_markers",
        name="dashboard_markers",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="v2_dashboard_rviz",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=moveit_params,
    )

    return LaunchDescription([
        robot_state_publisher,
        moveit,
        dashboard_markers,
        rviz,
    ])
