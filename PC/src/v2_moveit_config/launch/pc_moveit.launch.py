from launch import LaunchDescription
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

    urdf_path = os.path.join(desc_pkg, "urdf", "surgical_arm.urdf.xacro")
    srdf_path = os.path.join(desc_pkg, "config", "surgical_arm.srdf")

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

    common_params = [
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

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=common_params,
    )

    return LaunchDescription([
        move_group,
    ])
