from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro, yaml

def load_yaml(path):
    with open(path, 'r') as f:
        return yaml.safe_load(f)

def generate_launch_description():
    pkg = get_package_share_directory('surgical_arm_bringup')
    urdf = os.path.join(pkg, 'urdf', 'surgical_arm.urdf.xacro')
    srdf = os.path.join(pkg, 'config', 'surgical_arm.srdf')

    robot_description = {'robot_description': xacro.process_file(urdf).toxml()}
    robot_description_semantic = {'robot_description_semantic': open(srdf).read()}

    kinematics = load_yaml(os.path.join(pkg, 'config', 'kinematics.yaml'))
    joint_limits = load_yaml(os.path.join(pkg, 'config', 'joint_limits.yaml'))
    controllers = load_yaml(os.path.join(pkg, 'config', 'moveit_controllers.yaml'))

    move_group = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            {'joint_state_topic': '/joint_states_fresh'},
            {'trajectory_execution.allowed_execution_duration_scaling': 100.0},
            {'trajectory_execution.allowed_goal_duration_margin': 10.0},
            {'trajectory_execution.allowed_start_tolerance': 0.5},
            {'trajectory_execution.execution_duration_monitoring': False},

            {'joint_state_topic': '/joint_states_fresh'},
            {'trajectory_execution.allowed_execution_duration_scaling': 1.2,
             'trajectory_execution.allowed_goal_duration_margin': 0.5,
             'trajectory_execution.allowed_start_tolerance': 0.01},
            {'moveit_manage_controllers': True},
            {'trajectory_execution.execution_duration_monitoring': True},
            {'#planning_pipelines_disabled': ['ompl'], 'default_planning_pipeline': 'ompl', 'planning_plugin': 'ompl_interface/OMPLPlanner'},

            robot_description,
            robot_description_semantic,
            kinematics,
        load_yaml(os.path.join(pkg, 'config', 'ompl_planning.yaml')),

            joint_limits,
        load_yaml(os.path.join(pkg, 'config', 'planning.yaml')),

            controllers,
            {'#planning_pipelines_disabled': ['ompl'], 'default_planning_pipeline': 'ompl', 'planning_plugin': 'ompl_interface/OMPLPlanner'}
        ])

    rviz_config = os.path.join(pkg, 'rviz', 'moveit.rviz')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[
            {'joint_state_topic': '/joint_states_fresh'},
            {'trajectory_execution.allowed_execution_duration_scaling': 1.2,
             'trajectory_execution.allowed_goal_duration_margin': 0.5,
             'trajectory_execution.allowed_start_tolerance': 0.01},
            {'moveit_manage_controllers': True},
            {'trajectory_execution.execution_duration_monitoring': True},
            {'#planning_pipelines_disabled': ['ompl'], 'default_planning_pipeline': 'ompl', 'planning_plugin': 'ompl_interface/OMPLPlanner'},
robot_description, robot_description_semantic, kinematics,
        load_yaml(os.path.join(pkg, 'config', 'ompl_planning.yaml')),
 joint_limits]
    )

    return LaunchDescription([move_group, rviz])
