from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        Node(
            package="surgical_arm_manual_mode",
            executable="rviz_clicked_point_mode_gate_node",
            name="rviz_clicked_point_mode_gate_node",
            output="screen",
        ),
        Node(
            package="surgical_arm_manual_mode",
            executable="manual_udp_joystick_receiver_node",
            name="manual_udp_joystick_receiver_node",
            output="screen",
            parameters=[
                {"listen_ip": "0.0.0.0"},
                {"listen_port": 5005},
                {"x_center": 513.0},
                {"y_center": 520.0},
                {"deadzone": 0.08},
            ],
        ),
        Node(
            package="surgical_arm_manual_mode",
            executable="manual_command_mixer_node",
            name="manual_command_mixer_node",
            output="screen",
        ),
        Node(
            package="surgical_arm_manual_mode",
            executable="manual_joint_state_publisher_node",
            name="manual_joint_state_publisher_node",
            output="screen",
            parameters=[
                {"joint_1_name": "joint_1_y"},
                {"joint_2_name": "joint_2_z"},
                {"jaw_joint_name": "mobile_jaw_joint_y"},
                {"joint_1_min": -0.314},
                {"joint_1_max": 0.314},
                {"joint_2_min": -0.314},
                {"joint_2_max": 0.314},
                {"jaw_min": 0.0},
                {"jaw_max": 0.314},
                {"joint_velocity_gain": 0.55},
                {"jaw_velocity_gain": 0.45},
            ],
        ),
        Node(
            package="surgical_arm_manual_mode",
            executable="mode_selector_dock_node",
            name="mode_selector_dock_node",
            output="screen",
        ),
    ])