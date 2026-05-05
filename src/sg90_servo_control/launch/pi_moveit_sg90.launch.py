from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_rviz = LaunchConfiguration("use_rviz")
    bringup_pkg = get_package_share_directory("surgical_arm_bringup")

    pi_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_pkg, "launch", "pi_robot.launch.py")
        ),
        launch_arguments={
            "use_rviz": use_rviz,
        }.items(),
    )

    sg90_servo_node = Node(
        package="sg90_servo_control",
        executable="sg90_servo_node",
        name="sg90_servo_node",
        output="screen",
        parameters=[
            {
                "gpio_pin": 18,
                "gpiochip": 0,

                "min_angle_deg": 0.0,
                "max_angle_deg": 180.0,
                "home_angle_deg": 90.0,

                "min_pulse_us": 1000.0,
                "max_pulse_us": 2000.0,
                "pwm_frequency_hz": 50.0,

                "control_mode": "move_then_detach",
                "min_move_time_sec": 0.35,
                "max_move_time_sec": 1.60,
                "seconds_per_degree": 0.008,
                "extra_settle_time_sec": 0.20,

                "command_topic": "/sg90/angle_deg",
                "state_topic": "/sg90/state_deg",
                "publish_rate_hz": 10.0,

                "detach_on_shutdown": True,
            }
        ],
    )

    ros2_control_servo_bridge = Node(
        package="sg90_servo_control",
        executable="ros2_control_servo_bridge",
        name="ros2_control_servo_bridge",
        output="screen",
        parameters=[
            {
                "controller_state_topic": "/hardware_arm_controller/controller_state",
                "servo_command_topic": "/sg90/angle_deg",
                "controlled_joint": "base_yaw_joint",

                "joint_min_rad": -1.5708,
                "joint_max_rad": 1.5708,
                "servo_min_deg": 0.0,
                "servo_max_deg": 180.0,
                "invert_direction": False,

                "min_publish_delta_deg": 0.5,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        pi_robot,
        sg90_servo_node,
        TimerAction(period=7.0, actions=[ros2_control_servo_bridge]),
    ])
