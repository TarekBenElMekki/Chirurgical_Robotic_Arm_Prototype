from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_servo = LaunchConfiguration("enable_servo")

    final_position_controller = Node(
        package="v2_control",
        executable="final_position_controller",
        name="final_position_controller",
        output="screen",
    )

    servo_driver = Node(
        package="v2_control",
        executable="servo_driver",
        name="servo_driver",
        output="screen",
        condition=IfCondition(enable_servo),
        parameters=[
            {
                "gpiochip": 0,
                "gpio_pin": 18,
                "min_angle_deg": 0.0,
                "max_angle_deg": 180.0,
                "min_pulse_us": 1000.0,
                "max_pulse_us": 2000.0,
                "pwm_frequency_hz": 50.0,
                "command_topic": "/motor/command_angle_deg",
                "real_angle_topic": "/motor/real_angle_deg",
                "error_topic": "/motor/error_deg",
                "min_command_delta_deg": 0.5,
                "hold_pwm": True,
                "detach_on_shutdown": False,
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_servo", default_value="false"),
        final_position_controller,
        servo_driver,
    ])
