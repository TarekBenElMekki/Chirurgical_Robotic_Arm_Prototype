from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sg90_node = Node(
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

                "command_topic": "/sg90/angle_deg",
                "state_topic": "/sg90/state_deg",
                "publish_rate_hz": 10.0,

                "detach_on_shutdown": True,
            }
        ],
    )

    web_interface = Node(
        package="sg90_servo_control",
        executable="sg90_web_interface",
        name="sg90_web_interface",
        output="screen",
        parameters=[
            {
                "host": "0.0.0.0",
                "port": 8080,

                "min_angle_deg": 0.0,
                "max_angle_deg": 180.0,
                "default_angle_deg": 90.0,

                "command_topic": "/sg90/angle_deg",
                "state_topic": "/sg90/state_deg",
            }
        ],
    )

    return LaunchDescription([
        sg90_node,
        web_interface,
    ])
