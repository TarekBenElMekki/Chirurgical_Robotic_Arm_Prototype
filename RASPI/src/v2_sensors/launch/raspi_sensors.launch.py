from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_camera = LaunchConfiguration("enable_camera")
    enable_fsr = LaunchConfiguration("enable_fsr")

    camera = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        output="screen",
        condition=IfCondition(enable_camera),
        parameters=[
            {
                "width": 640,
                "height": 480,
                "format": "RGB888",
                "frame_id": "camera_link",
            }
        ],
    )

    fsr = Node(
        package="v2_sensors",
        executable="fsr_publisher",
        name="fsr_publisher",
        output="screen",
        condition=IfCondition(enable_fsr),
        parameters=[
            {
                "spi_bus": 0,
                "spi_device": 0,
                "spi_speed_hz": 1000000,
                "publish_rate_hz": 30.0,
                "calibration_max_raw": 450.0,
                "force_max_n": 10.0,
                "raw_topic": "/fsr/raw",
                "normalized_topic": "/fsr/normalized",
                "force_topic": "/fsr/force_n",
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("enable_camera", default_value="true"),
        DeclareLaunchArgument("enable_fsr", default_value="true"),
        camera,
        fsr,
    ])
