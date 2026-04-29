from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('surgical_arm_perception')
    fsr_config = os.path.join(pkg, 'config', 'fsr_config.yaml')

    camera = Node(
        package='camera_ros',
        executable='camera_node',
        name='camera',
        output='screen',
        parameters=[{
            'width': 640,
            'height': 480,
            'format': 'RGB888',
            'frame_id': 'camera_link',
        }]
    )

    fsr = Node(
        package='surgical_arm_perception',
        executable='fsr_publisher.py',
        name='fsr_publisher',
        output='screen',
        parameters=[fsr_config]
    )

    return LaunchDescription([
        camera,
        fsr,
    ])
