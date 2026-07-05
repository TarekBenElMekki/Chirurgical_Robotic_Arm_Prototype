from glob import glob
from setuptools import setup

package_name = "surgical_arm_manual_mode"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="master_thesis",
    maintainer_email="master_thesis@example.com",
    description="Standalone manual mode layer for surgical arm RViz reactivity.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mode_selector_dock_node = surgical_arm_manual_mode.mode_selector_dock_node:main",
            "manual_udp_joystick_receiver_node = surgical_arm_manual_mode.manual_udp_joystick_receiver_node:main",
            "manual_command_mixer_node = surgical_arm_manual_mode.manual_command_mixer_node:main",
            "manual_joint_state_publisher_node = surgical_arm_manual_mode.manual_joint_state_publisher_node:main",
            "rviz_clicked_point_mode_gate_node = surgical_arm_manual_mode.rviz_clicked_point_mode_gate_node:main",
        ],
    },
)