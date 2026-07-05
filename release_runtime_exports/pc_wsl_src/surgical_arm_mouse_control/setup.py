import os
from glob import glob
from setuptools import find_packages, setup

package_name = "surgical_arm_mouse_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="master_thesis",
    maintainer_email="master_thesis@example.com",
    description="2D RViz mouse target IK control for the primitive surgical arm.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mouse_2d_goal_node = surgical_arm_mouse_control.mouse_2d_goal_node:main",
            "workspace_2d_interactive_node = surgical_arm_mouse_control.workspace_2d_interactive_node:main",
        ],
    },
)
