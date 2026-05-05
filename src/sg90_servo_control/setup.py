from setuptools import setup
import os
from glob import glob

package_name = "sg90_servo_control"

setup(
    name=package_name,
    version="0.5.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Tarek Ben El Mekki",
    maintainer_email="benmekkitarek@gmail.com",
    description="Independent SG90 servo control package for Raspberry Pi GPIO using LGPIO.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sg90_servo_node = sg90_servo_control.servo_node:main",
            "ros2_control_servo_bridge = sg90_servo_control.ros2_control_servo_bridge:main",
            "sg90_web_interface = sg90_servo_control.web_interface:main",
        ],
    },
)
