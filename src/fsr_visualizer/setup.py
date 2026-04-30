from setuptools import setup

package_name = "fsr_visualizer"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="tbem",
    maintainer_email="benmekkitarek@gmail.com",
    description="Colored FSR gauge visualizer for ROS2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fsr_gauge = fsr_visualizer.fsr_gauge:main",
        ],
    },
)
