from setuptools import setup

package_name = "fsr_mcp3202"

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
    maintainer_email="tbem@example.com",
    description="MCP3202 FSR reader and ROS2 publisher",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fsr_publisher = fsr_mcp3202.fsr_publisher:main",
        ],
    },
)
