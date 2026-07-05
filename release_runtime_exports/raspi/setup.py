from setuptools import find_packages, setup

package_name = 'pfe_raspi_runtime'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tbem',
    maintainer_email='benmekkitarek@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_mjpeg_server = pfe_raspi_runtime.camera_mjpeg_server:main',
            'raspi_heartbeat_node = pfe_raspi_runtime.raspi_heartbeat_node:main',
            'force_sensor_node = pfe_raspi_runtime.force_sensor_node:main',
        ],
    },
)
