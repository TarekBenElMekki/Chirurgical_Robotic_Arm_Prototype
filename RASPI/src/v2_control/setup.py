from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'v2_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='master_thesis',
    maintainer_email='benmekkitarek@gmail.com',
    description='V2 Raspberry Pi final-position motor control',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'final_position_controller = v2_control.final_position_controller:main',
            'servo_driver = v2_control.servo_driver:main',
        ],
    },
)
