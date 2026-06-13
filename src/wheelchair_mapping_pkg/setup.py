import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'wheelchair_mapping_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    (os.path.join('share', package_name, 'config'), glob('config/*')),
    (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='janidu',
    maintainer_email='janidu@example.com',
    description='ROS2 package for wheelchair mapping with LiDAR SLAM',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_sensor_node = wheelchair_mapping_pkg.serial_sensor_node:main',
            'encoder_odom_node = wheelchair_mapping_pkg.encoder_odom_node:main',
            'cmd_vel_publisher = wheelchair_mapping_pkg.cmd_vel_publisher:main',
            'person_tracker_node = wheelchair_mapping_pkg.person_tracker_node:main',
        ],
    },
)
