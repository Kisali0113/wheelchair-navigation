import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'firebase_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kisali',
    maintainer_email='janidu@example.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'firebase_listener = firebase_bridge.firebase_listener:main',
            'firebase_publisher = firebase_bridge.firebase_publisher:main',
            'goal_bridge = firebase_bridge.goal_bridge:main',
        ],
    },
)
