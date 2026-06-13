from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robotdesign'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/robotdesign']),
    ('share/robotdesign', ['package.xml','model.config']),

    # Launch files
    ('share/robotdesign/launch', glob('launch/*.py')),

    # SDF/URDF files
    ('share/robotdesign/urdf', glob('urdf/*')),
    ('share/robotdesign/worlds', glob('worlds/*')),

    # Mesh files (THIS is your missing part)
    ('share/robotdesign/meshes', glob('meshes/*.STL')),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kisali',
    maintainer_email='kisali@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'pure = robotdesign.pure_pursuit_controller_node : main',
        'ackermann_to_joint = robotdesign.ackermann_to_joint:main',
        ],
    },
)
