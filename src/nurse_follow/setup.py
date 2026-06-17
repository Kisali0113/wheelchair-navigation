from setuptools import find_packages, setup

package_name = 'nurse_follow'

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
            'person_detector = nurse_follow.person_detector:main',
            'follow_controller = nurse_follow.follow_controller:main',
            'mode_manager = nurse_follow.mode_manager:main',
        ],
    },
)
