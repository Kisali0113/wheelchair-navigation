from setuptools import find_packages, setup

package_name = 'my_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'google-cloud-speech',
        'google-cloud-texttospeech',
        'google-genai',
        'pyaudio',
        'SpeechRecognition',  # Optional, for easier STT
        'pyserial',
        'opencv-python',
        'numpy',
        'ultralytics',
        'scikit-learn',
        'pyrealsense2',
        'deep-sort-realtime',
    ],
    zip_safe=True,
    maintainer='janidu',
    maintainer_email='janidu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gate_controller = my_robot_pkg.gate_controller:main',
            'speaker_node = my_robot_pkg.speaker_node:main',
            'odom_localizer = my_robot_pkg.odom_localizer:main',
            'human_position_node = my_robot_pkg.human_position_node:main',
            'person_tracker_node = my_robot_pkg.person_tracker_node:main',
            'goal_sender_node = my_robot_pkg.goal_sender_node:main',
            'path_receiver_node = my_robot_pkg.path_receiver_node:main',
            'pure_pursuit_controller_node = my_robot_pkg.pure_pursuit_controller_node:main',
            'pwm_conversion_node = my_robot_pkg.pwm_conversion_node:main',
        ],
    },
)
