from setuptools import setup, find_packages
import os

package_name = 'warebot_task_runner'

setup(
    name='warebot_task_runner',  # ✅ Changed from 'warebot-task-runner'
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Package metadata
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name,
            ['package.xml']),
        
        # Launch files
        ('share/' + package_name + '/launch',
            [
                'launch/task_runner.launch.py',
                'launch/simulation_fleet.launch.py',
            ]),
        
        # Configuration and calibration files
        ('share/' + package_name + '/config',
            [
                'config/calibration.npy',
                'config/task_runner.yaml',
                'config/locations.yaml',
            ]),
    ],
    install_requires=[
        'setuptools',
        'paho-mqtt>=2.0.0',
        'opencv-python>=4.5.0',
        'numpy>=1.19.0',
        'pupil-apriltags>=1.0.0',
        'pyserial>=3.5',
    ],
    zip_safe=True,
    maintainer='super',
    maintainer_email='super@example.com',
    description='WareBot task runner node with AprilTag alignment integration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_runner = warebot_task_runner.main:main',
            'robot_monitor = warebot_task_runner.robot_monitor:main',
        ],
    },
)