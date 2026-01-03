from setuptools import setup
import os

package_name = 'warebot_task_runner'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
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
            ]),
        
        # Configuration and calibration files
        ('share/' + package_name + '/config',
            [
                'config/camera_matrix.npy',
                'config/dist_coeffs.npy',
                'config/rvecs.npy',
                'config/tvecs.npy',
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
    ],
    zip_safe=True,
    maintainer='super',
    maintainer_email='super@example.com',
    description='WareBot task runner node with AprilTag alignment integration',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_runner_integrated = warebot_task_runner.task_runner_integrated:main',
            'task_runner = warebot_task_runner.task_runner:main',
        ],
    },
)