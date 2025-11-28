from setuptools import find_packages, setup

package_name = 'warebot_task_navigator'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='WareBotTeam',
    maintainer_email='robot@example.com',
    description='ROS2 package for robot task navigation and movement control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_navigator = warebot_task_navigator.task_navigator:main',
            'task_executor = warebot_task_navigator.task_executor:main',
            'reference_point_manager = warebot_task_navigator.reference_point_manager:main',
        ],
    },
)
