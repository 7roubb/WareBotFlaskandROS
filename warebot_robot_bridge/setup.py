from setuptools import setup

package_name = 'warebot_robot_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['warebot_robot_bridge/robot_monitor.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='osama',
    maintainer_email='robot@example.com',
    description='Robot telemetry → MQTT bridge',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_monitor_mqtt = warebot_robot_bridge.robot_monitor_mqtt:main',
        ],
    },
)
