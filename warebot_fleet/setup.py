from setuptools import setup

package_name = 'warebot_fleet'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': '.'},
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='Osama',
    maintainer_email='super@super.com',
    description='ROS2 Fleet → MQTT Bridge for MP-400',
    license='MIT',
    entry_points={
        'console_scripts': [
            'robot_state_mqtt_bridge = warebot_fleet.robot_state_mqtt_bridge:main',
            'map_to_mqtt = warebot_fleet.map_to_mqtt:main',
        ],
    },
)
