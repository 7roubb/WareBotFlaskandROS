from setuptools import setup

package_name = 'warebot_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': '.'},
    install_requires=['setuptools', 'paho-mqtt'],
    zip_safe=True,
    maintainer='Osama',
    maintainer_email='osama@super.ai',
    description='MQTT Task Executor for MP-400 robots',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mqtt_task_executor = warebot_robot.mqtt_task_executor:main',
        ],
    },
)
