from setuptools import setup

package_name = 'warebot_task_runner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/task_runner.launch.py']),
        ('share/' + package_name + '/config', ['config/task_runner.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='super',
    maintainer_email='super@example.com',
    description='WareBot task runner node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_runner = warebot_task_runner.task_runner:main',
        ],
    },
)
