from setuptools import find_packages, setup

package_name = 'mp400_apriltag_align'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/launch', ['launch/apriltag_align_launch.py']),    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neobotix',
    maintainer_email='neobotix@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'apriltag_align = mp400_apriltag_align.apriltag_align:main',        ],
    },
)
