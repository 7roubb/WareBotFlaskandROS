from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mp400_footprint_switcher'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.launch.py'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Dynamic footprint switching for Neobotix MP-400 robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'footprint_switcher_node = mp400_footprint_switcher.footprint_switcher_node:main',
        ],
    },
)
