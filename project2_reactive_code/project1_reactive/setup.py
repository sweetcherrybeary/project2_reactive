from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'project1_reactive'

setup(
    name=package_name,
    version='2.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Colin',
    maintainer_email='coliniswhat897@gmail.com',
    description='CS 4023/5023 Projects 1-2: reactive TurtleBot 4 with mapping.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive_controller = project1_reactive.reactive_controller:main',
            'occupancy_mapper = project1_reactive.occupancy_mapper:main',
        ],
    },
)
