from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'carto_bringup'

setup(
    name=package_name,
    version='0.0.1',

    packages=find_packages(),

    data_files=[
        # ROS 2 package index
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),

        # package.xml
        (
            'share/' + package_name,
            ['package.xml']
        ),

        # ALL launch files
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')
        ),

        # Cartographer Lua + Nav2 YAML
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.lua') +
            glob('config/*.yaml')
        ),

        # URDF
        (
            os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')
        ),

        # RViz configurations
        (
            os.path.join('share', package_name, 'rviz'),
            glob('rviz/*')
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',

    description='Cartographer 2D SLAM and Nav2 navigation for RPLIDAR A1',
    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [],
    },
)
