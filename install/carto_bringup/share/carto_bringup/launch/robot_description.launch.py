#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('carto_bringup')
    urdf_file = os.path.join(
        pkg_share,
        'urdf',
        'rover.urdf'
    )
    with open(urdf_file, 'r') as f:
        robot_description = f.read()
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': ParameterValue(
                        robot_description,
                        value_type=str
                    ),
                    'use_sim_time': False
                }
            ]
        )
    ])
