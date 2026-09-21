#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('carto_bringup')

    # Files
    nav2_params = os.path.join(
        pkg_share,
        'config',
        'nav2_params.yaml'
    )

    urdf_file = os.path.join(
        pkg_share,
        'urdf',
        'rover.urdf'
    )

    rviz_file = os.path.join(
        pkg_share,
        'rviz',
        'navigation.rviz'
    )

    map_file = os.path.expanduser(
        '~/carto_ws/maps/my_map.yaml'
    )

    # Read URDF
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([

        # --------------------------------------------------
        # MAP
        # --------------------------------------------------

        DeclareLaunchArgument(
            'map',
            default_value=map_file
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                nav2_params,
                {
                    'yaml_filename': LaunchConfiguration('map'),
                    'use_sim_time': False
                }
            ]
        ),

        # --------------------------------------------------
        # ROBOT STATE PUBLISHER
        # --------------------------------------------------

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    robot_description,
                    value_type=str
                ),
                'use_sim_time': False
            }]
        ),

        # --------------------------------------------------
        # AMCL
        # --------------------------------------------------

        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params]
        ),

        # --------------------------------------------------
        # PLANNER
        # --------------------------------------------------

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params]
        ),

        # --------------------------------------------------
        # CONTROLLER
        # --------------------------------------------------

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[nav2_params]
        ),

        # --------------------------------------------------
        # BT NAVIGATOR
        # --------------------------------------------------

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params]
        ),

        # --------------------------------------------------
        # BEHAVIOR SERVER
        # --------------------------------------------------

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params]
        ),

        # --------------------------------------------------
        # NAV2 LIFECYCLE MANAGER
        # --------------------------------------------------

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'autostart': True,
                'node_names': [
                    'map_server',
                    'amcl',
                    'planner_server',
                    'controller_server',
                    'bt_navigator',
                    'behavior_server'
                ]
            }]
        ),

        # --------------------------------------------------
        # RVIZ
        # --------------------------------------------------

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=[
                '-d',
                rviz_file
            ]
        )
    ])
