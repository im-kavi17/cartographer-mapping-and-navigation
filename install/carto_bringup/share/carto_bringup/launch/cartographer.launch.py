import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    carto_bringup_dir = get_package_share_directory(
        'carto_bringup'
    )

    rplidar_dir = get_package_share_directory(
        'rplidar_ros'
    )

    cartographer_config_dir = os.path.join(
        carto_bringup_dir,
        'config'
    )

    urdf_file = os.path.join(
        carto_bringup_dir,
        'urdf',
        'rover.urdf'
    )

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    return LaunchDescription([


        # =====================================================
        # 1. Robot State Publisher
        # =====================================================

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',

            parameters=[
                {
                    'robot_description': robot_description
                }
            ],

            output='screen'
        ),


        # =====================================================
        # 2. RPLIDAR A1
        # =====================================================

        IncludeLaunchDescription(

            PythonLaunchDescriptionSource(
                os.path.join(
                    rplidar_dir,
                    'launch',
                    'rplidar_a1_launch.py'
                )
            ),

            launch_arguments={

                'serial_port': '/dev/ttyUSB1',

                'frame_id': 'laser_frame',

                'scan_mode': 'Sensitivity'

            }.items()
        ),


        # =====================================================
        # 3. Cartographer
        # =====================================================

        Node(

            package='cartographer_ros',

            executable='cartographer_node',

            name='cartographer_node',

            output='screen',

            arguments=[

                '-configuration_directory',

                cartographer_config_dir,

                '-configuration_basename',

                'cartographer.lua'

            ],

            remappings=[

                ('scan', '/scan')

            ]
        ),


        # =====================================================
        # 4. Cartographer Occupancy Grid
        # =====================================================

        Node(

            package='cartographer_ros',

            executable='cartographer_occupancy_grid_node',

            name='cartographer_occupancy_grid_node',

            output='screen',

            arguments=[

                '-resolution',
                '0.05',

                '-publish_period_sec',
                '1.0'

            ]
        ),


        # =====================================================
        # 5. RViz2
        # =====================================================

        Node(

            package='rviz2',

            executable='rviz2',

            name='rviz2',

            output='screen'

        )

    ])
