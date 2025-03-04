'''
This launches the robot localization package
This fuses the IMU and GPS data
This publishes topic /odom
'''

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('stinger_bringup')

    robot_localization_file_path = (pkg_share + '/config/ekf.yaml')
    navsat_transform_file_path = (pkg_share + '/config/navsat_transform.yaml')

    return LaunchDescription([
        # Node(
        #     package='stinger_bringup',
        #     executable='imu-node',
        #     name='imu_node',
        # ),
        # Node(
        #     package='stinger_bringup',
        #     executable='gps-node',
        #     name='gps_node',
        # ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[robot_localization_file_path],
            remappings=[
            ("/example/imu", "/imu/data"),
            ],
        ),    
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[navsat_transform_file_path],
            respawn=True,
            remappings=[
            ('/imu', '/imu/data'),
            ],
        )
    ])
