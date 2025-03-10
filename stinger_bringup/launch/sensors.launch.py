'''
This launches the bringup of IMU, GPS, camera, and LiDAR.
'''

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='stinger_bringup',
            executable='imu-node',
            name='imu_node',
        ),
        Node(
            package='stinger_bringup',
            executable='gps-node',
            name='gps_node',
        ),
        Node(
            package='stinger_bringup',
            executable='camera-node',
            name='camera_node',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('sllidar_ros2'),
                    'launch',
                    'sllidar_c1_launch.py'
                ])
            ])),
        Node(
            package='stinger_bringup',
            executable='filter-node',
            name='filter_node',
        ),
    ])