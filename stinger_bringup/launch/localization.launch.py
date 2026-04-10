'''
This launches the robot localization package
This fuses the IMU and GPS data
This publishes topic /odom
'''

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('stinger_bringup')

    robot_localization_file_path = (pkg_share + '/config/ekf.yaml')
    navsat_transform_file_path = (pkg_share + '/config/navsat_transform.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[robot_localization_file_path],
        ),    
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_static_publisher',
            # Arguments: X Y Z Yaw Pitch Roll Parent_Frame Child_Frame
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'] 
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[navsat_transform_file_path],
            respawn=True,
            remappings=[
            # TODO: 4.4.b Navsat Node
            # Example: (topic, remaped_topic)
            ### STUDENT CODE HERE
                ('imu', '/stinger/imu/relative'), 
                ('gps/fix', '/stinger/gps/fix'),
                ('odometry/filtered', '/odometry/filtered')
            ### END STUDENT CODE
            ],
        ),
        Node(
            package='stinger_bringup',
            executable='imu_republisher',
            name='imu_republisher',
            output='screen'
        )
    ])
