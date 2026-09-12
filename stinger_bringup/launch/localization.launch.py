'''
This launches the robot localization package
This fuses the IMU and GPS data
This publishes topic /odom
'''

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import xacro

def generate_launch_description():

    pkg_share = get_package_share_directory('stinger_bringup')
    desc_share = get_package_share_directory('stinger_description')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    sim_time_param = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')
    navsat_config = os.path.join(pkg_share, 'config', 'navsat_transform.yaml')
    xacro_file = os.path.join(desc_share, 'urdf', 'stinger_tug.urdf.xacro')
    robot_description = xacro.process(xacro_file)

    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            parameters=[ekf_config, sim_time_param],
            output='screen'
        ),    
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            parameters=[navsat_config, sim_time_param],
            respawn=True,
            remappings=[
            # TODO: 4.4.b Navsat Node
            # Example: (topic, remaped_topic)
            ### STUDENT CODE HERE

            ### END STUDENT CODE
            ],
            output='screen'
        ),
        Node(
            package='stinger_bringup',
            executable='imu_republisher',
            name='imu_republisher',
            parameters=[sim_time_param],
            output='screen'
        )
    ])
