'''
This launches the robot localization package
This fuses the IMU and GPS data
This publishes topic /odom
'''

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
import xacro


def launch_robot_description(context):
    """Generate the optional description with the same prefixes as spawning."""
    robot_name = LaunchConfiguration('robot_name').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    topic_prefix = (
        LaunchConfiguration('topic_prefix').perform(context).strip('/')
        or robot_name.strip('/')
    )
    xacro_file = os.path.join(
        get_package_share_directory('stinger_description'),
        'urdf',
        'stinger_tug.urdf.xacro',
    )
    robot_description = xacro.process(
        xacro_file,
        mappings={'prefix': prefix, 'topic_prefix': topic_prefix},
    )
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=robot_name,
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool,
                ),
            }],
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
        ),
    ]

def generate_launch_description():

    pkg_share = get_package_share_directory('stinger_bringup')
    robot_name = LaunchConfiguration('robot_name')
    prefix = LaunchConfiguration('prefix')
    use_sim_time = LaunchConfiguration('use_sim_time')
    imu_topic = LaunchConfiguration('imu_topic')
    gps_odom_topic = LaunchConfiguration('gps_odom_topic')
    robot_localization_file_path = pkg_share + '/config/ekf.yaml'
    navsat_transform_file_path = pkg_share + '/config/navsat_transform.yaml'

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='stinger'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use the tutorial simulation clock; hardware launches must pass false',
        ),
        DeclareLaunchArgument(
            'publish_robot_description', default_value='false',
            description='Publish robot transforms when no other launch provides them',
        ),
        DeclareLaunchArgument('topic_prefix', default_value=''),
        DeclareLaunchArgument('imu_topic', default_value='imu/relative'),
        DeclareLaunchArgument('gps_odom_topic', default_value='odometry/gps'),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            namespace=robot_name,
            parameters=[robot_localization_file_path, {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'imu0': imu_topic,
                'odom0': gps_odom_topic,
                'map_frame': [prefix, 'map'],
                'odom_frame': [prefix, 'odom'],
                'base_link_frame': [prefix, 'base_link'],
                'world_frame': [prefix, 'odom'],
            }],
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform_node',
            namespace=robot_name,
            parameters=[navsat_transform_file_path, {
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            }],
            respawn=True,
            remappings=[
                ('imu', imu_topic),
                ('odometry/gps', gps_odom_topic),
            ],
            output='screen',
        ),
        Node(
            package='stinger_bringup',
            executable='imu_republisher',
            name='imu_republisher',
            namespace=robot_name,
            parameters=[{
                'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
                'input_topic': 'imu/data',
                'output_topic': imu_topic,
                'target_frame': [prefix, 'base_link'],
                'output_frame': [prefix, 'base_link'],
            }],
            output='screen'
        ),
        OpaqueFunction(
            function=launch_robot_description,
            condition=IfCondition(LaunchConfiguration('publish_robot_description')),
        ),
    ])
