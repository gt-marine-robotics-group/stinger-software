import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_vehicle(context):
    vehicle_namespace = LaunchConfiguration(
        'vehicle_namespace',
    ).perform(context).strip('/')
    if not vehicle_namespace:
        raise ValueError('vehicle_namespace must not be empty')

    sim = LaunchConfiguration('sim').perform(context)
    if sim not in ('0', '1'):
        raise ValueError("sim must be '0' or '1'")
    use_sim_time = 'true' if sim == '1' else 'false'

    description_launch = os.path.join(
        get_package_share_directory('stinger_description'),
        'launch',
        'spawn.launch.py',
    )
    localization_launch = os.path.join(
        get_package_share_directory('stinger_bringup'),
        'launch',
        'localization.launch.py',
    )

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(description_launch),
            launch_arguments={
                'gazebo': 'false',
                'robot_name': vehicle_namespace,
                'prefix': f'{vehicle_namespace}_',
                'topic_prefix': vehicle_namespace,
                'use_sim_time': use_sim_time,
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(localization_launch),
            launch_arguments={
                'robot_name': vehicle_namespace,
                'prefix': f'{vehicle_namespace}_',
                'topic_prefix': vehicle_namespace,
                'use_sim_time': use_sim_time,
                'publish_robot_description': 'false',
            }.items(),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_namespace',
            default_value='stinger_1',
            description='Namespace and topic prefix for this Stinger',
        ),
        DeclareLaunchArgument(
            'vehicle_instance_number',
            default_value='1',
            description='Lyoko vehicle instance number',
        ),
        DeclareLaunchArgument(
            'sim',
            default_value='1',
            description='Set to 1 when using simulation time',
        ),
        OpaqueFunction(function=launch_vehicle),
    ])
