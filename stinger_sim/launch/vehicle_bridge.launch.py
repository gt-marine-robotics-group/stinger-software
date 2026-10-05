import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_bridge(context):
    model_name = LaunchConfiguration('model_name').perform(context)
    topic_prefix = LaunchConfiguration('topic_prefix').perform(context).strip('/')
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    template_path = os.path.join(
        get_package_share_directory('stinger_sim'),
        'config',
        'vehicle_bridge.yml.in',
    )
    with open(template_path, encoding='utf-8') as template_file:
        config = template_file.read()

    config = config.replace('@TOPIC_PREFIX@', topic_prefix)
    config = config.replace('@MODEL_NAME@', model_name)
    config = config.replace('@JOINT_PREFIX@', frame_prefix)
    config_path = os.path.join(
        tempfile.gettempdir(), f'stinger_bridge_{model_name}.yml'
    )
    with open(config_path, 'w', encoding='utf-8') as config_file:
        config_file.write(config)

    return [Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name=f'{model_name}_bridge',
        parameters=[{'config_file': config_path}],
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model_name', default_value='stinger'),
        DeclareLaunchArgument('topic_prefix', default_value='stinger'),
        DeclareLaunchArgument('frame_prefix', default_value=''),
        OpaqueFunction(function=_launch_bridge),
    ])