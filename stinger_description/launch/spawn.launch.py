from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
import xacro

def launch_robot(context):
    gazebo = LaunchConfiguration('gazebo').perform(context)
    robot_name = LaunchConfiguration('robot_name').perform(context)
    prefix = LaunchConfiguration('prefix').perform(context)
    topic_prefix = LaunchConfiguration('topic_prefix').perform(context).strip('/')
    if not topic_prefix:
        topic_prefix = robot_name
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() == 'true'
    x = LaunchConfiguration('x').perform(context)
    y = LaunchConfiguration('y').perform(context)
    z = LaunchConfiguration('z').perform(context)

    # URDF File Path
    xacro_file = os.path.join(
        get_package_share_directory('stinger_description'),
        'urdf',
        'stinger_tug.urdf.xacro'
    )
    
    # Get URDF from xacro
    robot_description = xacro.process(
        xacro_file,
        mappings={'prefix': prefix, 'topic_prefix': topic_prefix},
    )

    # Robot description publisher
    robot_state_publisher = Node(
        name='robot_state_publisher',
        namespace=robot_name,
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
        remappings=[('tf', '/tf'), ('tf_static', '/tf_static')],
    )

    # URDF spawner
    args = [
        '-name', robot_name,
        '-topic', f'/{robot_name}/robot_description',
        '-x', x,
        '-y', y,
        '-z', z,
    ]
    spawn = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=args, 
        output='screen',
        condition=IfCondition(gazebo)
    )

    return [robot_state_publisher, spawn]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('gazebo', default_value='True'),
        DeclareLaunchArgument('robot_name', default_value='stinger'),
        DeclareLaunchArgument('prefix', default_value=''),
        DeclareLaunchArgument('topic_prefix', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        OpaqueFunction(function=launch_robot),
    ])
