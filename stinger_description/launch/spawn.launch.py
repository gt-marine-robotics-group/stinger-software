from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os
import xacro

def generate_launch_description():
    gazebo_arg = DeclareLaunchArgument('gazebo', default_value='True')
    gazebo_config = LaunchConfiguration('gazebo', default='True')

    # URDF File Path
    xacro_file = os.path.join(
        get_package_share_directory('stinger_description'),
        'urdf',
        'stinger_tug.urdf.xacro'
    )
    
    # Get URDF from xacro
    robot_description = xacro.process(xacro_file)

    # Robot description publisher
    robot_state_publisher = Node(
        name = 'robot_state_publisher',
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [{'robot_description': robot_description}]
    )

    # URDF spawner
    args = ['-name', 'stinger', '-topic', 'robot_description']
    spawn = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=args, 
        output='screen',
        condition=IfCondition(gazebo_config)
    )

    return LaunchDescription([
        gazebo_arg,
        robot_state_publisher,
        spawn
    ])