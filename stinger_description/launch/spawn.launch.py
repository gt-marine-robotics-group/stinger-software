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
    robot_name_arg = DeclareLaunchArgument('robot_name', default_value='stinger')
    robot_name = LaunchConfiguration('robot_name')
    x_arg = DeclareLaunchArgument('x', default_value='0.0')
    x = LaunchConfiguration('x')
    y_arg = DeclareLaunchArgument('y', default_value='0.0')
    y = LaunchConfiguration('y')
    z_arg = DeclareLaunchArgument('z', default_value='0.0')
    z = LaunchConfiguration('z')

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
        namespace = robot_name,
        package = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output = 'screen',
        parameters = [{'robot_description': robot_description}]
    )

    # URDF spawner
    args = [
        '-name', robot_name,
        '-topic', ["/", robot_name, '/robot_description'],
        '-x', x,
        '-y', y,
        '-z', z,
    ]
    spawn = Node(
        package='ros_gz_sim', 
        executable='create', 
        arguments=args, 
        output='screen',
        condition=IfCondition(gazebo_config)
    )

    return LaunchDescription([
        gazebo_arg,
        robot_name_arg,
        x_arg,
        y_arg,
        z_arg,
        robot_state_publisher,
        spawn
    ])