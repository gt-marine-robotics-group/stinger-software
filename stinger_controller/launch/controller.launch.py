'''Launch the tutorial velocity controller and thruster allocation node.'''

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the tutorial simulation clock; hardware launches must pass false'
    )
    sim_time_param = {'use_sim_time': LaunchConfiguration('use_sim_time')}

    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='stinger_controller',
            executable='throttle_controller',
            output='screen',
            name='throttle_controller',
            parameters=[sim_time_param],
        ),
        Node(
            package='stinger_controller',
            executable='velocity_controller',
            output='screen',
            name='velocity_controller',
            parameters=[sim_time_param],
        ),
    ])
