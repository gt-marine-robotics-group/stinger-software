from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

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
            executable='direct_thrust_controller',
            output='screen',
            name='direct_thrust_controller',
            parameters=[sim_time_param],
        ),
    ])
