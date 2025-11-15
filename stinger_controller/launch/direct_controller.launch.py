from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='stinger_controller',
            executable='throttle_controller',
            output='screen',
            name='throttle_controller',
        ),
        Node(
            package='stinger_controller',
            executable='direct_thrust_controller',
            output='screen',
            name='direct_thrust_controller',
        ),
    ])
