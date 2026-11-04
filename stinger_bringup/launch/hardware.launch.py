'''
One-command hardware bringup for a real Stinger tug (multi-vehicle branch).

Starts the same things as running these by hand in separate terminals:
    ros2 launch stinger_bringup sensors.launch.py
    ros2 launch stinger_bringup localization.launch.py use_sim_time:=false publish_robot_description:=true
    ros2 run stinger_bringup motor-node

Usage:
    ros2 launch stinger_bringup hardware.launch.py
    ros2 launch stinger_bringup hardware.launch.py motors:=false    # everything except the motors

The keyboard drive (teleop-keyboard) still runs in its own terminal, because
it needs to read your key presses.
'''

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # On/off switches, e.g. `motors:=false` on the command line skips the motors
    sensors = LaunchConfiguration('sensors')
    localization = LaunchConfiguration('localization')
    motors = LaunchConfiguration('motors')

    launch_dir = PathJoinSubstitution([FindPackageShare('stinger_bringup'), 'launch'])

    return LaunchDescription([
        DeclareLaunchArgument('sensors', default_value='true',
                              description='Start the IMU, GPS, camera and LiDAR'),
        DeclareLaunchArgument('localization', default_value='true',
                              description='Start localization (EKF, navsat, robot frames)'),
        DeclareLaunchArgument('motors', default_value='true',
                              description='Start the motor driver (motor-node)'),

        # 1. Same as: ros2 launch stinger_bringup sensors.launch.py
        GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([launch_dir, 'sensors.launch.py'])
                    ),
                ),
            ],
            condition=IfCondition(sensors),
        ),

        # 2. Same as: ros2 launch stinger_bringup localization.launch.py
        #             use_sim_time:=false publish_robot_description:=true
        GroupAction(
            [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([launch_dir, 'localization.launch.py'])
                    ),
                    launch_arguments={
                        'use_sim_time': 'false',
                        'publish_robot_description': 'true',
                    }.items(),
                ),
            ],
            condition=IfCondition(localization),
        ),

        # 3. Same as: ros2 run stinger_bringup motor-node
        Node(
            package='stinger_bringup',
            executable='motor-node',
            name='esc_control_node',
            output='screen',
            condition=IfCondition(motors),
        ),
    ])
