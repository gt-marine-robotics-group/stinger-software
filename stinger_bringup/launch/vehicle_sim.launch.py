from ament_index_python import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.actions import TimerAction

def generate_launch_description():

    ld = []

    world_arg = DeclareLaunchArgument(
        'world',
        default_value = 'default.world'
    )
    world = LaunchConfiguration('world')
    ld.append(world_arg)

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use the Gazebo clock for simulated vehicle nodes'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')
    ld.append(use_sim_time_arg)

    gzsim = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('stinger_sim'),
                'launch',
                'sim.launch.py'
            ])
        ),
        launch_arguments = {'world': world}.items()
    )
    ld.append(gzsim)

    spawn_vehicle = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('stinger_description'),
                'launch',
                'spawn.launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    ld.append(spawn_vehicle)

    # TODO: Uncomment after completing section 4
    # localization = launch.actions.IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             get_package_share_directory('stinger_bringup'),
    #             'launch',
    #             'localization.launch.py'
    #         ]),
    #     ),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'publish_robot_description': 'false'
    #     }.items()
    # )
    # # Delay to allow sensors to populate
    # delayed_localization = TimerAction(period=5.0, actions=[localization])
    # ld.append(delayed_localization)

    return LaunchDescription(ld)
