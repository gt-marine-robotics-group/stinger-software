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

    spawn_vehicle_1 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('stinger_description'),
                'launch',
                'spawn.launch.py'
            ])
        ),
        launch_arguments={
            'robot_name': 'stinger_1',
            'x': '0.0',
            'y': '0.0',
        }.items(),
    )
    ld.append(spawn_vehicle_1)

    spawn_vehicle_2 = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('stinger_description'),
                'launch',
                'spawn.launch.py'
            ])
        ),
        launch_arguments={
            'robot_name': 'stinger_2',
            'x': '0.0',
            'y': '5.0',
        }.items(),
    )
    ld.append(spawn_vehicle_2)

    # TODO: Uncomment after completing section 4
    localization = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('stinger_bringup'),
                'launch',
                'localization.launch.py'
            ]),
        ),
    )
    # Delay to allow sensors to populate
    delayed_localization = TimerAction(period=5.0, actions=[localization])
    ld.append(delayed_localization)

    return LaunchDescription(ld)