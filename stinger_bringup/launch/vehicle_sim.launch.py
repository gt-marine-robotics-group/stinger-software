from ament_index_python import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    world_arg = DeclareLaunchArgument(
        'world',
        default_value = 'default.world'
    )
    world = LaunchConfiguration('world')

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

    spawn_vehicle = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('stinger_description'),
                'launch',
                'spawn.launch.py'
            ])
        ),
    )

    return LaunchDescription([
        world_arg,
        gzsim,
        spawn_vehicle,
    ])