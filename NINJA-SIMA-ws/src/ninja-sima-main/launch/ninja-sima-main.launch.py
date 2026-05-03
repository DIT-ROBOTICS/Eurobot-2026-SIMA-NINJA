from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    team = LaunchConfiguration('team')

    declare_team_arg = DeclareLaunchArgument(
        'team',
        default_value='blue',
        choices=['blue', 'yellow'],
        description='Team side for waypoint parameters'
    )

    ninja_sima_main_node = Node(
        package='ninja-sima-main',
        executable='ninja_sima_main_node',
        name='ninja_sima_main_node',
        output='screen',
        emulate_tty=True,
        parameters=[{'team': team}]
    )

    return LaunchDescription([
        declare_team_arg,
        ninja_sima_main_node
    ])
