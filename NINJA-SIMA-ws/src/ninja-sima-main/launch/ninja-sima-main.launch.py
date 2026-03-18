import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('ninja-sima-main')
    
    # Parameters file path
    main_param_path = os.path.join(pkg_dir, 'params', 'main_param.yaml')

    ninja_sima_main_node = Node(
        package='ninja-sima-main',
        executable='ninja_sima_main_node',
        name='ninja_sima_main_node',
        output='screen',
        emulate_tty=True,
        parameters=[main_param_path]
    )

    return LaunchDescription([
        ninja_sima_main_node
    ])
