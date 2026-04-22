import os

from ament_index_python.packages import get_package_share_directory  # type: ignore
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription  # type: ignore
from launch.conditions import IfCondition  # type: ignore
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource  # type: ignore
from launch.substitutions import LaunchConfiguration, PythonExpression  # type: ignore


def generate_launch_description():
	mode = LaunchConfiguration('mode')

	navigation2_run_dir = get_package_share_directory('navigation2_run')
	localization_real_dir = get_package_share_directory('sima-localization-real')
	localization_sim_dir = get_package_share_directory('sima-localization-sim')
	ninja_sima_main_dir = get_package_share_directory('ninja-sima-main')

	declare_mode_arg = DeclareLaunchArgument(
		'mode',
		default_value='real',
		choices=['real', 'sim'],
		description='Bringup mode: real or sim'
	)

	nav2_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(navigation2_run_dir, 'launch', 'real_launch.py')
		)
	)

	localization_real_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(localization_real_dir, 'launch', 'robot_localization.launch.py')
		),
		condition=IfCondition(PythonExpression(["'", mode, "' == 'real'"])),
	)

	localization_sim_launch = IncludeLaunchDescription(
		AnyLaunchDescriptionSource(
			os.path.join(localization_sim_dir, 'launch', 'sima_localization_sim_no-rviz.launch')
		),
		condition=IfCondition(PythonExpression(["'", mode, "' == 'sim'"])),
	)

	ninja_sima_main_launch = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			os.path.join(ninja_sima_main_dir, 'launch', 'ninja-sima-main.launch.py')
		)
	)

	return LaunchDescription([
		declare_mode_arg,
		nav2_launch,
		localization_real_launch,
		localization_sim_launch,
		ninja_sima_main_launch,
	])
