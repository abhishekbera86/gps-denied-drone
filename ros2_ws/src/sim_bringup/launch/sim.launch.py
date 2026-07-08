"""sim.launch.py — bring up the autonomy stack against PX4 SITL (simulation).

The single sim-side entry point. It layers the simulation params
(config/sim_params.yaml) onto the shared, transport-agnostic
common_missions/autonomy.launch.py. The PX4 SITL + Gazebo world and the
uXRCE-DDS UDP agent are already running (the px4-sim / ros2-autonomy
containers) — this only starts the flight/mission node.

The hw_bringup package is the mirror of this file for real hardware: same
autonomy include, different params, plus a serial DDS agent. Nothing about
the flight logic differs between them.

Examples:
  ros2 launch sim_bringup sim.launch.py action:=hover
  ros2 launch sim_bringup sim.launch.py action:=mission mission:=survey
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    autonomy_launch = os.path.join(
        get_package_share_directory('common_missions'), 'launch', 'autonomy.launch.py')
    sim_params = os.path.join(
        get_package_share_directory('sim_bringup'), 'config', 'sim_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'action', default_value='mission',
            description="What to fly: 'hover' or 'mission'"),
        DeclareLaunchArgument(
            'mission', default_value='square',
            description='Mission to fly when action=mission: square or survey'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(autonomy_launch),
            launch_arguments={
                'action': LaunchConfiguration('action'),
                'mission': LaunchConfiguration('mission'),
                'params_file': sim_params,
            }.items(),
        ),
    ])
