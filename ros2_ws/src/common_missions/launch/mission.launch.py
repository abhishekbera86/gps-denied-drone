"""Launch a mission by name: ros2 launch common_missions mission.launch.py mission:=square

Missions are console_scripts in this package named <mission>_mission; the
mission:= argument picks which one to run.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MISSIONS = ('square', 'survey')


def launch_mission(context):
    mission = LaunchConfiguration('mission').perform(context)
    if mission not in MISSIONS:
        raise ValueError(f"Unknown mission '{mission}' — choose from {MISSIONS}")

    return [Node(
        package='common_missions',
        executable=f'{mission}_mission',
        output='screen',
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'mission',
            default_value='square',
            description=f'Mission to fly: one of {MISSIONS}',
        ),
        OpaqueFunction(function=launch_mission),
    ])
