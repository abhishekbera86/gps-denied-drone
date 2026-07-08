"""autonomy.launch.py — the shared, transport-agnostic autonomy entry point.

This is the ONE launch file that actually starts the flight/mission nodes.
It is deliberately transport-agnostic: it knows nothing about UDP vs serial,
sim vs real, or where the params come from. The sim_bringup and hw_bringup
packages each include this file, passing their own `params_file` — that is the
entire sim-vs-real difference at the autonomy layer.

Arguments:
  action=hover|mission   what to fly (default: mission)
  mission=square|survey  which mission, when action=mission (default: square)
  params_file=<path>     optional ROS 2 params YAML applied to the node
                         (empty = use the nodes' built-in defaults)

Examples:
  ros2 launch common_missions autonomy.launch.py action:=hover
  ros2 launch common_missions autonomy.launch.py action:=mission mission:=survey \
      params_file:=/path/to/sim_params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MISSIONS = ('square', 'survey')


def launch_autonomy(context):
    action = LaunchConfiguration('action').perform(context)
    mission = LaunchConfiguration('mission').perform(context)
    params_file = LaunchConfiguration('params_file').perform(context)

    # Only attach a parameters= entry when a file was actually given, so the
    # nodes fall back to their in-code defaults when run bare.
    parameters = [params_file] if params_file else []

    if action == 'hover':
        package, executable = 'common_control', 'offboard_control_node'
    elif action == 'mission':
        if mission not in MISSIONS:
            raise ValueError(f"Unknown mission '{mission}' — choose from {MISSIONS}")
        package, executable = 'common_missions', f'{mission}_mission'
    else:
        raise ValueError(f"Unknown action '{action}' — choose 'hover' or 'mission'")

    return [Node(
        package=package,
        executable=executable,
        output='screen',
        parameters=parameters,
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'action', default_value='mission',
            description="What to fly: 'hover' (takeoff/hover/land) or 'mission'"),
        DeclareLaunchArgument(
            'mission', default_value='square',
            description=f'Mission to fly when action=mission: one of {MISSIONS}'),
        DeclareLaunchArgument(
            'params_file', default_value='',
            description='Optional ROS 2 params YAML applied to the node'),
        OpaqueFunction(function=launch_autonomy),
    ])
