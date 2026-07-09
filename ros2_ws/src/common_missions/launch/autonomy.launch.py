"""autonomy.launch.py — the shared, transport-agnostic autonomy entry point.

This is the ONE launch file that actually starts the flight/mission nodes.
It is deliberately transport-agnostic: it knows nothing about UDP vs serial,
sim vs real, or where the params come from. The sim_bringup and hw_bringup
packages each include this file, passing their own `params_file` — that is the
entire sim-vs-real difference at the autonomy layer.

Arguments:
  action=hover|mission   what to fly (default: mission)
  mission=square         which mission, when action=mission (default and
                         currently only option: square)
  params_file=<path>     optional ROS 2 params YAML applied to the node
                         (empty = use the nodes' built-in defaults)

Examples:
  ros2 launch common_missions autonomy.launch.py action:=hover
  ros2 launch common_missions autonomy.launch.py action:=mission mission:=square \
      params_file:=/path/to/sim_params.yaml
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MISSIONS = ('square',)


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

    flight_node = Node(
        package=package,
        executable=executable,
        output='screen',
        parameters=parameters,
    )
    return [
        flight_node,
        # The VIO/bridge nodes (openvins, camera_imu_bridge, the odometry
        # bridge — added by sim_bringup/hw_bringup alongside this include)
        # have no exit condition of their own; without this, they keep
        # running indefinitely after the flight node's own state machine
        # finishes (or crashes), continuing to fuse a VIO estimate into PX4
        # with nothing left to sanity-check it against a live mission.
        # Confirmed live (2026-07-09): a leftover OpenVINS instance kept
        # running after a mission's "Landed and disarmed" line, its own
        # position estimate ran away unbounded (100+ m within a couple
        # minutes — see DEVELOPMENT_STATUS.md), and PX4's real EKF2 fused
        # enough of it before disarm to leave `vehicle_local_position`
        # reporting a bogus estimate tens of meters off — which then broke
        # arming for the NEXT flight in the same containers. Shutting the
        # whole launch down the moment the flight node exits, success or
        # failure, closes that window.
        RegisterEventHandler(OnProcessExit(
            target_action=flight_node,
            on_exit=Shutdown(
                reason='flight/mission node exited — shutting down the rest of the launch'),
        )),
    ]


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
