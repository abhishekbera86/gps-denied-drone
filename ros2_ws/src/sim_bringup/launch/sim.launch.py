"""sim.launch.py — bring up the autonomy stack against PX4 SITL (simulation).

The single sim-side entry point. It layers the simulation params
(config/sim_params.yaml) onto the shared, transport-agnostic
common_missions/autonomy.launch.py. The PX4 SITL + Gazebo world and the
uXRCE-DDS UDP agent are already running (the px4-sim / ros2-autonomy
containers) — this only starts the flight/mission node.

Before the mission node starts, a ONE-SHOT localization-source switch runs
to completion (`set_localization_source`, common_perception) — it sets
PX4's EKF2 fusion source (GPS or vision) over a MAVLink side-channel, since
the uXRCE-DDS bridge this project otherwise uses everywhere cannot set PX4
parameters at all in this pinned PX4 version. See
resource/phase3-gps-denied-localization-source.md for the full rationale.
This is a launch-time CHOICE, not a live in-flight switch.

The hw_bringup package is the mirror of this file for real hardware: same
autonomy include, different params, plus a serial DDS agent. Nothing about
the flight logic differs between them.

When localization_source:=vision, `vio_backend` picks which VIO actually
feeds the switch: `loopback` (default — Milestone A's zero-drift fake-VIO
stand-in, proves the mechanism with no camera needed) or `openvins`
(Milestone B's real VIO pipeline, common_perception/launch/vio.launch.py).

Examples:
  ros2 launch sim_bringup sim.launch.py action:=hover
  ros2 launch sim_bringup sim.launch.py action:=mission mission:=square \
      localization_source:=vision
  ros2 launch sim_bringup sim.launch.py action:=mission mission:=square \
      localization_source:=vision vio_backend:=openvins
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _autonomy_include():
    autonomy_launch = os.path.join(
        get_package_share_directory('common_missions'), 'launch', 'autonomy.launch.py')
    sim_params = os.path.join(
        get_package_share_directory('sim_bringup'), 'config', 'sim_params.yaml')
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(autonomy_launch),
        launch_arguments={
            'action': LaunchConfiguration('action'),
            'mission': LaunchConfiguration('mission'),
            'params_file': sim_params,
        }.items(),
    )


def _after_localization_source_set(event, context):
    """Start the mission only after the localization-source switch confirms
    — never on failure.

    GPS: the mission starts immediately (GPS is fused from PX4 boot).

    Vision: the VIO pipeline and the mission both start now. There is
    deliberately NO "wait for vision data before arming" gate here, and one
    must not be re-added in this shape: it was built and then REMOVED the
    same day (2026-07-10) after being confirmed live as a structural
    DEADLOCK — this OpenVINS version publishes NOTHING while the vehicle is
    parked (its `initialized()` requires a real feature update, but every
    parked frame takes the pre-takeoff ZUPT early-return, so all its
    publishers stay gated until first physical motion; verified in
    VioManager.cpp/VioManager.h and live on multiple boots). So: gate
    waits for vision -> vision waits for motion -> motion waits for arming
    -> arming waits for gate. Vision data instead appears within ~a second
    of first takeoff motion, which is how every working openvins flight in
    this project has actually flown. The planned (not yet built) safety
    net for "vision never came up at all" — a real risk, confirmed live
    once via a world-texture bug: armed and wandered 125 s with no
    localization until the geofence caught it — is an IN-FLIGHT watchdog
    in common_control (abort to LAND if no vision within seconds of
    arming, or if it stops mid-flight), not a pre-arm gate. See
    common_perception/wait_for_vision.py (kept for that future use) and
    README §12 issue 30.

    Failure of the switch returns Shutdown(), never None: returning None
    leaves an already-started VIO pipeline (which has no exit condition of
    its own) running in the launch FOREVER — confirmed live (2026-07-10):
    an orphaned odometry bridge from exactly that path contaminated the
    next diagnostic session in the same container — the part-9 leftover-
    process failure class, reachable through launch-sequencing this time.
    """
    if event.returncode != 0:
        return [Shutdown(
            reason='localization-source switch failed — shutting down (see log above)')]

    if LaunchConfiguration('localization_source').perform(context) != 'vision':
        return [_autonomy_include()]

    vio_backend = LaunchConfiguration('vio_backend').perform(context)
    if vio_backend == 'openvins':
        # Milestone B: real VIO — camera/IMU bridge + OpenVINS + the
        # real odometry bridge, all in one included launch file.
        vio_launch = os.path.join(
            get_package_share_directory('common_perception'), 'launch', 'vio.launch.py')
        vio_actions = [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vio_launch))]
    else:
        # Milestone A (default): no camera/OpenVINS needed — loop PX4's
        # own estimate back in as a stand-in, proving the whole
        # switch+EKF2+bridge mechanism works before real VIO is added.
        vio_actions = [Node(
            package='common_perception',
            executable='loopback_odometry_bridge',
            output='screen',
        )]

    return vio_actions + [_autonomy_include()]


def generate_launch_description():
    # console_scripts live under install/<pkg>/lib/<pkg>/ (the ros2-run
    # convention every package in this repo uses), not on raw $PATH — so
    # ExecuteProcess (a plain subprocess spawn) must go through `ros2 run`,
    # unlike ros2 launch Node actions which resolve this via the ament index.
    switch_process = ExecuteProcess(
        cmd=['ros2', 'run', 'common_perception', 'set_localization_source',
             '--source', LaunchConfiguration('localization_source'),
             '--mavlink-url', LaunchConfiguration('mavlink_url')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'action', default_value='mission',
            description="What to fly: 'hover' or 'mission'"),
        DeclareLaunchArgument(
            'mission', default_value='square',
            description='Mission to fly when action=mission (currently only square)'),
        DeclareLaunchArgument(
            'localization_source', default_value='gps',
            description="Localization source: 'gps' (outdoor) or 'vision' (indoor VIO)"),
        DeclareLaunchArgument(
            'vio_backend', default_value='loopback',
            description="When localization_source=vision, which VIO feeds it: "
                        "'loopback' (Milestone A fake-VIO stand-in) or "
                        "'openvins' (Milestone B real VIO)"),
        DeclareLaunchArgument(
            'mavlink_url', default_value='udpin:0.0.0.0:14540',
            description=(
                'pymavlink connection string for the localization-source switch '
                '(PX4 SITL onboard MAVLink port, independent of the uXRCE-DDS agent '
                '— must be udpin: since PX4 sends here unsolicited)'
            )),
        switch_process,
        RegisterEventHandler(OnProcessExit(
            target_action=switch_process,
            on_exit=_after_localization_source_set,
        )),
    ])
