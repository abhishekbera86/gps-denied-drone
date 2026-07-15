"""hw.launch.py — real-hardware bringup (Phase 4 — UNTESTED, no hardware has
run this yet. Follow resource/hardware-bringup-gps.md and
resource/hardware-bringup-vio.md, in that order, before trusting anything
here on a free-flying vehicle).

Mirror of sim_bringup/sim.launch.py for a real Pixhawk 6C + Orange Pi 5 Plus.
The autonomy include is identical to sim — the only differences live here in
the bringup layer:
  * DDS transport: a uXRCE-DDS agent over SERIAL to the Pixhawk (sim uses
    UDP). This launch file does NOT start that agent itself — unlike an
    earlier version, which tied its lifecycle to this file (a real mistake,
    found live: it meant `/fmu/*` topics weren't verifiable during dry-run
    bench testing without starting an agent by hand first). It's now
    auto-started at `hw-autonomy` container BOOT instead, by
    `entrypoint_hw_autonomy.sh` — the exact same pattern sim's
    `ros2-autonomy` already uses for its own UDP agent. By the time this
    launch file runs, the bridge should already be up; see resource/
    hardware-bringup-gps.md's wiring/dry-run sections.
  * Sensor source (Phase 3/4): when localization_source:=vision, includes
    common_perception/launch/hw_vio.launch.py — realsense-ros + OpenVINS
    against the real D435i (in sim this is ros_gz_bridge + OpenVINS against
    Gazebo instead). No vio_backend choice here, unlike sim: real hardware
    has no fake-VIO loopback stand-in to choose instead, so vision always
    means the real pipeline.
  * params_file: hw_params.yaml instead of sim_params.yaml.

EKF2_GPS_CTRL/EKF2_EV_CTRL (and, for vision, the lever arm/noise-floor
params): DEFAULT is to assume you've already set these once via
QGroundControl (resource/hardware-bringup-gps.md's "Configure
localization source manually" section) and skip touching them here at
all — `use_mavlink_switch` defaults to `false`. Unlike sim (which always
has GPS to fall back to and no reason not to automate this), real
hardware only strictly needs a SECOND physical link
(`set_localization_source`'s MAVLink connection) if you want this launch
file to change PX4's fusion source for you at every startup — most users
flying mostly one mode should just configure it once by hand and skip
that connection entirely. Set `use_mavlink_switch:=true` (and wire up
Link 2 — see the guide) to restore the fully-automated sim-equivalent
behavior instead.

Example (on the real companion computer, single-connection default):
  ros2 launch hw_bringup hw.launch.py action:=mission mission:=square \
      localization_source:=vision

Example (opting into the automated MAVLink switch, needs Link 2 wired):
  ros2 launch hw_bringup hw.launch.py action:=mission mission:=square \
      localization_source:=vision \
      use_mavlink_switch:=true mavlink_url:=/dev/ttyUSB1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _autonomy_and_vio_actions(context):
    """The actions every path needs eventually: the mission/hover node,
    plus the real VIO pipeline if flying vision mode. Shared by both the
    default (no MAVLink switch) path and the OnProcessExit callback for
    the opt-in automated-switch path, so the two paths can't drift apart.
    """
    autonomy_launch = os.path.join(
        get_package_share_directory('common_missions'), 'launch', 'autonomy.launch.py')
    hw_params = os.path.join(
        get_package_share_directory('hw_bringup'), 'config', 'hw_params.yaml')

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(autonomy_launch),
            launch_arguments={
                'action': LaunchConfiguration('action'),
                'mission': LaunchConfiguration('mission'),
                'params_file': hw_params,
            }.items(),
        ),
    ]

    if LaunchConfiguration('localization_source').perform(context) == 'vision':
        hw_vio_launch = os.path.join(
            get_package_share_directory('common_perception'), 'launch', 'hw_vio.launch.py')
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(hw_vio_launch)))

    return actions


def _after_localization_source_set(event, context):
    """use_mavlink_switch:=true path only. Start the mission (and VIO)
    only after the localization-source switch confirms — never on
    failure.
    """
    if event.returncode != 0:
        return None
    return _autonomy_and_vio_actions(context)


def generate_launch_description():
    # console_scripts live under install/<pkg>/lib/<pkg>/ (the ros2-run
    # convention every package in this repo uses), not on raw $PATH — so
    # ExecuteProcess (a plain subprocess spawn) must go through `ros2 run`,
    # unlike ros2 launch Node actions which resolve this via the ament index.
    use_mavlink_switch = LaunchConfiguration('use_mavlink_switch')

    # Only actually runs when use_mavlink_switch:=true (IfCondition below)
    # — the default path never touches this or needs Link 2 wired at all.
    switch_process = ExecuteProcess(
        cmd=['ros2', 'run', 'common_perception', 'set_localization_source',
             '--source', LaunchConfiguration('localization_source'),
             '--mavlink-url', LaunchConfiguration('mavlink_url'),
             # Harmless in gps mode (set_localization_source only applies
             # these when --source vision — see that module). MUST be
             # overridden with your real measured mount offset before any
             # vision-mode flight — see resource/hardware-bringup-vio.md.
             # The 0.0 defaults below are NOT the sim's co-located value;
             # they're a deliberately-obvious "not yet measured" sentinel
             # so an un-overridden vision flight is wrong in an easy-to-
             # notice way (zero lever arm) rather than silently reusing a
             # different airframe's value.
             '--ev-pos-x', LaunchConfiguration('ev_pos_x'),
             '--ev-pos-y', LaunchConfiguration('ev_pos_y'),
             '--ev-pos-z', LaunchConfiguration('ev_pos_z')],
        output='screen',
        condition=IfCondition(use_mavlink_switch),
    )

    return LaunchDescription([
        # No serial_device/baud launch arguments here anymore — the ONE
        # connection this setup needs (the DDS agent) is now started at
        # container boot by entrypoint_hw_autonomy.sh, reading
        # PIXHAWK_SERIAL_PORT/PIXHAWK_BAUD_RATE directly from the
        # container's own environment (set from .env via docker-compose.yml,
        # per resource/hardware-bringup-gps.md's wiring/dry-run sections) —
        # not from launch arguments, since the agent is already running
        # before this launch file ever starts.
        DeclareLaunchArgument('action', default_value='mission'),
        DeclareLaunchArgument('mission', default_value='square'),
        DeclareLaunchArgument(
            'localization_source', default_value='gps',
            description=(
                "Localization source: 'gps' (outdoor) or 'vision' (indoor VIO). "
                "With use_mavlink_switch:=false (the default), this must already "
                "match what you configured via QGroundControl — see "
                "resource/hardware-bringup-gps.md."
            )),
        DeclareLaunchArgument(
            'use_mavlink_switch', default_value='false',
            description=(
                "false (default): assume EKF2_GPS_CTRL/EKF2_EV_CTRL were already "
                "set once via QGroundControl -- single connection (Link 1/USB "
                "only), nothing here touches PX4 params. true: run the automated "
                "MAVLink-based switch every launch instead (sim-equivalent "
                "behavior) -- needs Link 2 wired, see the guide."
            )),
        DeclareLaunchArgument(
            'mavlink_url', default_value='udpin:0.0.0.0:14540',
            description=(
                'pymavlink connection string for the localization-source switch — '
                'only used when use_mavlink_switch:=true. UNTESTED default '
                '(SITL-shaped, matching sim_bringup) — '
                'see resource/hardware-bringup-gps.md for how to set this correctly'
            )),
        DeclareLaunchArgument('ev_pos_x', default_value='0.0'),
        DeclareLaunchArgument('ev_pos_y', default_value='0.0'),
        DeclareLaunchArgument('ev_pos_z', default_value='0.0'),

        switch_process,
        RegisterEventHandler(
            OnProcessExit(
                target_action=switch_process,
                on_exit=_after_localization_source_set,
            ),
            condition=IfCondition(use_mavlink_switch),
        ),
        # Default path: go straight to the mission/VIO, no MAVLink switch.
        OpaqueFunction(
            function=_autonomy_and_vio_actions,
            condition=UnlessCondition(use_mavlink_switch),
        ),
    ])
