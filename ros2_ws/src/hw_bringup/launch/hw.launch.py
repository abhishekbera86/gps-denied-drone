"""hw.launch.py — real-hardware bringup (Phase 4 STUB, not yet flown).

Mirror of sim_bringup/sim.launch.py for a real Pixhawk 6 + Orange Pi 5 Plus.
The autonomy include is identical to sim — the only differences live here in
the bringup layer:
  * DDS transport: a uXRCE-DDS agent over SERIAL to the Pixhawk (sim uses UDP,
    started by the ros2-autonomy container entrypoint).
  * Sensor source (Phase 3/4): realsense-ros + common_perception VIO, wired in
    at the placeholder slot below (in sim these come from ros_gz_bridge).
  * params_file: hw_params.yaml instead of sim_params.yaml.
  * mavlink_url: a real MAVLink telemetry endpoint (TBD Phase 4 — untested,
    same status as everything else here) instead of SITL's UDP port.

Like sim_bringup, a ONE-SHOT localization-source switch (`set_localization_source`,
common_perception) runs to completion before the mission starts — see
resource/phase3-gps-denied-localization-source.md.

STATUS: untested — there is no hardware yet. This exists so the sim→real seam
is real and reviewable now. Do not expect it to run end-to-end until Phase 4
(flash PX4 v1.17.0 to the Pixhawk 6, wire the D435i, build ARM64 on the
Orange Pi). See IMPLEMENTATION_PLAN.md Phase 4.

Example (on the real companion computer, once it exists):
  ros2 launch hw_bringup hw.launch.py action:=mission mission:=square \
      serial_device:=/dev/ttyACM0 localization_source:=vision
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _after_localization_source_set(event, context):
    """Start the mission only after the localization-source switch confirms
    — never on failure. (No Milestone-A loopback stand-in on real hardware —
    hw_bringup's vision path is the real common_perception VIO pipeline once
    Phase 4 wires it in; see the placeholder slot below.)
    """
    if event.returncode != 0:
        return None

    autonomy_launch = os.path.join(
        get_package_share_directory('common_missions'), 'launch', 'autonomy.launch.py')
    hw_params = os.path.join(
        get_package_share_directory('hw_bringup'), 'config', 'hw_params.yaml')

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(autonomy_launch),
            launch_arguments={
                'action': LaunchConfiguration('action'),
                'mission': LaunchConfiguration('mission'),
                'params_file': hw_params,
            }.items(),
        ),
    ]


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
        # Pixhawk 6 serial link. Defaults match the .env HW pins; override on
        # the real machine (the Orange Pi typically enumerates it as ttyACM0).
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='921600'),
        DeclareLaunchArgument('action', default_value='mission'),
        DeclareLaunchArgument('mission', default_value='square'),
        DeclareLaunchArgument(
            'localization_source', default_value='gps',
            description="Localization source: 'gps' (outdoor) or 'vision' (indoor VIO)"),
        DeclareLaunchArgument(
            'mavlink_url', default_value='udpin:0.0.0.0:14540',
            description=(
                'pymavlink connection string for the localization-source switch — '
                'UNTESTED default (SITL-shaped, matching sim_bringup); Phase 4 must '
                'set this to a real MAVLink telemetry endpoint on the Pixhawk 6'
            )),

        # uXRCE-DDS agent over serial (the real-HW analog of the sim's UDP
        # agent). This is the ONLY transport difference from sim_bringup.
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev',
                 LaunchConfiguration('serial_device'), '-b', LaunchConfiguration('baud')],
            output='screen',
        ),

        # ── Phase 3/4 sensor + VIO slot (placeholder) ───────────────────────
        # When realsense-ros lands, include it here so it publishes the SAME
        # topic names sim's ros_gz_bridge produces (common_perception's
        # openvins_odometry_bridge + vio.launch.py already assume this):
        #   IncludeLaunchDescription(PythonLaunchDescriptionSource(
        #       os.path.join(get_package_share_directory('common_perception'),
        #                    'launch', 'vio.launch.py')))
        # (only needed when localization_source=vision — sim_bringup shows
        # the OpaqueFunction pattern for making that conditional)

        switch_process,
        RegisterEventHandler(OnProcessExit(
            target_action=switch_process,
            on_exit=_after_localization_source_set,
        )),
    ])
