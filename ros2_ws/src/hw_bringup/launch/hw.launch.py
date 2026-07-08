"""hw.launch.py — real-hardware bringup (Phase 4 STUB, not yet flown).

Mirror of sim_bringup/sim.launch.py for a real Pixhawk 6 + Orange Pi 5 Plus.
The autonomy include is identical to sim — the only differences live here in
the bringup layer:
  * DDS transport: a uXRCE-DDS agent over SERIAL to the Pixhawk (sim uses UDP,
    started by the ros2-autonomy container entrypoint).
  * Sensor source (Phase 3/4): realsense-ros + common_perception VIO, wired in
    at the placeholder slot below (in sim these come from ros_gz_bridge).
  * params_file: hw_params.yaml instead of sim_params.yaml.

STATUS: untested — there is no hardware yet. This exists so the sim→real seam
is real and reviewable now. Do not expect it to run end-to-end until Phase 4
(flash PX4 v1.17.0 to the Pixhawk 6, wire the D435i, build ARM64 on the
Orange Pi). See IMPLEMENTATION_PLAN.md Phase 4.

Example (on the real companion computer, once it exists):
  ros2 launch hw_bringup hw.launch.py action:=mission mission:=square \
      serial_device:=/dev/ttyACM0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    autonomy_launch = os.path.join(
        get_package_share_directory('common_missions'), 'launch', 'autonomy.launch.py')
    hw_params = os.path.join(
        get_package_share_directory('hw_bringup'), 'config', 'hw_params.yaml')

    return LaunchDescription([
        # Pixhawk 6 serial link. Defaults match the .env HW pins; override on
        # the real machine (the Orange Pi typically enumerates it as ttyACM0).
        DeclareLaunchArgument('serial_device', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud', default_value='921600'),
        DeclareLaunchArgument('action', default_value='mission'),
        DeclareLaunchArgument('mission', default_value='square'),

        # uXRCE-DDS agent over serial (the real-HW analog of the sim's UDP
        # agent). This is the ONLY transport difference from sim_bringup.
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'serial', '--dev',
                 LaunchConfiguration('serial_device'), '-b', LaunchConfiguration('baud')],
            output='screen',
        ),

        # ── Phase 3/4 sensor + VIO slot (placeholder) ───────────────────────
        # When common_perception / realsense-ros land, include them here so
        # they publish the SAME topic names the sim's ros_gz_bridge produces
        # (sensor_msgs/Image, CameraInfo, Imu) and VIO feeds PX4 EKF2:
        #   IncludeLaunchDescription(PythonLaunchDescriptionSource(
        #       os.path.join(get_package_share_directory('common_perception'),
        #                    'launch', 'vio.launch.py')))

        # Same shared autonomy as sim — different params only.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(autonomy_launch),
            launch_arguments={
                'action': LaunchConfiguration('action'),
                'mission': LaunchConfiguration('mission'),
                'params_file': hw_params,
            }.items(),
        ),
    ])
