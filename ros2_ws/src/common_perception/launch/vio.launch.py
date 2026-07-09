"""vio.launch.py — Milestone B: real OpenVINS VIO pipeline, sim side.

Starts three pieces, in order they're declared (no strict sequencing needed
— each is independent until data actually needs to flow between them):
  1. `ros_gz_bridge` (parameter_bridge) — Gazebo camera/IMU -> ROS 2,
     remapped to the exact topic names realsense-ros uses for a real D435i
     (config/ros_gz_bridge.yaml).
  2. OpenVINS (`ov_msckf`'s own `subscribe.launch.py`) — the VIO estimator
     itself, pointed at our derived (not guessed) calibration
     (config/openvins/estimator_config.yaml). `use_stereo`/`max_cameras`
     are passed explicitly here too, matching the YAML's mono settings —
     OpenVINS's launch file also accepts these as separate ROS params, and
     relying on undocumented precedence between the two paths isn't worth
     the risk when they can just agree.
  3. `openvins_odometry_bridge` — republishes OpenVINS's output as
     `/fmu/in/vehicle_visual_odometry`, the same topic the Milestone A
     `loopback_odometry_bridge` uses (drop-in replacement).

Included by `sim_bringup/launch/sim.launch.py` when `localization_source:=vision`
and Milestone B is selected — see
resource/phase3-gps-denied-localization-source.md.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bridge_config = os.path.join(
        get_package_share_directory('common_perception'), 'config', 'ros_gz_bridge.yaml')
    estimator_config = os.path.join(
        get_package_share_directory('common_perception'), 'config', 'openvins',
        'estimator_config.yaml')
    openvins_launch = os.path.join(
        get_package_share_directory('ov_msckf'), 'launch', 'subscribe.launch.py')

    return LaunchDescription([
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='camera_imu_bridge',
            output='screen',
            parameters=[{'config_file': bridge_config}],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(openvins_launch),
            launch_arguments={
                'config_path': estimator_config,
                'use_stereo': 'false',
                'max_cameras': '1',
                'rviz_enable': 'false',
            }.items(),
        ),
        Node(
            package='common_perception',
            executable='openvins_odometry_bridge',
            output='screen',
        ),
    ])
