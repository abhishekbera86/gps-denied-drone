"""hw_vio.launch.py — Phase 4: real OpenVINS VIO pipeline against the
physical D435i, over realsense-ros. UNTESTED — no hardware has run this yet.

The real-hardware sibling of vio.launch.py (sim). Same OpenVINS +
openvins_odometry_bridge pieces, reused completely unmodified — neither
node has any sim-specific logic (openvins_odometry_bridge.py's own
docstring confirms this explicitly). The only real difference is the
sensor source: sim bridges Gazebo's simulated camera via ros_gz_bridge
(world-templated, see vio.launch.py); real hardware runs realsense-ros
directly against the physical D435i — there is no Gazebo topic tree to
template a world name into here, and realsense-ros's own default topic
names (camera_name=camera, camera_namespace=camera -> `/camera/camera/...`)
already match what ros_gz_bridge.yaml targets in sim, and what OpenVINS's
config subscribes to either way, by design (see README's Architecture:
"same camera, same topic names" was the whole point of the sim d435i
model swap).

Starts three pieces, mirroring vio.launch.py's shape:
  1. realsense-ros (`realsense2_camera`'s `rs_launch.py`) — the physical
     D435i's color + combined IMU streams. `unite_imu_method` MUST be set
     (not left at realsense-ros's default of separate /gyro and /accel
     topics) — OpenVINS's config (kalibr_imu_chain_hw.yaml) subscribes to
     ONE combined `/camera/camera/imu` topic, matching what the sim's
     onboard-IMU-per-color-sensor model already produces. VERIFY the exact
     argument names below against your installed realsense-ros version
     (`ros2 launch realsense2_camera rs_launch.py --show-args`) before
     trusting them as-is — realsense-ros's launch API has changed across
     versions and this was authored without a live install to check
     against (see resource/hardware-bringup-vio.md).
  2. OpenVINS (`ov_msckf`'s own `subscribe.launch.py`), pointed at
     `estimator_config_hw.yaml` — the REAL calibration file, not sim's.
     Do not fly this until resource/hardware-bringup-vio.md's calibration
     procedure has filled in kalibr_imucam_chain_hw.yaml/
     kalibr_imu_chain_hw.yaml's placeholder values.
  3. `openvins_odometry_bridge` — identical package/executable as sim,
     zero changes needed.
  4. `vio_output_check` — same fail-loud guard as sim's vio.launch.py
     (README Known Issues #35/#36), watching OpenVINS's own output
     (`/ov_msckf/odomimu`) rather than raw camera/IMU input, for the same
     reason documented there: input presence alone can't distinguish
     "genuinely broken" from "producing data but never initializing."

Included by hw_bringup/launch/hw.launch.py when localization_source:=vision
— there is no vio_backend choice on real hardware (unlike sim's loopback/
openvins choice): vision on real hardware always means the real pipeline,
since there is no fake-VIO stand-in to prove the mechanism with here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    estimator_config = os.path.join(
        get_package_share_directory('common_perception'), 'config', 'openvins',
        'estimator_config_hw.yaml')
    openvins_launch = os.path.join(
        get_package_share_directory('ov_msckf'), 'launch', 'subscribe.launch.py')
    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                'camera_name': 'camera',
                'camera_namespace': 'camera',
                'enable_color': 'true',
                'enable_depth': 'false',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                # Combines /gyro and /accel into one /camera/camera/imu
                # topic at the accel rate, matching kalibr_imu_chain_hw
                # .yaml's single `imu0` source. VERIFY this argument name
                # and its accepted values against your installed
                # realsense-ros -- some versions want a string like
                # "linear_interpolation", others an integer enum.
                'unite_imu_method': '2',
                # 640x480@30 as a starting point (not calibrated/validated)
                # -- whatever you actually calibrate with in
                # resource/hardware-bringup-vio.md, set here to match.
                'rgb_camera.color_profile': '640x480x30',
            }.items(),
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
        ExecuteProcess(
            cmd=['bash', '-c',
                 'timeout 15 ros2 topic echo /ov_msckf/odomimu --once >/dev/null 2>&1 || '
                 'echo "[vio_output_check] ERROR: OpenVINS produced no odometry on '
                 '/ov_msckf/odomimu after 15s -- the position estimate will never '
                 'become valid. Check (in order): realsense-ros actually publishing '
                 '(ros2 topic hz /camera/camera/color/image_raw and '
                 '/camera/camera/imu), the D435i has enough visual texture in view '
                 '(a blank wall/ceiling will not initialize, same as sim'"'"'s empty '
                 'world), and estimator_config_hw.yaml/kalibr_*_hw.yaml are actually '
                 'filled in, not still template placeholders." >&2'],
            name='vio_output_check',
            output='screen',
        ),
    ])
