#!/usr/bin/env python3
"""
hw.launch.py — Hardware Stack Launch File (Orange Pi 5 + Pixhawk + RealSense)
==============================================================================
Starts the complete real-hardware autonomy stack:

  1. RealSense D435i camera node   — IR images + IMU @ 400 Hz
  2. as2_platform_pixhawk          — bridges AS2 ↔ Pixhawk via serial UART
  3. as2_state_estimator           — ingests VIO odometry
  4. as2_motion_controller         — converts behaviour goals → Pixhawk setpoints
  5. as2_behaviors_motion          — Takeoff, Land, GoTo, FollowPath action servers

NOTE: OpenVINS is launched separately (make mission or as a second launch).
      The RealSense node must be running and publishing before OpenVINS starts.

Usage (inside hw_stack container on Orange Pi 5):
  ros2 launch quad_real hw.launch.py
  ros2 launch quad_real hw.launch.py serial_device:=/dev/ttyUSB1
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    LogInfo,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="drone0",
        description="ROS 2 namespace for this drone",
    )
    serial_device_arg = DeclareLaunchArgument(
        "serial_device",
        default_value="/dev/ttyUSB0",
        description="Serial port connecting Orange Pi to Pixhawk TELEM2",
    )
    serial_baud_arg = DeclareLaunchArgument(
        "serial_baud",
        default_value="921600",
        description="Baud rate (must match PX4 SER_TEL2_BAUD parameter)",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level: debug | info | warn | error",
    )

    ns            = LaunchConfiguration("namespace")
    serial_device = LaunchConfiguration("serial_device")
    serial_baud   = LaunchConfiguration("serial_baud")
    log_level     = LaunchConfiguration("log_level")

    # ── Config paths ──────────────────────────────────────────────────────
    try:
        quad_real_share = get_package_share_directory("quad_real")
        config_dir = os.path.join(quad_real_share, "config")
    except Exception:
        config_dir = "/ros2_ws/src/quad_real/config"

    try:
        quad_core_share = get_package_share_directory("quad_core")
        core_config_dir = os.path.join(quad_core_share, "config")
    except Exception:
        core_config_dir = "/ros2_ws/src/quad_core/config"

    platform_config  = os.path.join(config_dir, "as2_platform_hw.yaml")
    estimator_config = os.path.join(config_dir, "state_estimator_hw.yaml")
    realsense_config = os.path.join(config_dir, "realsense_hw.yaml")

    # ── Package share directories ─────────────────────────────────────────
    realsense_dir    = get_package_share_directory("realsense2_camera")
    as2_platform_dir = get_package_share_directory("as2_platform_pixhawk")
    as2_estimator_dir = get_package_share_directory("as2_state_estimator")
    as2_controller_dir = get_package_share_directory("as2_motion_controller")
    as2_behaviors_dir  = get_package_share_directory("as2_behaviors_motion")

    # ── Banner ────────────────────────────────────────────────────────────
    banner = LogInfo(msg=[
        "\n",
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n",
        "  GPS-Denied Drone — HARDWARE Stack\n",
        "  Namespace  : ", ns, "\n",
        "  Pixhawk    : ", serial_device, " @ ", serial_baud, " baud\n",
        "  RealSense  : firmware 5.17.3.10 | SDK 2.58.2\n",
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n",
    ])

    # ── 1. RealSense D435i Camera Node ────────────────────────────────────
    # Publishes:  /camera/infra1/image_rect_raw  (IR, 640x480 @ 30fps)
    #             /camera/imu                    (fused IMU @ 400Hz)
    #             /camera/depth/image_rect_raw   (depth, for future use)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, "launch", "rs_launch.py")
        ),
        launch_arguments={
            "config_file":        realsense_config,
            "camera_name":        "camera",
            "enable_infra1":      "true",
            "enable_depth":       "true",
            "enable_color":       "false",
            "unite_imu_method":   "linear_interpolation",
            "log_level":          log_level,
        }.items(),
    )

    # ── 2. AS2 Platform: serial bridge to Pixhawk ─────────────────────────
    # The key difference from sim: connection_type=serial
    platform_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(as2_platform_dir, "launch", "pixhawk_launch.py")
                ),
                launch_arguments={
                    "namespace":       ns,
                    "use_sim_time":    "false",
                    "connection_type": "serial",
                    "serial_device":   serial_device,
                    "serial_baudrate": serial_baud,
                    "config":          platform_config,
                    "log_level":       log_level,
                }.items(),
            )
        ],
    )

    # ── 3. State Estimator ────────────────────────────────────────────────
    estimator_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(as2_estimator_dir, "launch", "state_estimator_launch.py")
                ),
                launch_arguments={
                    "namespace":    ns,
                    "use_sim_time": "false",
                    "plugin":       "raw_odometry",
                    "config":       estimator_config,
                    "log_level":    log_level,
                }.items(),
            )
        ],
    )

    # ── 4. Motion Controller ─────────────────────────────────────────────
    controller_launch = TimerAction(
        period=8.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(as2_controller_dir, "launch", "controller_launch.py")
                ),
                launch_arguments={
                    "namespace":    ns,
                    "use_sim_time": "false",
                    "plugin":       "differential_flatness_controller",
                    "log_level":    log_level,
                }.items(),
            )
        ],
    )

    # ── 5. Motion Behaviours ─────────────────────────────────────────────
    behaviors_launch = TimerAction(
        period=10.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(as2_behaviors_dir, "launch", "motion_behaviors_launch.py")
                ),
                launch_arguments={
                    "namespace":    ns,
                    "use_sim_time": "false",
                    "log_level":    log_level,
                }.items(),
            )
        ],
    )

    # ── Pre-flight checklist reminder ────────────────────────────────────
    preflight_msg = TimerAction(
        period=12.0,
        actions=[
            LogInfo(msg=[
                "\n",
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n",
                "  ✓  Hardware stack is running\n",
                "\n",
                "  PRE-FLIGHT CHECKLIST:\n",
                "    □ Remove all propellers for first test\n",
                "    □ RealSense topics visible (run: make shell → ros2 topic list | grep camera)\n",
                "    □ EKF2 healthy in QGroundControl\n",
                "    □ Upload EKF2 params if not done: make params-upload\n",
                "\n",
                "  Then in a new terminal:  make mission\n",
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n",
            ])
        ],
    )

    return LaunchDescription([
        namespace_arg,
        serial_device_arg,
        serial_baud_arg,
        log_level_arg,
        banner,
        realsense_launch,
        platform_launch,
        estimator_launch,
        controller_launch,
        behaviors_launch,
        preflight_msg,
    ])
