#!/usr/bin/env python3
"""
sim.launch.py — Simulation World Launch File
=============================================
Single launch file that starts the complete GPS-denied simulation stack:

  1. as2_platform_pixhawk  — bridges AS2 ↔ PX4 SITL via UDP
  2. as2_state_estimator   — ingests VIO odometry (raw_odometry plugin)
  3. as2_motion_controller — converts behaviour goals → PX4 setpoints
  4. as2_behaviors_motion  — action servers: Takeoff, Land, GoTo, FollowPath

Usage (inside aerostack2 container):
  ros2 launch quad_sim sim.launch.py
  ros2 launch quad_sim sim.launch.py namespace:=drone1
  ros2 launch quad_sim sim.launch.py namespace:=drone0 log_level:=debug

This launch file is used by:
  make sim  →  ros2 launch quad_sim sim.launch.py  (inside container)
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
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():

    # ── Launch arguments (overridable from command line) ───────────────────
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="drone0",
        description="ROS 2 namespace for this drone",
    )
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level: debug | info | warn | error",
    )

    ns = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")

    # ── Config file paths (mounted from host at /ros2_ws) ─────────────────
    # These files live in quad_sim/config/ and are mounted read-only.
    # We use the installed share path (after colcon build) OR the direct
    # source path if the package is not installed yet.
    try:
        quad_sim_share = get_package_share_directory("quad_sim")
        config_dir = os.path.join(quad_sim_share, "config")
    except Exception:
        # Fallback: package not installed — use source path directly
        config_dir = "/ros2_ws/src/quad_sim/config"

    platform_config = os.path.join(config_dir, "as2_platform_sim.yaml")
    estimator_config = os.path.join(config_dir, "state_estimator_sim.yaml")
    controller_config = os.path.join(config_dir, "motion_controller_sim.yaml")

    # ── Aerostack2 package share directories ──────────────────────────────
    # All AS2 packages are installed via apt — their launch files are at
    # /opt/ros/humble/share/<package>/launch/
    as2_platform_dir    = get_package_share_directory("as2_platform_pixhawk")
    as2_estimator_dir   = get_package_share_directory("as2_state_estimator")
    as2_controller_dir  = get_package_share_directory("as2_motion_controller")
    as2_behaviors_dir   = get_package_share_directory("as2_behaviors_motion")

    # ── Banner ────────────────────────────────────────────────────────────
    banner = LogInfo(msg=[
        "\n",
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n",
        "  GPS-Denied Drone — Simulation World\n",
        "  Namespace  : ", ns, "\n",
        "  Config dir : ", config_dir, "\n",
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n",
    ])

    # ── 1. Platform: AS2 ↔ PX4 SITL bridge (UDP) ─────────────────────────
    # Connects to PX4 SITL via MicroXRCE-DDS Agent on localhost:8888.
    # Translates AS2 motion setpoints → PX4 offboard control messages.
    # Also forwards visual odometry ENU → NED for PX4 EKF2.
    platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(as2_platform_dir, "launch", "pixhawk_launch.py")
        ),
        launch_arguments={
            "namespace":       ns,
            "use_sim_time":    "false",
            "connection_type": "udp",
            "udp_ip":          "127.0.0.1",
            "udp_port":        "8888",
            "config":          platform_config,
            "log_level":       log_level,
        }.items(),
    )

    # ── 2. State Estimator: VIO → AS2 state ──────────────────────────────
    # Plugin: raw_odometry
    # Subscribes to:  /openvins/odometry  (nav_msgs/Odometry, ENU frame)
    # Publishes to:   /<namespace>/self_localization/pose
    #                 /<namespace>/self_localization/twist
    # Delayed 3s so the platform node is ready first.
    estimator_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(as2_estimator_dir, "launch", "state_estimator_launch.py")
                ),
                launch_arguments={
                    "namespace":   ns,
                    "use_sim_time": "false",
                    "plugin":       "raw_odometry",
                    "config":       estimator_config,
                    "log_level":    log_level,
                }.items(),
            )
        ],
    )

    # ── 3. Motion Controller: goals → PX4 setpoints ──────────────────────
    # Plugin: differential_flatness_controller
    # Converts position/velocity/acceleration goals from behaviours
    # into attitude + thrust setpoints for PX4.
    # Delayed 5s so estimator publishes state first.
    controller_launch = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(as2_controller_dir, "launch", "controller_launch.py")
                ),
                launch_arguments={
                    "namespace":    ns,
                    "use_sim_time": "false",
                    "plugin":       "differential_flatness_controller",
                    "config":       controller_config,
                    "log_level":    log_level,
                }.items(),
            )
        ],
    )

    # ── 4. Motion Behaviours: Takeoff, Land, GoTo, FollowPath ────────────
    # ROS 2 action servers that mission.py calls via the AS2 Python API.
    # Delayed 7s so the controller is ready first.
    behaviors_launch = TimerAction(
        period=7.0,
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

    # ── Ready message ──────────────────────────────────────────────────────
    ready_msg = TimerAction(
        period=10.0,
        actions=[
            LogInfo(msg=[
                "\n",
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n",
                "  ✓  All Aerostack2 nodes are running\n",
                "  Open a NEW terminal and run:\n",
                "     make mission\n",
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n",
            ])
        ],
    )

    return LaunchDescription([
        # Args
        namespace_arg,
        log_level_arg,
        # Banner
        banner,
        # Nodes (staggered startup)
        platform_launch,
        estimator_launch,
        controller_launch,
        behaviors_launch,
        ready_msg,
    ])
