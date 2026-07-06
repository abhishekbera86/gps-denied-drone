#!/usr/bin/env python3
"""
sim.launch.py — Simulation World Launch File
=============================================
Starts the complete GPS-denied simulation stack using Aerostack2's
built-in multirotor simulator (no PX4, no Gazebo, no GPU required).

Nodes launched:
  1. as2_platform_multirotor_simulator — physics simulation engine
  2. as2_state_estimator (ground_truth) — provides perfect state for sim
  3. as2_motion_controller (differential_flatness) — position control
  4. as2_behaviors_motion — Takeoff, Land, GoTo action servers

Usage (inside aerostack2 container — called by make sim):
  ros2 launch quad_sim sim.launch.py
  ros2 launch quad_sim sim.launch.py namespace:=drone0
  ros2 launch quad_sim sim.launch.py namespace:=drone0 log_level:=debug

After this terminal shows "✓ All nodes running", run in a NEW terminal:
  make mission
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
    log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level: debug | info | warn | error",
    )

    ns        = LaunchConfiguration("namespace")
    log_level = LaunchConfiguration("log_level")

    # ── Config paths ──────────────────────────────────────────────────────
    # Try installed path first (after colcon build), fallback to source path
    try:
        quad_sim_share = get_package_share_directory("quad_sim")
        config_dir = os.path.join(quad_sim_share, "config")
    except Exception:
        config_dir = "/ros2_ws/src/quad_sim/config"

    # ── AS2 package share directories ─────────────────────────────────────
    # All packages are pre-built in /root/aerostack2_ws/install/
    sim_platform_dir  = get_package_share_directory("as2_platform_multirotor_simulator")
    estimator_dir     = get_package_share_directory("as2_state_estimator")
    controller_dir    = get_package_share_directory("as2_motion_controller")
    behaviors_dir     = get_package_share_directory("as2_behaviors_motion")

    # ── Banner ────────────────────────────────────────────────────────────
    banner = LogInfo(msg=[
        "\n",
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n",
        "  GPS-Denied Drone — Simulation World\n",
        "  Platform : as2_platform_multirotor_simulator\n",
        "  Namespace: ", ns, "\n",
        "  No PX4 · No Gazebo · No GPU required\n",
        "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n",
    ])

    # ── 1. Multirotor Simulator Platform ──────────────────────────────────
    # AS2's built-in physics simulation. Provides the same ROS 2 topic
    # interface as as2_platform_pixhawk — mission.py is unchanged.
    # Config: uav dynamics (mass, inertia, motor params), sim frequency
    platform_config = os.path.join(
        sim_platform_dir, "config", "platform_config_file.yaml"
    )
    uav_config = os.path.join(
        sim_platform_dir, "config", "uav_config.yaml"
    )

    platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_platform_dir, "launch",
                         "as2_platform_multirotor_simulator.launch.py")
        ),
        launch_arguments={
            "namespace":           ns,
            "use_sim_time":        "false",
            "platform_config_file": platform_config,
            "uav_config":          uav_config,
            "log_level":           log_level,
        }.items(),
    )

    # ── 2. State Estimator: Ground Truth ─────────────────────────────────
    # Uses simulator ground truth for perfect state in simulation.
    # (In real hardware, this is replaced by raw_odometry + OpenVINS VIO)
    estimator_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(estimator_dir, "launch",
                                 "ground_truth_state_estimator.launch.py")
                ),
                launch_arguments={
                    "namespace":    ns,
                    "use_sim_time": "false",
                    "log_level":    log_level,
                }.items(),
            )
        ],
    )

    # ── 3. Motion Controller: Differential Flatness ───────────────────────
    # Converts position/velocity goals → motor thrust commands.
    controller_launch = TimerAction(
        period=4.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(controller_dir, "launch",
                                 "differential_flatness_controller.launch.py")
                ),
                launch_arguments={
                    "namespace":    ns,
                    "use_sim_time": "false",
                    "log_level":    log_level,
                }.items(),
            )
        ],
    )

    # ── 4. Motion Behaviours ─────────────────────────────────────────────
    # ROS 2 action servers: Takeoff, Land, GoTo, FollowPath
    # These are called by mission.py via the AS2 Python API.
    behaviors_launch = TimerAction(
        period=6.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(behaviors_dir, "launch",
                                 "composable_motion_behaviors.launch.py")
                ),
                launch_arguments={
                    "namespace":    ns,
                    "use_sim_time": "false",
                    "log_level":    log_level,
                }.items(),
            )
        ],
    )

    # ── Ready message ─────────────────────────────────────────────────────
    ready_msg = TimerAction(
        period=8.0,
        actions=[
            LogInfo(msg=[
                "\n",
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n",
                "  ✓  All Aerostack2 nodes are running\n",
                "  Open a NEW terminal and run:\n",
                "       make mission\n",
                "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n",
            ])
        ],
    )

    return LaunchDescription([
        namespace_arg,
        log_level_arg,
        banner,
        platform_launch,
        estimator_launch,
        controller_launch,
        behaviors_launch,
        ready_msg,
    ])
