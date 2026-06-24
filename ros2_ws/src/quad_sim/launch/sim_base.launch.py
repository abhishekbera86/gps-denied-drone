#!/usr/bin/env python3
"""
sim_base.launch.py — Phase 1 stub
====================================
PLACEHOLDER: Expanded in Phase 1.

This launch file will:
  - Verify the uXRCE-DDS agent is running (already started by container)
  - Set environment variables for simulation
  - Provide the base for all other sim launch files to include

Usage (Phase 1+):
  ros2 launch quad_sim sim_base.launch.py
"""

from launch import LaunchDescription
from launch.actions import LogInfo


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='[quad_sim] sim_base.launch.py loaded. '
                    'Phase 1 implementation added next.'),
    ])
