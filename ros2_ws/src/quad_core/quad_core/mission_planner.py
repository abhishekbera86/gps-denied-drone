#!/usr/bin/env python3
"""
mission_planner.py — Phase 3 stub
====================================
PLACEHOLDER: Full implementation built in Phase 3.

This node will:
  1. Load waypoints from a YAML file (passed as ROS2 parameter)
  2. Run a state machine: IDLE → ARM → TAKEOFF → WP[0..N] → LAND → DISARM
  3. Switch waypoints when within tolerance radius of current target
  4. Publish mission status on /quad/mission_status

Topics used:
  Uses offboard_controller internally (or calls its services)
  Publish: /quad/mission_status (std_msgs/String — current state)
"""

import rclpy
from rclpy.node import Node


class MissionPlanner(Node):
    """Phase 3 — Waypoint sequencer state machine. STUB."""

    def __init__(self):
        super().__init__('mission_planner')
        self.get_logger().info(
            'MissionPlanner stub loaded. '
            'Full implementation added in Phase 3.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MissionPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
