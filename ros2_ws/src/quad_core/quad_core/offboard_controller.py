#!/usr/bin/env python3
"""
offboard_controller.py — Phase 2 stub
======================================
PLACEHOLDER: Full implementation built in Phase 2.

This node will:
  1. Arm the drone via VehicleCommand
  2. Switch PX4 to OFFBOARD mode
  3. Publish TrajectorySetpoint to fly to a target altitude
  4. Hold hover until shutdown
  5. Land cleanly on Ctrl-C

Topics used:
  Publish:  /fmu/in/offboard_control_mode   (10Hz keepalive)
  Publish:  /fmu/in/trajectory_setpoint     (position setpoint)
  Publish:  /fmu/in/vehicle_command         (arm / mode change)
  Subscribe: /fmu/out/vehicle_status        (arming state)
  Subscribe: /fmu/out/vehicle_local_position (current position)
"""

import rclpy
from rclpy.node import Node


class OffboardController(Node):
    """Phase 2 — Arm, takeoff to 5m, hover. STUB — not yet implemented."""

    def __init__(self):
        super().__init__('offboard_controller')
        self.get_logger().info(
            'OffboardController stub loaded. '
            'Full implementation added in Phase 2.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = OffboardController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
