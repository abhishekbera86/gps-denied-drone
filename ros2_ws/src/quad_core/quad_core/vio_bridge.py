#!/usr/bin/env python3
"""
vio_bridge.py — Phase 4 stub
===============================
PLACEHOLDER: Full implementation built in Phase 4.

This node bridges OpenVINS output to PX4 EKF2 input.

The challenge: OpenVINS outputs in ENU (East-North-Up) frame,
but PX4 EKF2 expects NED (North-East-Down) frame.

This node:
  1. Subscribes to OpenVINS odometry (nav_msgs/Odometry, ENU)
  2. Converts position and orientation ENU → NED
  3. Converts velocity ENU → NED
  4. Maps covariance matrix
  5. Publishes VehicleVisualOdometry to PX4 via DDS

Topics:
  Subscribe: /openvins/odometry           (nav_msgs/Odometry, ENU)
  Publish:   /fmu/in/vehicle_visual_odometry (px4_msgs/VehicleVisualOdometry, NED)

ENU → NED conversion:
  x_ned =  y_enu
  y_ned =  x_enu
  z_ned = -z_enu
"""

import rclpy
from rclpy.node import Node


class VioBridge(Node):
    """Phase 4 — OpenVINS (ENU) → PX4 EKF2 (NED) bridge. STUB."""

    def __init__(self):
        super().__init__('vio_bridge')
        self.get_logger().info(
            'VioBridge stub loaded. '
            'Full implementation added in Phase 4.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = VioBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
