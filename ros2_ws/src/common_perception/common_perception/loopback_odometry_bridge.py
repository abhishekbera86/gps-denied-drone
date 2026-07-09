#!/usr/bin/env python3
"""loopback_odometry_bridge — Milestone A: a fake VIO source for testing.

Republishes PX4's own current best estimate (`/fmu/out/vehicle_odometry`)
back in as external vision (`/fmu/in/vehicle_visual_odometry`). This proves
the whole localization-source mechanism — the MAVLink switch, EKF2 accepting
vision fusion, `common_control`/`common_missions` flying unmodified — using
a zero-drift, always-available "VIO" stand-in, *before* introducing real
OpenVINS estimation error as a second variable. See
resource/phase3-gps-denied-localization-source.md for the full rationale.

No frame conversion needed here: `/fmu/out/vehicle_odometry` is PX4's own
EKF2 output, already in NED/FRD — the same convention
`/fmu/in/vehicle_visual_odometry` expects. (Contrast with
`openvins_odometry_bridge.py`, whose input is in ROS-convention ENU/FLU and
does need `frame_transforms`.)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from px4_msgs.msg import VehicleOdometry

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class LoopbackOdometryBridge(Node):

    def __init__(self) -> None:
        super().__init__('loopback_odometry_bridge')

        self._pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', PX4_QOS)
        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self._on_odometry, PX4_QOS)

        self.get_logger().info(
            'loopback_odometry_bridge started: /fmu/out/vehicle_odometry -> '
            '/fmu/in/vehicle_visual_odometry (Milestone A fake-VIO stand-in)')

    def _on_odometry(self, msg: VehicleOdometry) -> None:
        out = VehicleOdometry()
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        out.timestamp = now_us
        out.timestamp_sample = now_us

        # Already NED/FRD (PX4's own output) — pass the source frame/data
        # straight through, no axis conversion needed.
        out.pose_frame = msg.pose_frame
        out.position = msg.position
        out.q = msg.q
        out.velocity_frame = msg.velocity_frame
        out.velocity = msg.velocity
        out.angular_velocity = msg.angular_velocity
        out.position_variance = msg.position_variance
        out.orientation_variance = msg.orientation_variance
        out.velocity_variance = msg.velocity_variance
        out.reset_counter = msg.reset_counter
        out.quality = msg.quality

        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoopbackOdometryBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
