#!/usr/bin/env python3
"""wait_for_vision — block until vision odometry is actually flowing to PX4.

One-shot launch-sequencing gate (same pattern as `set_localization_source`):
subscribes to `/fmu/in/vehicle_visual_odometry` and exits 0 on the first
message, or exits 1 (loudly) after --timeout seconds with nothing received.
`sim_bringup/launch/sim.launch.py` runs this between starting the VIO
pipeline and starting the flight/mission node whenever
`localization_source:=vision`, so a vision mission structurally CANNOT arm
before vision data exists.

WHY THIS EXISTS (2026-07-10, confirmed live — the first tilted-camera test
flight): PX4's arming check passes on EKF2's CURRENT estimate validity,
and right after `set_localization_source` flips GPS off, that estimate is
still valid from boot-time GPS fusion for a while. In that window a
mission can arm and take off with ZERO functioning localization source —
OpenVINS's initializer was stuck below its feature threshold the entire
flight (a world-texture bug, since fixed), no vision message was ever
published, and the vehicle "flew" 125 s on unaided dead-reckoning until
the estimate wandered ~5 m and the geofence aborted it on a phantom
position (Gazebo ground truth showed the vehicle essentially at the
origin). This gate turns that silent false flight into an immediate, loud
pre-arm failure.

Scope note: this covers "vision never came up" (pre-arm). "Vision died
mid-flight" is a separate, harder problem — the planned Phase B fallback
work — deliberately not attempted here.
"""

import argparse
import sys

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

TOPIC = '/fmu/in/vehicle_visual_odometry'


def main(args=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--timeout', type=float, default=90.0,
        help='Seconds to wait for the first vision odometry message before '
             'failing (default 90 — OpenVINS static init takes a few seconds '
             'at real-time factor 1.0, but leave margin for a loaded host; '
             'see README §12 issue 29/30 on RTF)')
    parsed, ros_args = parser.parse_known_args(args)

    rclpy.init(args=ros_args)
    node = Node('wait_for_vision')
    got = []
    node.create_subscription(VehicleOdometry, TOPIC, lambda m: got.append(True), PX4_QOS)

    node.get_logger().info(
        f'waiting up to {parsed.timeout:.0f}s for vision odometry on {TOPIC}...')
    # Poll in slices so we can time out; the clock here is wall time, which
    # is what a human debugging a hung bringup experiences.
    deadline = node.get_clock().now().nanoseconds + int(parsed.timeout * 1e9)
    while not got and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    ok = bool(got)
    if ok:
        node.get_logger().info('vision odometry is flowing — clear to start the mission')
    else:
        node.get_logger().fatal(
            f'NO vision odometry on {TOPIC} within {parsed.timeout:.0f}s — '
            'refusing to let a vision mission arm without vision. Check that '
            'OpenVINS initialized (look for "successful initialization" in its '
            'log; a feature-starved camera view blocks init) and that the '
            'camera bridge is up. See README §12 issue 30.')

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
