#!/usr/bin/env python3
"""openvins_odometry_bridge — Milestone B: feed real OpenVINS VIO into PX4.

Subscribes to OpenVINS's `ov_msckf/odomimu` (`nav_msgs/Odometry`) and
republishes it as `/fmu/in/vehicle_visual_odometry`
(`px4_msgs/VehicleOdometry`) — the exact same target topic
`loopback_odometry_bridge.py` (Milestone A) publishes to, so switching from
the fake-VIO stand-in to real OpenVINS is a drop-in launch-file change, not
a code change anywhere else. See
resource/phase3-gps-denied-localization-source.md for the full design.

FRAME CONVERSION (the one real difference from the Milestone A loopback
bridge, which needs none): OpenVINS publishes in its own convention —
`pose.pose` in a gravity-aligned but NOT North-aligned "global" world frame
(ENU-ish; the yaw offset from true North is arbitrary, fixed at whatever
heading the vehicle had at VIO init), `twist.twist` in the IMU BODY frame
(FLU-ish; per `ov_msckf`'s ROS2Visualizer, `child_frame_id="imu"`). PX4
expects NED/FRD. Since the world yaw is not North-aligned, this publishes
with `pose_frame = POSE_FRAME_FRD` (not NED) — FRD is explicitly the
"non-North-aligned" option `VehicleOdometry.msg` documents for exactly this
case.

TRIED AND REVERTED (2026-07-09): a raw-IMU "stillness override" here (freeze
velocity/position when accel/gyro look at-rest, to fix post-landing drift —
see DEVELOPMENT_STATUS.md "Milestone B result, part 3/4") turned out to have
the same fundamental flaw as the OpenVINS-internal ZUPT approach it was
meant to replace: accelerometer-based "is moving" detection can't see
constant/near-constant velocity, only acceleration, so it stayed falsely
"still" for 23+ seconds during a real, slow (0.2 m/s-capped) takeoff climb
— confirmed live, the vehicle diverged (`x=-7.8,y=-2.6` instead of ~0,0)
once it disengaged, because the frozen belief and reality had drifted apart
during the false-stillness window. Reverted to plain passthrough. The
post-landing drift issue remains open — see DEVELOPMENT_STATUS.md for the
current best next step (a common_control-level explicit disarm-after-
landing-timeout, not a VIO/bridge-layer fix).

WHY THIS NODE MIGHT PUBLISH NOTHING FOR A WHILE AFTER LAUNCH (2026-07-10,
not a bug here): OpenVINS's own `ROS2Visualizer::visualize_odometry()`
refuses to publish `/ov_msckf/odomimu` at all until 1 full second of ITS
OWN internal (Gazebo-simulation-derived) time has passed since
initialization. Under CPU contention, Gazebo's real-time-factor can
collapse hard — confirmed live at 0.023 (~1/44th of real time), at which
that 1-second gate alone needs ~44 REAL seconds, and `/fmu/in/
vehicle_visual_odometry` (this node's own output) can go a genuinely long
time — confirmed once at 250+ seconds — with zero messages while
completely healthy otherwise. If a mission is stuck at PX4's "Preflight
Fail: ekf2 missing data" / "waiting for estimator to initialize", check
`gz topic -e -t /world/<world>/stats` for `real_time_factor` before
assuming a code bug — see README §12 issue 29 and
`set_localization_source.py`'s docstring for the full investigation.
SITL-only: no simulated clock exists on Phase 4 real hardware, so this
cannot recur there.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

from common_perception.frame_transforms import (
    enu_to_ned,
    flu_enu_to_frd_ned_quaternion,
    flu_to_frd,
    quat_rotate_vector,
)

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


class OpenvinsOdometryBridge(Node):

    def __init__(self) -> None:
        super().__init__('openvins_odometry_bridge')

        self._pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', PX4_QOS)
        self.create_subscription(
            Odometry, '/ov_msckf/odomimu', self._on_odometry, 10)

        self.get_logger().info(
            'openvins_odometry_bridge started: /ov_msckf/odomimu -> '
            '/fmu/in/vehicle_visual_odometry (Milestone B real VIO)')

    def _on_odometry(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular

        position_ned = enu_to_ned((p.x, p.y, p.z))
        # geometry_msgs/Quaternion field order is (x, y, z, w); frame_transforms
        # uses Hamilton (w, x, y, z) throughout, matching VehicleOdometry.msg.
        q_flu_enu = (q.w, q.x, q.y, q.z)
        q_ned_frd = flu_enu_to_frd_ned_quaternion(q_flu_enu)
        # msg.twist.twist.linear is body-frame (child_frame_id="imu"), but
        # VehicleOdometry.velocity is world-frame (FRD) — rotate into world
        # (ENU) by the body's own orientation BEFORE the ENU->NED axis
        # conversion. angular_velocity needs no such rotation: both
        # nav_msgs/Odometry's twist.angular and VehicleOdometry's
        # angular_velocity are body-frame, so a plain axis relabel is
        # correct as-is. See frame_transforms.quat_rotate_vector.
        velocity_enu = quat_rotate_vector(q_flu_enu, (v.x, v.y, v.z))
        velocity_frd = enu_to_ned(velocity_enu)
        angular_velocity_frd = flu_to_frd((w.x, w.y, w.z))

        out = VehicleOdometry()
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        out.timestamp = now_us
        out.timestamp_sample = now_us

        # Not North-aligned — OpenVINS's world yaw is arbitrary at init.
        out.pose_frame = VehicleOdometry.POSE_FRAME_FRD
        out.position = list(position_ned)
        out.q = list(q_ned_frd)
        out.velocity_frame = VehicleOdometry.VELOCITY_FRAME_FRD
        out.velocity = list(velocity_frd)
        out.angular_velocity = list(angular_velocity_frd)

        # Best-effort diagonal from OpenVINS's 6x6 row-major covariance
        # (indices 0, 7, 14 = xx, yy, zz). This per-message covariance
        # swinging with however confident (or not) OpenVINS happened to feel
        # on a given frame was a real, confirmed contributor to
        # flight-to-flight instability (resource/Vio_Drift_analysis.txt) —
        # `set_localization_source.py` now raises EKF2_EVP_NOISE/
        # EKF2_EVV_NOISE at the vision-source switch, which PX4 applies as a
        # LOWER BOUND on top of whatever's sent here (PX4's own
        # EKF2_EV_NOISE_MD=0 default — deliberately left untouched, see that
        # module's docstring for why EKF2_EV_NOISE_MD=1 was tried and
        # reverted). So this field still matters when OpenVINS reports
        # something looser than the floor; it's just no longer trusted
        # BELOW that floor.
        pc = msg.pose.covariance
        tc = msg.twist.covariance
        out.position_variance = [pc[0], pc[7], pc[14]]
        out.orientation_variance = [pc[21], pc[28], pc[35]]
        out.velocity_variance = [tc[0], tc[7], tc[14]]

        out.reset_counter = 0
        out.quality = 0

        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OpenvinsOdometryBridge()
    try:
        # Manual spin_once loop, NOT plain rclpy.spin(node) (2026-07-13):
        # confirmed live that under severe host CPU contention (load
        # average 15+ on this project's dev machine, `make sim-gui`'s
        # rviz2/Gazebo GPU rendering alone at 60%+ CPU), rclpy's own
        # message-deserialization step (executors.py's
        # `sub.handle.take_message(...)`, a pybind11 call into the DDS
        # layer, BEFORE `_on_odometry` is ever reached) can raise
        # `RuntimeError: Unable to convert call argument to Python
        # object` — a known category of rclpy/CycloneDDS fragility under
        # resource starvation, not a bug in this node's own logic. Plain
        # `rclpy.spin()` lets that exception escape the whole spin loop
        # and kills the process outright — which silently stops ALL
        # vision data to PX4 for the rest of the flight (EKF2 falls back
        # to dead-reckoning on whatever GPS/estimate it last had, with no
        # warning). Catching it here and continuing keeps this node (and
        # therefore vision fusion) alive through a transient DDS hiccup
        # instead of a hard crash. Doesn't fix the underlying rclpy/DDS
        # fragility (host contention is the real trigger — see README
        # §12) — this is resilience, not a root-cause fix, and it only
        # protects THIS node from dying; it does nothing if the same
        # condition is bad enough to also kill camera_imu_bridge or
        # run_subscribe_msckf. A mission-level watchdog that detects
        # vision going silent regardless of WHICH piece failed and WHY is
        # the complementary, more general fix — see offboard_control_node.py
        # if/when that exists.
        while rclpy.ok():
            try:
                rclpy.spin_once(node, timeout_sec=1.0)
            except RuntimeError as exc:
                node.get_logger().error(
                    f'transient rclpy/DDS error, continuing: {exc}')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
