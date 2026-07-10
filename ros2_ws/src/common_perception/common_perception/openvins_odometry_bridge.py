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

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry

from common_perception.frame_transforms import (
    enu_to_ned,
    flu_enu_to_frd_ned_quaternion,
    flu_to_frd,
    quat_conjugate,
    quat_mul,
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

        # CAMERA MOUNT COMPENSATION (2026-07-10): the d435i is mounted
        # pitched down (30 deg in sim — x500_d435i_depth/model.sdf; keep
        # this parameter in sync via vio.launch.py). OpenVINS estimates the
        # pose of the CAMERA'S OWN IMU frame, which tilts with the mount —
        # so its reported attitude/gyro are for the TILTED frame, offset
        # from the vehicle body by a constant mount rotation. PX4's
        # VehicleOdometry.q documents vehicle-body attitude, and EKF2's
        # external-vision frame alignment assumes the vision frame differs
        # from its own by YAW only — a constant uncompensated pitch there
        # is exactly the kind of silent frame error Milestone B's
        # extrinsics bug taught this project to fear. Compensation (math
        # numerically validated against 5 cases incl. non-commuting
        # rotations and gravity/gyro sanity — scratchpad
        # validate_mount_quat.py, all passing, before this was written):
        #   q_vehicle = q_imu * conj(q_mount)     (mount composes on the right)
        #   w_body    = rotate(q_mount, w_imu)    (gyro coords imu -> body)
        #   velocity  : UNCHANGED — v_imu is expressed in the IMU frame, so
        #               it must still be rotated into world by the ORIGINAL
        #               q_imu (using the compensated q here would be wrong).
        #   position  : UNCHANGED — a point is a point; the mount offset is
        #               EKF2_EV_POS_X/Y/Z's job (set_localization_source.py)
        #               and a pure rotation about the sensor origin doesn't
        #               move it.
        # NOTE this is the vehicle-body <-> camera-IMU relationship, NOT
        # the camera <-> camera-IMU one: kalibr_imucam_chain.yaml's
        # T_cam_imu is invariant under the mount tilt (both sensors sit on
        # the same rigid link and rotate together) and must NOT change
        # with this parameter.
        self.declare_parameter('mount_pitch_deg', 0.0)
        pitch = math.radians(
            self.get_parameter('mount_pitch_deg').get_parameter_value().double_value)
        # Pitch-down about body +Y (FLU), Hamilton (w, x, y, z).
        self._q_mount = (math.cos(pitch / 2), 0.0, math.sin(pitch / 2), 0.0)
        self._q_mount_conj = quat_conjugate(self._q_mount)

        self._pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_visual_odometry', PX4_QOS)
        self.create_subscription(
            Odometry, '/ov_msckf/odomimu', self._on_odometry, 10)

        self.get_logger().info(
            'openvins_odometry_bridge started: /ov_msckf/odomimu -> '
            '/fmu/in/vehicle_visual_odometry (Milestone B real VIO), '
            f'mount_pitch_deg={math.degrees(pitch):.1f}')

    def _on_odometry(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular

        position_ned = enu_to_ned((p.x, p.y, p.z))
        # geometry_msgs/Quaternion field order is (x, y, z, w); frame_transforms
        # uses Hamilton (w, x, y, z) throughout, matching VehicleOdometry.msg.
        # q_imu is the TILTED camera-IMU frame's attitude; q_vehicle removes
        # the fixed mount pitch (see the mount-compensation comment in
        # __init__ — velocity below must keep using q_imu, not q_vehicle).
        q_imu_flu_enu = (q.w, q.x, q.y, q.z)
        q_vehicle_flu_enu = quat_mul(q_imu_flu_enu, self._q_mount_conj)
        q_ned_frd = flu_enu_to_frd_ned_quaternion(q_vehicle_flu_enu)
        # msg.twist.twist.linear is body-frame (child_frame_id="imu"), but
        # VehicleOdometry.velocity is world-frame (FRD) — rotate into world
        # (ENU) by the frame the vector actually lives in (the IMU frame,
        # hence q_imu NOT q_vehicle) BEFORE the ENU->NED axis conversion.
        # angular_velocity: gyro rates are also IMU-frame — rotate into the
        # vehicle body frame by the mount rotation, then axis-relabel
        # FLU->FRD (both nav_msgs twist.angular and VehicleOdometry's
        # angular_velocity are body-frame).
        velocity_enu = quat_rotate_vector(q_imu_flu_enu, (v.x, v.y, v.z))
        velocity_frd = enu_to_ned(velocity_enu)
        angular_velocity_body_flu = quat_rotate_vector(self._q_mount, (w.x, w.y, w.z))
        angular_velocity_frd = flu_to_frd(angular_velocity_body_flu)

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
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
