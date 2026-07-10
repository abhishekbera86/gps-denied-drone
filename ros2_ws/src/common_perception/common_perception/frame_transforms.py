"""frame_transforms — ENU/FLU (ROS convention) <-> NED/FRD (PX4 convention).

PX4's `VehicleOdometry` expects position/velocity in NED (or FRD, for
non-North-aligned world frames such as OpenVINS's own) and a body-to-world
attitude quaternion in the same convention. ROS-ecosystem VIO output
(OpenVINS, and most everything else) uses ENU world / FLU body. These are
simple axis conversions at the vector level (not full rotations) — the
standard "manually invert the right axes" approach, not a general-purpose
rotation library. Implemented directly here rather than depending on PX4's
C++ `frame_transforms` header, since this workspace is pure `ament_python`.

Quaternion convention throughout: Hamilton, (w, x, y, z), body-to-world —
matching `VehicleOdometry.msg`'s documented convention.
"""

Vec3 = tuple  # (float, float, float)
Quat = tuple  # (float, float, float, float), Hamilton w,x,y,z

# Static quaternions for the two axis conversions, each a 180-degree
# rotation and its own inverse (applying either twice is a no-op).
_NED_ENU_Q: Quat = (0.0, 0.70710678, 0.70710678, 0.0)
_FRD_FLU_Q: Quat = (0.0, 1.0, 0.0, 0.0)


def enu_to_ned(v: Vec3) -> Vec3:
    """Convert an ENU (East, North, Up) vector to NED (North, East, Down).

    This swap-first-two/negate-third axis remap is its own inverse (applying
    it to a NED/FRD vector yields ENU/FLU), since it only ever permutes/
    negates positions 0/1/2 — it never inspects which convention the input
    is actually in. `state_tf_publisher.py` relies on this to go NED->ENU
    with the same function, rather than a near-duplicate `ned_to_enu`.
    """
    east, north, up = v
    return (north, east, -up)


def flu_to_frd(v: Vec3) -> Vec3:
    """Convert an FLU (Forward, Left, Up) body-frame vector to FRD (Forward, Right, Down)."""
    forward, left, up = v
    return (forward, -left, -up)


def quat_rotate_vector(q: Quat, v: Vec3) -> Vec3:
    """Rotate a vector by a Hamilton (w, x, y, z) unit quaternion: q * v * q^-1.

    Needed for `nav_msgs/Odometry.twist.twist.linear`, which per ROS/REP-103
    convention (and confirmed in `ov_msckf`'s `ROS2Visualizer`,
    `child_frame_id="imu"`) is expressed in the BODY frame, not the world
    frame `pose.pose` is in. A body-frame vector cannot become a valid
    world-frame vector via axis-relabeling alone (`flu_to_frd`/`enu_to_ned`)
    — it must first be rotated into the world frame by the body's own
    orientation. Skipping this step silently publishes body-frame velocity
    as if it were world-frame velocity: harmless only while body and world
    yaw coincide, and divergent everywhere else (confirmed live: a real
    square_mission flight, whose whole point is a different yaw on each of
    its 4 legs, drifted the EKF2 position estimate ~50m off after a couple
    of legs before this fix). See
    resource/phase3-gps-denied-localization-source.md.
    """
    qw, qx, qy, qz = q
    vx, vy, vz = v
    # q * (0, v) * conj(q), expanded (q is assumed unit-norm so conj == inverse).
    uvx = qy * vz - qz * vy
    uvy = qz * vx - qx * vz
    uvz = qx * vy - qy * vx
    uuvx = qy * uvz - qz * uvy
    uuvy = qz * uvx - qx * uvz
    uuvz = qx * uvy - qy * uvx
    return (
        vx + 2.0 * (qw * uvx + uuvx),
        vy + 2.0 * (qw * uvy + uuvy),
        vz + 2.0 * (qw * uvz + uuvz),
    )


def quat_mul(a: Quat, b: Quat) -> Quat:
    """Hamilton product a*b, both (w, x, y, z) body-to-world quaternions.

    Composition order matters: for chained frames, a child frame's rotation
    relative to its parent composes on the RIGHT (q_world_child =
    q_world_parent * q_parent_child) — relied on by
    openvins_odometry_bridge.py's camera-mount compensation and verified
    numerically there (see its mount-compensation comment).
    """
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    )


# Backwards-compatible private alias (flu_enu_to_frd_ned_quaternion below
# predates quat_mul being public API).
_quat_mul = quat_mul


def quat_conjugate(q: Quat) -> Quat:
    """Conjugate of a Hamilton (w, x, y, z) quaternion (= inverse for unit q)."""
    return (q[0], -q[1], -q[2], -q[3])


def flu_enu_to_frd_ned_quaternion(q_flu_enu: Quat) -> Quat:
    """Convert a body(FLU)-to-world(ENU) quaternion to body(FRD)-to-world(NED).

    Hamilton (w, x, y, z) in both directions, matching `VehicleOdometry.msg`.

    Also its own inverse: `_NED_ENU_Q`/`_FRD_FLU_Q` are each 180-degree
    rotations, so as quaternion VALUES they satisfy Q*Q = -identity — but
    -identity and +identity represent the SAME rotation, and the two sign
    flips from substituting Q for Q^-1 on both sides of the product cancel
    exactly. So this same function converts FRD/NED -> FLU/ENU too, which
    `state_tf_publisher.py` relies on instead of a near-duplicate inverse.
    """
    return _quat_mul(_quat_mul(_NED_ENU_Q, q_flu_enu), _FRD_FLU_Q)
