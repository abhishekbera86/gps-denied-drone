#!/usr/bin/env python3
"""state_tf_publisher — broadcasts PX4's live position estimate as a ROS TF
(`odom` -> `base_link`) and an accumulated `nav_msgs/Path`, so RViz2 can show
the vehicle's current pose and the trajectory it has actually flown.

Subscribes to `/fmu/out/vehicle_odometry` — PX4's own EKF2 output, published
continuously once the estimator initializes, regardless of mission state and
of which localization source (GPS or vision) is active, and identical in sim
and on real hardware. Deliberately NOT included in
common_missions/launch/autonomy.launch.py: that launch tears everything down
the instant a mission node exits (its on_exit=Shutdown(), see
DEVELOPMENT_STATUS.md part 9), which would kill RViz's view between
missions. This node is instead started once by
`common_perception/launch/viz.launch.py`, up from the start of the sim/hw
session — the same "up from the start" reasoning as docker-compose.gui.
yml's rqt-viewer.

FRAME CONVERSION: VehicleOdometry position/orientation are NED/FRD (PX4's
own convention); RViz/TF want ENU/FLU. `frame_transforms.enu_to_ned` and
`flu_enu_to_frd_ned_quaternion` are each their own inverse (see that
module), so they're reused here unchanged for the NED/FRD -> ENU/FLU
direction rather than duplicating near-identical functions.
"""

import collections

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from tf2_ros import TransformBroadcaster

from common_perception.frame_transforms import enu_to_ned, flu_enu_to_frd_ned_quaternion

PX4_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)

ODOM_FRAME = 'odom'
BASE_FRAME = 'base_link'

# Decoupled from VehicleOdometry's own (much higher, EKF2-driven) publish
# rate — bounds this node's CPU/bandwidth to a fixed 10 Hz regardless.
PUBLISH_PERIOD_S = 0.1

# Caps Path memory/publish cost so a viz session left running across many
# missions (this node is never restarted between them, by design — see
# module docstring) doesn't grow Path unbounded. 20000 poses at 10 Hz is
# well over half an hour of continuous flight history.
MAX_PATH_POSES = 20000


class StateTfPublisher(Node):

    def __init__(self) -> None:
        super().__init__('state_tf_publisher')

        self._tf_broadcaster = TransformBroadcaster(self)
        self._path_pub = self.create_publisher(Path, '/drone/path', 10)
        self._path_poses = collections.deque(maxlen=MAX_PATH_POSES)
        self._latest = None

        self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self._on_odometry, PX4_QOS)
        self.create_timer(PUBLISH_PERIOD_S, self._publish)

        self.get_logger().info(
            'state_tf_publisher started: /fmu/out/vehicle_odometry -> '
            f'tf({ODOM_FRAME}->{BASE_FRAME}) + /drone/path')

    def _on_odometry(self, msg: VehicleOdometry) -> None:
        self._latest = msg

    def _publish(self) -> None:
        msg = self._latest
        if msg is None:
            return

        # msg.position/msg.q are numpy float32 arrays (rosidl_generator_py
        # convention for float32[N] fields) — geometry_msgs setters assert
        # on plain Python float, so cast explicitly rather than relying on
        # numpy's implicit float coercion (which this assertion rejects).
        position_enu = enu_to_ned(tuple(float(v) for v in msg.position))
        q_flu_enu = flu_enu_to_frd_ned_quaternion(tuple(float(v) for v in msg.q))
        stamp = self.get_clock().now().to_msg()

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = ODOM_FRAME
        tf_msg.child_frame_id = BASE_FRAME
        tf_msg.transform.translation.x = position_enu[0]
        tf_msg.transform.translation.y = position_enu[1]
        tf_msg.transform.translation.z = position_enu[2]
        tf_msg.transform.rotation.w = q_flu_enu[0]
        tf_msg.transform.rotation.x = q_flu_enu[1]
        tf_msg.transform.rotation.y = q_flu_enu[2]
        tf_msg.transform.rotation.z = q_flu_enu[3]
        self._tf_broadcaster.sendTransform(tf_msg)

        pose = PoseStamped()
        pose.header = tf_msg.header
        pose.pose.position.x = position_enu[0]
        pose.pose.position.y = position_enu[1]
        pose.pose.position.z = position_enu[2]
        pose.pose.orientation = tf_msg.transform.rotation
        self._path_poses.append(pose)

        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = ODOM_FRAME
        path.poses = list(self._path_poses)
        self._path_pub.publish(path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StateTfPublisher()
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
