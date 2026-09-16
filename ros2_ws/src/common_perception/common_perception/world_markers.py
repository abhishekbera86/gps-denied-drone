#!/usr/bin/env python3
"""world_markers — static RViz MarkerArray built from the active world's SDF.

Parses `/px4_sitl_worlds/<world_name>.sdf` (bind-mounted read-only into this
container, see docker-compose.gui.yml's `rviz2` service) for every box prop
and publishes them once as CUBE markers in the `odom` frame — the actual
fence/checkerboard geometry the vehicle flies against in Gazebo (see
docker/px4_sitl_worlds/vio_test.sdf), not a hand-copied duplicate that could
drift out of sync with it.

No frame conversion needed: vio_test.sdf declares
`spherical_coordinates/world_frame_orientation: ENU`, and the vehicle spawns
at the world origin — so SDF world-frame (x=East, y=North, z=Up) coordinates
already equal this project's `odom` frame (see state_tf_publisher.py)
one-to-one.

Published once (with TRANSIENT_LOCAL durability, so a late-joining RViz2
still receives it), then this node just spins to keep that latched sample
alive for any RViz2 window opened after it.
"""

import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray

LATCHED_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)


def _euler_to_quat(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """Standard XYZ-intrinsic Euler (SDF's convention) -> Hamilton (x, y, z, w)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class WorldMarkers(Node):
    """Publishes the active world's static box props as an RViz MarkerArray once."""

    def __init__(self) -> None:
        super().__init__('world_markers')
        world_name = self.declare_parameter('world_name', 'empty').value
        sdf_path = f'/px4_sitl_worlds/{world_name}.sdf'

        self._pub = self.create_publisher(MarkerArray, '/world_markers', LATCHED_QOS)

        try:
            markers = self._build_markers(sdf_path)
        except (FileNotFoundError, ET.ParseError) as exc:
            self.get_logger().warn(f"Could not parse world SDF at '{sdf_path}': {exc}")
            markers = []

        self._pub.publish(MarkerArray(markers=markers))
        self.get_logger().info(
            f"Published {len(markers)} static prop marker(s) from '{sdf_path}'")

    def _build_markers(self, sdf_path: str) -> list:
        tree = ET.parse(sdf_path)
        markers = []
        for marker_id, model in enumerate(tree.getroot().iter('model')):
            if model.get('name') == 'ground_plane':
                continue
            box_size = model.find('link/visual/geometry/box/size')
            pose = model.find('pose')
            if box_size is None or pose is None:
                continue
            material = model.find('link/visual/material/diffuse')

            sx, sy, sz = (float(v) for v in box_size.text.split())
            x, y, z, roll, pitch, yaw = (float(v) for v in pose.text.split())
            qx, qy, qz, qw = _euler_to_quat(roll, pitch, yaw)
            if material is not None:
                r, g, b, *rest = (float(v) for v in material.text.split())
                a = rest[0] if rest else 1.0
            else:
                r, g, b, a = 0.6, 0.6, 0.6, 1.0

            marker = Marker()
            marker.header.frame_id = 'odom'
            marker.ns = 'world_props'
            marker.id = marker_id
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = z
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = sx
            marker.scale.y = sy
            marker.scale.z = sz
            marker.color.r, marker.color.g, marker.color.b, marker.color.a = r, g, b, a
            markers.append(marker)
        return markers


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WorldMarkers()
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
