#!/usr/bin/env python3
"""MissionBase — waypoint missions on top of the common_control primitives.

A mission is just an OffboardControlNode with a waypoint queue: the base
class already handles arming, offboard mode, takeoff, waypoint following,
landing and disarm. Subclasses only describe *where to fly*:

    class MyMission(MissionBase):
        def declare_mission_parameters(self):   # ROS params for geometry
            self.declare_parameter('side_length_m', 4.0)

        def build_waypoints(self):              # the actual route
            return [self.waypoint(north, east, height_m, yaw_deg), ...]

Waypoints are given in the friendly frame (north, east, height-above-ground,
yaw in degrees); `waypoint()` converts to the NED tuple common_control flies.
"""

import math

import rclpy

from common_control.offboard_control_node import OffboardControlNode


class MissionBase(OffboardControlNode):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name=node_name)
        self.declare_mission_parameters()
        waypoints = self.build_waypoints()
        self.set_waypoints(waypoints)
        self.get_logger().info(f'{node_name}: {len(waypoints)} waypoints queued')

    # ── Hooks for subclasses ─────────────────────────────────────────────
    def declare_mission_parameters(self) -> None:
        """Declare mission-geometry ROS parameters (called before build_waypoints)."""

    def build_waypoints(self) -> list[tuple[float, float, float, float]]:
        """Return the route as a list of `waypoint()` tuples."""
        raise NotImplementedError

    # ── Helpers ──────────────────────────────────────────────────────────
    def waypoint(self, north_m: float, east_m: float, height_m: float,
                 yaw_deg: float = 0.0) -> tuple[float, float, float, float]:
        """Build one NED waypoint from height-above-ground and yaw in degrees."""
        return (north_m, east_m, -abs(height_m), math.radians(yaw_deg))

    @staticmethod
    def heading_deg(from_ne: tuple[float, float], to_ne: tuple[float, float]) -> float:
        """Yaw (degrees from north, positive toward east) facing from → to."""
        return math.degrees(math.atan2(to_ne[1] - from_ne[1], to_ne[0] - from_ne[0]))


def run_mission(mission_class) -> None:
    """Shared main(): spin the mission node through its full flight cycle."""
    rclpy.init()
    node = mission_class()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except rclpy.executors.ExternalShutdownException:
        # Raised by spin() when the DONE state calls rclpy.shutdown() —
        # this is the normal mission-complete exit path.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
