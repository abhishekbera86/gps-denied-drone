#!/usr/bin/env python3
"""SquareMission — fly a square CENTERED on the takeoff point, land back at it.

The square is symmetric around the local origin (corners at ±side/2), with a
final waypoint returning to the origin before landing, nose pointed along
the direction of travel on each leg.

CENTERED, not first-quadrant (changed 2026-07-13): the original route flew
corners (side,0)→(side,side)→(0,side)→(0,0) — entirely into the first
quadrant, with the takeoff/landing origin sitting at the route's own
corner. In the vio_test world that placed the landing point only 1.5m from
two fence-prop lines, and with mono-VIO's known stochastic estimate drift,
user-observed flights drifted into the props during landing. Centering the
square puts the landing point at the maximum possible distance from every
fence line (the world was recentered on the origin at the same time — see
docker/px4_sitl_worlds/vio_test.sdf), and the auto-derived geofence
(origin + waypoints bounding box, offboard_control_node._geofence_bounds)
follows the new symmetric shape with no geofence code change.
"""

from common_missions.mission_base import MissionBase, run_mission


class SquareMission(MissionBase):

    def __init__(self) -> None:
        super().__init__('square_mission')

    def declare_mission_parameters(self) -> None:
        self._side_length_m = self._require_param('side_length_m')

    def build_waypoints(self) -> list[tuple[float, float, float, float]]:
        half = self._side_length_m / 2.0
        height = self._takeoff_height_m

        # Corners of a square centered on the origin, then an explicit
        # return-to-center waypoint so the vehicle lands mid-fence rather
        # than at a corner of its own route.
        corners = [
            (half, -half),
            (half, half),
            (-half, half),
            (-half, -half),
            (0.0, 0.0),
        ]
        waypoints = []
        previous = (0.0, 0.0)
        for corner in corners:
            yaw = self.heading_deg(previous, corner)
            waypoints.append(self.waypoint(corner[0], corner[1], height, yaw))
            previous = corner
        return waypoints


def main(args=None) -> None:
    run_mission(SquareMission)


if __name__ == '__main__':
    main()
