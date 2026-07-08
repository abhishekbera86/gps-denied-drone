#!/usr/bin/env python3
"""SquareMission — fly a square at takeoff height and land back at the start.

Corners in order: origin → north → north-east → east → origin, nose pointed
along the direction of travel on each leg.
"""

from common_missions.mission_base import MissionBase, run_mission


class SquareMission(MissionBase):

    def __init__(self) -> None:
        super().__init__('square_mission')

    def declare_mission_parameters(self) -> None:
        self._side_length_m = self._require_param('side_length_m')

    def build_waypoints(self) -> list[tuple[float, float, float, float]]:
        side = self._side_length_m
        height = self._takeoff_height_m

        corners = [(side, 0.0), (side, side), (0.0, side), (0.0, 0.0)]
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
