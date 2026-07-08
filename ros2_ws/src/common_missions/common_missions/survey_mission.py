#!/usr/bin/env python3
"""SurveyMission — lawnmower coverage of a rectangle, then return and land.

Lanes run along the north axis (`area_length_m` long), stepped east every
`lane_spacing_m` across `area_width_m`, alternating direction like mowing a
lawn. Ends back at the origin so the landing spot matches the takeoff spot.
"""

from common_missions.mission_base import MissionBase, run_mission


class SurveyMission(MissionBase):

    def __init__(self) -> None:
        super().__init__('survey_mission')

    def declare_mission_parameters(self) -> None:
        self.declare_parameter('area_length_m', 8.0)
        self.declare_parameter('area_width_m', 6.0)
        self.declare_parameter('lane_spacing_m', 2.0)

    def build_waypoints(self) -> list[tuple[float, float, float, float]]:
        length = self.get_parameter('area_length_m').value
        width = self.get_parameter('area_width_m').value
        spacing = self.get_parameter('lane_spacing_m').value
        height = self.get_parameter('takeoff_height_m').value

        corners = []
        east = 0.0
        northbound = True
        while east <= width + 1e-6:
            if northbound:
                corners += [(0.0, east), (length, east)]
            else:
                corners += [(length, east), (0.0, east)]
            northbound = not northbound
            east += spacing
        corners.append((0.0, 0.0))

        waypoints = []
        previous = (0.0, 0.0)
        for corner in corners:
            if corner == previous:
                continue
            yaw = self.heading_deg(previous, corner)
            waypoints.append(self.waypoint(corner[0], corner[1], height, yaw))
            previous = corner
        return waypoints


def main(args=None) -> None:
    run_mission(SurveyMission)


if __name__ == '__main__':
    main()
