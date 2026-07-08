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
        self._area_length_m = self._require_param('area_length_m')
        self._area_width_m = self._require_param('area_width_m')
        self._lane_spacing_m = self._require_param('lane_spacing_m')

    def build_waypoints(self) -> list[tuple[float, float, float, float]]:
        length = self._area_length_m
        width = self._area_width_m
        spacing = self._lane_spacing_m
        height = self._takeoff_height_m

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
