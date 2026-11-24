#!/usr/bin/env python3
"""Isolated Gazebo trial. Keep the west bedroom exit leg straight through the opening."""
import math
from baseline_coordinator import AutonomousSupervisorNode as BaselineCoordinator
from baseline_coordinator import main as baseline_main


class AutonomousSupervisorNode(BaselineCoordinator):
    def _insert_room_doorway_waypoints(self, waypoints, start_x, start_y):
        route = super()._insert_room_doorway_waypoints(waypoints, start_x, start_y)
        for prev, stage in zip(route, route[1:]):
            # The bedroom target is only 0.60 m from the original staging
            # point, so baseline deduplication omits its exit stage entirely.
            # Use the existing transition slot for the first portal crossed.
            if prev["name"] == "Master Bedroom" and stage.get("internal_route_hint"):
                y = math.floor((.90 - math.hypot(.36, .35) - .25) / .05) * .05
                stage.update(name="Master Bedroom hallway exit",
                             coords=dict(x=-5., y=y, yaw=-math.pi/2, frame_id="map"))
        return route


if __name__ == "__main__":
    import baseline_coordinator
    baseline_coordinator.AutonomousSupervisorNode = AutonomousSupervisorNode
    baseline_main()
