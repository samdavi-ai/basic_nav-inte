#!/usr/bin/env python3
"""Isolated Gazebo trial. Keep the west bedroom exit leg straight through the opening."""
import math
from baseline_coordinator import AutonomousSupervisorNode as BaselineCoordinator
from baseline_coordinator import main as baseline_main


class AutonomousSupervisorNode(BaselineCoordinator):
    def _insert_room_doorway_waypoints(self, waypoints, start_x, start_y):
        route = super()._insert_room_doorway_waypoints(waypoints, start_x, start_y)
        for stage in route:
            if stage.get("internal_route_hint") and stage["name"] == "Master Bedroom doorway staging":
                if math.sin(stage["coords"]["yaw"]) < 0:
                    # Hall-side face less full turning radius and XY goal tolerance.
                    stage["coords"]["y"] = math.floor((.90 - math.hypot(.36, .35) - .25) / .05) * .05
        return route


if __name__ == "__main__":
    import baseline_coordinator
    baseline_coordinator.AutonomousSupervisorNode = AutonomousSupervisorNode
    baseline_main()
