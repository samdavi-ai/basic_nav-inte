#!/usr/bin/env python3
"""Gazebo trial of a staging pose that permits a full turn inside the study."""
import math
from baseline_coordinator import AutonomousSupervisorNode as BaselineCoordinator


class AutonomousSupervisorNode(BaselineCoordinator):
    def _insert_room_doorway_waypoints(self, waypoints, start_x, start_y):
        stages = super()._insert_room_doorway_waypoints(waypoints, start_x, start_y)
        # Wall inner face + padded corner radius + Nav2 XY goal tolerance,
        # rounded outward to the next cell. Keep the doorway centerline.
        stage_y = math.ceil((1.10 + math.hypot(.36, .35) + .25) / .05) * .05
        for stage in stages:
            if stage.get("internal_route_hint") and stage["name"] == "Study Room doorway staging":
                stage["coords"]["y"] = stage_y
        return stages


if __name__ == '__main__':
    import rclpy
    rclpy.init()
    node=AutonomousSupervisorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
