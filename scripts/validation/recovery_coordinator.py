#!/usr/bin/env python3
"""Private Gazebo trial entry point for the existing mission coordinator."""
import json
import os
from pathlib import Path

import rclpy
from std_msgs.msg import String

from baseline_coordinator import AutonomousSupervisorNode


class RecoveryCoordinator(AutonomousSupervisorNode):
    def __init__(self):
        super().__init__()
        if not self.get_parameter('use_sim_time').value or not os.environ.get('GZ_PARTITION','').startswith('vira-dynamic-'):
            raise RuntimeError('Recovery trial requires a private Gazebo graph')
        self.create_subscription(String,'/dynamic_guard/state',self.on_guard,10)

    def on_guard(self,msg):
        state=json.loads(msg.data)
        if (state['state']=='OBSTACLE_DETECTED' and self.active_goal_handle is not None
                and self.state==self.STATE_MISSION_EXECUTING):
            self._abort_active_navigation_goal(keep_goal_msg=True)
            self._handle_navigation_failure('unsafe_swept_route')

    def _handle_navigation_failure(self, reason):
        if self.active_goal_msg is not None:
            self.active_goal_msg.behavior_tree=str(Path(__file__).with_name('dynamic_replan.xml'))
        super()._handle_navigation_failure(reason)

    def _clear_costmaps_async(self):
        # Keep marked cells during automatic retries. The sensor obstacle layers
        # continue raytracing observations while the robot is held stationary.
        self.get_logger().info('Preserving sensor observations for dynamic route validation')


if __name__=='__main__':
    rclpy.init()
    node=RecoveryCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
