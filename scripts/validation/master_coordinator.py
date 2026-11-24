#!/usr/bin/env python3
"""Private mission owner for versioned recovery. Imported only inside the trial overlay."""
import json
import os
from pathlib import Path

import rclpy
from std_msgs.msg import String
from doorway_trial import AutonomousSupervisorNode


class MasterCoordinator(AutonomousSupervisorNode):
    def __init__(self):
        super().__init__()
        if not self.get_parameter('use_sim_time').value or not os.environ.get('GZ_PARTITION','').startswith('vira-dynamic-'):
            raise RuntimeError('A private Gazebo graph is required')
        self.last_replan = None
        self.create_subscription(String,'/dynamic_guard/state',self.on_guard,10)

    def on_guard(self, msg):
        state=json.loads(msg.data)
        if self.terrain_emergency_active or not self.active_mission_id or self.state!=self.STATE_MISSION_EXECUTING:
            return
        if state['state']=='RECOVERY_EXPIRED':
            self._abort_active_navigation_goal()
            self._finalize_mission(False,'Recovery deadline expired without a validated route')
        if state['state']!='REPLANNING' or self.active_goal_handle is None:
            return
        x,y,_=self.get_current_map_pose()
        request=(self.active_mission_id,self.current_waypoint_index,round(x/.05),round(y/.05))
        if self.last_replan!=request:
            self.last_replan=request
            self._abort_active_navigation_goal(keep_goal_msg=True)
            if os.environ.get('VIRA_CLEARANCE_RECOVERY')=='1':
                self.active_goal_msg.behavior_tree=str(Path(__file__).with_name('dynamic_replan.xml'))
            self._handle_navigation_failure('stopped and reassessed route')

    def _clear_costmaps_async(self):
        self.get_logger().info('Retaining current obstacle observations during recovery')

    def handle_emergency_stop(self):
        super().handle_emergency_stop()
        self._pending_replacement_mission=None
        if self.active_mission_id:
            self._finalize_mission(False,'Operator emergency stop',cancelled=True)

    def transition_to(self,new_state,reason=''):
        if getattr(self,'terrain_emergency_active',False) and self.state==self.STATE_EMERGENCY_STOP:
            if new_state==self.STATE_MISSION_CANCELLED:
                self._emit_kernel_event('AutonomyStateChanged',dict(previous_state=self.state,state=new_state,
                    reason=reason,mission_id=self.active_mission_id,mission_name=self.active_mission_name))
                return
            if new_state==self.STATE_AUTONOMY_READY:
                return
        super().transition_to(new_state,reason)


if __name__=='__main__':
    rclpy.init()
    node=MasterCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
