"""Gazebo-only recovery guard. All drive output still passes through the normal mux."""
import json
import math
import os

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String
from dynamic_guard import ObstacleGuard, rclpy, seconds


class GenerationGuard(ObstacleGuard):
    def __init__(self):
        super().__init__()
        self.generation = {"grid": 0, "global": 0}
        self.approval = None
        self.phase = None
        self.phase_entered = 0.
        self.clearance_recovery = os.environ.get('VIRA_CLEARANCE_RECOVERY') == '1'
        self.latched = False
        self.planner_fence = os.environ.get('VIRA_PLANNER_FENCE') == '1'
        self.planning = set()
        self.plan_barrier_stamp = None
        if self.planner_fence:
            from action_msgs.msg import GoalStatusArray
            self.create_subscription(GoalStatusArray, '/compute_path_to_pose/_action/status',
                                     self.on_planner_status, 10)
        self.test_phase = self.declare_parameter('interrupt_phase', '').value
        self.create_subscription(Bool, '/terrain/emergency_stop', self.on_latch, 10)

    def on_planner_status(self, message):
        active = {bytes(s.goal_info.goal_id.uuid) for s in message.status_list
                  if s.status in (1, 2, 3)}
        if active - self.planning:
            self.approval = None
            self.plan_barrier_stamp = self.now()
            self.pub.publish(Twist())
        self.planning = active

    def on_plan(self, message):
        super().on_plan(message)
        if self.planner_fence and self.plan is not None:
            self.plan_barrier_stamp = self.stamps['plan']

    def on_latch(self, message):
        if message.data or self.latched:
            self.approval = self.plan = self.pose_path = self.hold_since = self.phase = None
            self.plan_valid = False
            self.request = Twist()
            self.stamps.pop('command', None)
        self.latched = message.data

    def on_grid(self, message):
        stamp = seconds(message.header.stamp)
        if stamp <= self.stamps.get('grid', -1):
            return
        super().on_grid(message)
        if self.stamps.get('grid') == stamp:
            self.generation['grid'] += 1

    def on_global(self, message):
        stamp = seconds(message.header.stamp)
        if stamp <= self.stamps.get('global', -1):
            return
        super().on_global(message)
        if self.stamps.get('global') == stamp:
            self.generation['global'] += 1

    def change(self, phase, now, reason=None):
        self.phase, self.phase_entered = phase, now
        self.events.publish(String(data=json.dumps(dict(state=phase, sim_time=now, reason=reason,
            generation=self.generation, hold_since=self.hold_since, global_plan_stamp=self.stamps.get('plan'),
            local_rollout_stamp=self.stamps.get('rollout'), ages={k:now-v for k,v in self.stamps.items()}))))

    def rollout_valid(self, now):
        if not self.pose_path or not 0 <= now-self.stamps.get('rollout',-100) <= .15:
            return False
        return all(self.clear(x,y,h,self.res/4)
                   for x,y,h in self.interpolate(self.pose_path,self.half,self.res/2))

    def step(self):
        now = self.now()
        cmd = self.request
        active = 0 <= now-self.stamps.get('command',-100) <= .15
        moving = abs(cmd.linear.x)+abs(cmd.angular.z) > .001
        fresh = self.half is not None and all(0 <= now-self.stamps.get(k,-100) <= age for k,age in
                        [('grid',1.15),('global',2.1),('scan',.35),('depth',.5),('odom',.2)])
        supported = all(math.isfinite(v) for v in (cmd.linear.x,cmd.angular.z)) and cmd.linear.x>=0 and cmd.linear.y==0
        output = Twist()
        version = (self.generation['grid'],self.generation['global'],self.stamps.get('plan'),self.stamps.get('rollout'))
        if self.approval != version:
            self.approval = None
        if self.latched:
            self.approval = None
        elif self.planner_fence and (self.planning or self.plan_barrier_stamp is None or
                min(self.stamps.get(k,-1) for k in ('global','grid','scan','depth')) < self.plan_barrier_stamp):
            self.approval = None
        elif self.hold_since is None and active and moving:
            # RotationShim has no DWB rollout. Its in-place braking sweep still
            # includes every corner and measured translational velocity.
            rotation = abs(cmd.linear.x)<1e-6 and fresh and abs(self.odom.twist.twist.linear.x)<.02
            path_ok = fresh and self.plan_valid and (rotation or self.rollout_valid(now))
            original_brakes = supported and path_ok and self.brake_corridor_clear(cmd,now)
            if self.clearance_recovery and supported and path_ok and not original_brakes:
                self.validated_plan_stamp = self.stamps['plan']
                reduced = self.admissible_recovery_command(cmd,now)
                if reduced is not None:
                    cmd, original_brakes = reduced, True
            if original_brakes:
                self.approval = version
                output = cmd
                if self.phase != 'NAVIGATING':
                    self.change('NAVIGATING',now)
            else:
                self.hold_since = now
                self.change('OBSTACLE_DETECTED',now,'invalid_global' if not self.plan_valid else
                            ('stale_observations' if not fresh else ('invalid_rollout' if not path_ok else 'braking')))
        elif self.hold_since is not None:
            start = self.hold_since
            if now-start > 12 and self.phase != 'RECOVERY_EXPIRED':
                self.change('RECOVERY_EXPIRED',now)
            elif self.phase == self.test_phase:
                pass  # Private interruption barrier holds the already stopped route.
            elif self.phase == 'OBSTACLE_DETECTED':
                self.change('SAFETY_STOP',now)
            elif self.phase == 'SAFETY_STOP' and fresh and abs(self.odom.twist.twist.linear.x)<.02 and abs(self.odom.twist.twist.angular.z)<.05:
                self.change('COSTMAP_UPDATE',now)
            elif self.phase == 'COSTMAP_UPDATE' and fresh and min(self.stamps[k] for k in ('grid','global','depth','scan'))>start:
                self.change('ROUTE_REASSESSMENT',now)
            elif self.phase == 'ROUTE_REASSESSMENT' and fresh:
                self.validate_plan()
                self.change('REPLANNING',now)
            elif self.phase == 'REPLANNING' and fresh and self.stamps.get('plan',0)>self.phase_entered:
                self.change('PATH_VALIDATION',now)
            elif self.phase == 'PATH_VALIDATION' and fresh and active and moving and supported and self.plan_valid:
                if (self.stamps.get('rollout',0)>=self.stamps['plan'] and self.rollout_valid(now)
                        and self.brake_corridor_clear(cmd,now)):
                    self.approval=version
                    self.hold_since=None
                    self.change('RESUMING',now)
                elif abs(cmd.linear.x)<1e-6 and abs(self.odom.twist.twist.linear.x)<.02 and self.brake_corridor_clear(cmd,now):
                    output=cmd
        if active or self.publishing:
            self.pub.publish(output)
        self.publishing=active


if __name__=='__main__':
    rclpy.init()
    node=GenerationGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
