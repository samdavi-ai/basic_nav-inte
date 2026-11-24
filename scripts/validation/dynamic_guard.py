#!/usr/bin/env python3
"""Gazebo candidate. Veto Nav2 twists upstream of the existing velocity mux."""
import json
import math
import os

import numpy as np
import rclpy
from geometry_msgs.msg import PolygonStamped, Twist
from nav2_msgs.msg import Costmap
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from std_msgs.msg import String


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


class ObstacleGuard(Node):
    def __init__(self):
        super().__init__('dynamic_braking_guard')
        if not self.get_parameter('use_sim_time').value or not os.environ.get('GZ_PARTITION','').startswith('vira-dynamic-'):
            raise RuntimeError('This guard requires a private Gazebo validation graph')
        self.recovery = self.declare_parameter('recovery',False).value
        self.govern_speed=self.declare_parameter('govern_recovery_speed',False).value
        self.validated_plan_stamp=None
        self.global_grid=None
        self.plan=None
        self.plan_valid=False
        if self.recovery:
            self.create_subscription(Costmap,'/global_costmap/costmap_raw',self.on_global,2)
            self.create_subscription(Path,'/plan',self.on_plan,5)
        self.stamps = {}
        self.obstacles = None
        self.half = None
        self.odom = None
        self.request = Twist()
        self.state = None
        self.publishing = False
        self.hold_since = None
        self.rest_verified=False
        self.pose_path = None
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)
        self.create_subscription(Costmap, '/local_costmap/costmap_raw', self.on_grid, 2)
        self.create_subscription(PolygonStamped, '/local_costmap/published_footprint', self.on_footprint, 2)
        self.create_subscription(Path, '/local_plan', self.on_rollout, 2)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, '/camera/depth/points', self.on_depth, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_twist, 2)
        self.pub = self.create_publisher(Twist, '/cmd_vel_guarded_nav', 2)
        self.events = self.create_publisher(String, '/dynamic_guard/state', 10)
        self.create_timer(.05, self.step)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_scan(self, m):
        if m.ranges and any(math.isfinite(v) and m.range_min <= v <= m.range_max for v in m.ranges):
            self.stamps['scan'] = seconds(m.header.stamp)

    def on_depth(self, m):
        if m.width*m.height and m.data:
            z = next((f for f in m.fields if f.name=='z' and f.datatype==7),None)
            if z:
                samples = np.ndarray((m.height,m.width),dtype='>f4' if m.is_bigendian else '<f4',
                                     buffer=m.data,offset=z.offset,strides=(m.row_step,m.point_step))
                values = samples[::4,::4]
                if np.any(np.isfinite(values) & (values>0)):
                    self.stamps['depth'] = seconds(m.header.stamp)

    def on_odom(self, m):
        if m.header.frame_id != 'odom':
            return
        self.odom = m
        self.stamps['odom'] = seconds(m.header.stamp)

    def on_grid(self, m):
        if m.header.frame_id != 'odom' or abs(yaw(m.metadata.origin.orientation))>1e-5:
            return
        md = m.metadata
        cells = np.asarray(m.data, dtype=np.uint8).reshape(md.size_y,md.size_x)
        # Lethal and unknown cells occupy their full grid squares.
        iy, ix = np.where(cells >= 254)
        self.res = md.resolution
        self.origin = np.array([md.origin.position.x, md.origin.position.y])
        self.extent = self.origin + self.res*np.array([md.size_x, md.size_y])
        self.obstacles = self.origin + self.res*(.5+np.column_stack((ix,iy)))
        self.stamps['grid'] = seconds(m.header.stamp)

    def on_global(self, m):
        from scipy.spatial import cKDTree
        if m.header.frame_id!='map' or abs(yaw(m.metadata.origin.orientation))>1e-5:
            return
        md=m.metadata
        occupied=np.asarray(m.data, dtype=np.uint8).reshape(md.size_y, md.size_x)>=254
        iy,ix=np.where(occupied)
        origin=np.array([md.origin.position.x,md.origin.position.y])
        xy=origin+(np.column_stack((ix,iy))+.5)*md.resolution
        self.global_grid=(cKDTree(xy),origin,origin+md.resolution*np.array([md.size_x,md.size_y]),md.resolution)
        self.stamps['global']=seconds(m.header.stamp)
        self.validate_plan()

    def on_plan(self,m):
        if m.header.frame_id=='map' and m.poses:
            self.plan=np.array([(p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)) for p in m.poses])
            self.stamps['plan']=seconds(m.header.stamp)
            self.validate_plan()

    @staticmethod
    def interpolate(poses,half,spacing):
        poses=np.asarray(poses,dtype=float).copy()
        poses[:,2]=np.unwrap(poses[:,2])
        delta=np.diff(poses,axis=0)
        span=np.linalg.norm(delta[:,:2],axis=1)+math.sqrt(2)*half*np.abs(delta[:,2])
        count=np.maximum(1,np.ceil(span/spacing).astype(int))
        start=np.repeat(np.cumsum(count)-count,count)
        fractions=((np.arange(sum(count))-start)/np.repeat(count,count))[:,None]
        return np.vstack([np.repeat(poses[:-1],count,axis=0)+fractions*np.repeat(delta,count,axis=0),poses[-1]])

    def validate_plan(self):
        self.plan_valid=False
        if self.plan is None or self.global_grid is None or self.half is None:
            return
        tree,origin,end,res=self.global_grid
        # Validate published orientations and translational tangents. NavFn
        # leaves most intermediate quaternions at identity in a curved route.
        tangent=self.plan.copy()
        delta=np.diff(tangent[:,:2],axis=0)
        tangent[:-1,2]=np.where(np.linalg.norm(delta,axis=1)>1e-7,np.arctan2(delta[:,1],delta[:,0]),tangent[:-1,2])
        for path in (self.plan,tangent):
            poses=self.interpolate(path,self.half,res/2)
            margin=res/4
            extent=self.half*(abs(np.cos(poses[:,2]))+abs(np.sin(poses[:,2])))+margin
            if np.any(poses[:,:2]-extent[:,None]<origin) or np.any(poses[:,:2]+extent[:,None]>end):
                return
            near=tree.query_ball_point(poses[:,:2],math.sqrt(2)*self.half+res/math.sqrt(2)+margin)
            for (x,y,h),indices in zip(poses,near):
                if indices:
                    c,s=math.cos(h),math.sin(h)
                    local=(tree.data[indices]-[x,y])@np.array([[c,-s],[s,c]])
                    if np.any(np.max(abs(local),axis=1)<=self.half+res/math.sqrt(2)+margin):
                        return
        self.plan_valid=True

    def on_footprint(self, m):
        if m.header.frame_id != 'odom' or self.odom is None or len(m.polygon.points)!=4:
            return
        pts = np.array([[p.x,p.y] for p in m.polygon.points])
        edges = np.linalg.norm(pts-np.roll(pts,1,axis=0),axis=1)
        # Circumscribe the published rectangular envelope if its edge order changes.
        self.half = max(edges)/2

    def on_rollout(self, m):
        if m.header.frame_id == 'odom' and m.poses:
            self.pose_path = [(p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)) for p in m.poses]
            self.stamps['rollout'] = seconds(m.header.stamp)

    def on_twist(self, m):
        self.request = m
        self.stamps['command'] = self.now()

    def clear(self, x, y, heading, margin):
        center = np.array([x,y])
        c,s = math.cos(heading),math.sin(heading)
        extent = (abs(c)+abs(s))*self.half + margin
        if np.any(center-extent < self.origin) or np.any(center+extent > self.extent):
            return False
        # Expand by the cell half-diagonal. This encloses every occupied square at any yaw.
        d = self.obstacles-center
        local = d @ np.array([[c,-s],[s,c]])
        return not np.any(np.max(np.abs(local),axis=1)<=self.half+margin+self.res/math.sqrt(2))

    def brake_corridor_clear(self, cmd, now):
        p = self.odom.pose.pose
        tw = self.odom.twist.twist
        # Use the larger measured or requested speed in each axis. Model one
        # publication period plus grid age and one downstream command cycle.
        lag = .5 + max(0., now-self.stamps['grid']) + .05
        for linear in (tw.linear.x,cmd.linear.x):
            for angular in (tw.angular.z,cmd.angular.z):
                v, w = linear, angular
                x,y,h = p.position.x,p.position.y,yaw(p.orientation)
                dt = .025
                for i in range(math.ceil((lag+max(abs(v)/2.5,abs(w)/3.2))/dt)+1):
                    margin = (abs(v)+math.sqrt(2)*self.half*abs(w))*dt
                    if not self.clear(x,y,h,margin):
                        return False
                    x += v*math.cos(h+w*dt/2)*dt
                    y += v*math.sin(h+w*dt/2)*dt
                    h += w*dt
                    if i*dt>=lag:
                        v = math.copysign(max(0.,abs(v)-2.5*dt),v)
                        w = math.copysign(max(0.,abs(w)-3.2*dt),w)
        return True

    def admissible_recovery_command(self, cmd, now):
        if (not self.govern_speed or self.validated_plan_stamp is None or
                self.validated_plan_stamp!=self.stamps.get('plan') or not self.plan_valid or
                self.hold_since is not None or not self.pose_path or now-self.stamps.get('rollout',-100.)>.15):
            return None
        if not all(self.clear(x,y,h,self.res/4) for x,y,h in self.interpolate(self.pose_path,self.half,self.res/2)):
            return None
        # Search the original command's curvature for a lower, braking-safe
        # velocity. Each candidate is independently validated with measured
        # odometry, since clearance need not be monotone during rotation.
        for divisor in (2,4,8,16):
            slower=Twist()
            slower.linear.x=cmd.linear.x/divisor
            slower.angular.z=cmd.angular.z/divisor
            if self.brake_corridor_clear(slower,now):
                return slower
        return None

    def step(self):
        now = self.now()
        # Grid publishes at 2 Hz, depth at 5 Hz. A missing stream closes the gate.
        limits = {'scan':.35,'depth':.5,'odom':.2,'grid':1.15,'command':.15}
        fresh = self.half is not None and all(-.05<=now-self.stamps.get(k,-100.)<=age for k,age in limits.items())
        cmd = self.request
        reason = 'NAVIGATING'
        if self.recovery:
            fresh = fresh and self.plan is not None and 0<=now-self.stamps.get('global',-100.)<=2.1
        if not fresh:
            reason = 'STALE_OR_IDLE'
            active_request=abs(cmd.linear.x)+abs(cmd.angular.z)>.001 and now-self.stamps.get('command',-100.)<=.15
            if self.recovery and self.state in ('NAVIGATING','BRAKING_LIMITED') and active_request and self.hold_since is None:
                self.hold_since=now
                self.rest_verified=False
                self.events.publish(String(data=json.dumps({'state':'OBSTACLE_DETECTED','sim_time':now,
                                                           'reason':'stale_telemetry'})))
        elif cmd.linear.x<0 or cmd.linear.y or not all(math.isfinite(v) for v in (cmd.linear.x,cmd.angular.z)):
            reason = 'UNSUPPORTED_COMMAND'
        elif (self.recovery and not self.plan_valid) or not self.brake_corridor_clear(cmd,now):
            slower=self.admissible_recovery_command(cmd,now) if self.recovery else None
            if slower is not None:
                cmd=slower
                reason='BRAKING_LIMITED'
            else:
                reason='SAFETY_STOP'
            if slower is None and self.hold_since is None:
                self.hold_since = now
                self.rest_verified=False
                if self.recovery:
                    self.events.publish(String(data=json.dumps({
                        'state':'OBSTACLE_DETECTED','sim_time':now,
                        'reason':'invalid_global_path' if not self.plan_valid else 'braking_envelope',
                        'requested':[cmd.linear.x, cmd.angular.z],
                        'measured':[self.odom.twist.twist.linear.x, self.odom.twist.twist.angular.z],
                        'grid_age_s':now-self.stamps['grid'],
                    })))
        elif self.hold_since is not None:
            stopped = abs(self.odom.twist.twist.linear.x)<.02 and abs(self.odom.twist.twist.angular.z)<.05
            self.rest_verified = self.rest_verified or stopped
            new_route = (not self.recovery or (self.plan_valid and self.stamps['plan']>self.hold_since
                         and self.stamps['global']>self.hold_since))
            updated = self.stamps['grid']>self.hold_since and self.rest_verified and new_route
            rollout_after=max(self.hold_since,self.stamps.get('plan',0.) if self.recovery else 0.)
            rollout_ok=(self.pose_path and self.stamps.get('rollout',0)>rollout_after
                        and now-self.stamps['rollout']<=.15
                        and all(self.clear(x,y,h,self.res/4) for x,y,h in self.interpolate(self.pose_path,self.half,self.res/2)))
            if updated and rollout_ok:
                self.validated_plan_stamp=self.stamps.get('plan')
                self.events.publish(String(data=json.dumps(dict(state='RESUMING',sim_time=now,
                    hold_since=self.hold_since,global_plan_stamp=self.stamps.get('plan'),
                    local_rollout_stamp=self.stamps['rollout']))))
                self.hold_since = None
            elif (self.recovery and updated and abs(cmd.linear.x)<1e-6 and abs(cmd.angular.z)>.001
                  and abs(self.odom.twist.twist.linear.x)<.02):
                # RotationShim must align the robot before DWB can publish a
                # translational rollout. The braking sweep above covers this
                # actual pure-rotation request, including the footprint corners.
                reason='ALIGNING_FOR_DWB'
            else:
                reason = 'WAITING_FOR_CLEAR_ROLLOUT'
        has_command = 0 <= now-self.stamps.get('command',-100.) <= .15
        if has_command or self.publishing:
            self.pub.publish(cmd if reason in ('NAVIGATING','ALIGNING_FOR_DWB','BRAKING_LIMITED') else Twist())
        self.publishing = has_command
        if reason != self.state:
            self.events.publish(String(data=json.dumps({'state':reason,'sim_time':now,'plan_valid':self.plan_valid,
                                      'ages':{k:round(now-v,3) for k,v in self.stamps.items()}})))
            self.state = reason


def main():
    rclpy.init()
    node = ObstacleGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
