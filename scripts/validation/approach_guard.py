"""Private Gazebo validation of lattice approaches and swept in-place turns."""
import math

import numpy as np
import rclpy
import tf2_ros
from rclpy.time import Time
from geometry_msgs.msg import Twist
from scipy.spatial import cKDTree
from dynamic_guard import yaw
from rectangular_guard import RectangularGuard


def path_fits_grid(poses, tree, origin, end, res):
    """Batched separating-axis test for all rectangle/cell pairs in the sweep."""
    margin = res/4
    c, s = np.cos(poses[:,2]), np.sin(poses[:,2])
    ex = np.abs(c)*.36 + np.abs(s)*.35 + margin
    ey = np.abs(s)*.36 + np.abs(c)*.35 + margin
    extent = np.column_stack((ex,ey))
    if np.any(poses[:,:2]-extent < origin) or np.any(poses[:,:2]+extent > end):
        return False
    # The expanded body axes give an enclosing radius even at diagonal contacts.
    radius = math.hypot(.36+margin,.35+margin) + res/math.sqrt(2) + 1e-10
    groups = tree.query_ball_point(poses[:,:2], radius)
    counts = np.fromiter(map(len, groups), dtype=int, count=len(groups))
    if not counts.any():
        return True
    idx = np.repeat(np.arange(len(poses)), counts)
    cells = np.concatenate([group for group in groups if group])
    dx, dy = (tree.data[cells] - poses[idx,:2]).T
    h = res/2
    projected = h*(np.abs(c[idx])+np.abs(s[idx])) + margin + 1e-12
    return not np.any((np.abs(dx) <= ex[idx]+h+1e-12) &
                      (np.abs(dy) <= ey[idx]+h+1e-12) &
                      (np.abs(c[idx]*dx+s[idx]*dy) <= .36+projected) &
                      (np.abs(-s[idx]*dx+c[idx]*dy) <= .35+projected))


class ApproachGuard(RectangularGuard):
    def __init__(self):
        super().__init__()
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.shim_settling_since = None
        self.planning_padding = self.declare_parameter("planning_padding", 0.).value
        self.rollout_ns = None
        self.await_rollout_since = None

    def on_grid(self, msg):
        previous = self.generation['grid']
        super().on_grid(msg)
        if self.generation['grid'] != previous:
            self.local_tree = cKDTree(self.obstacles)

    def validate_plan(self):
        self.plan_valid = False
        if self.plan is None or self.global_grid is None or self.half is None:
            return
        tangent = self.plan.copy()
        dx,dy = np.diff(tangent[:,:2],axis=0).T
        tangent[:-1,2] = np.where(np.hypot(dx,dy)>1e-7,np.arctan2(dy,dx),tangent[:-1,2])
        res = self.global_grid[-1]
        self.plan_valid = all(path_fits_grid(self.interpolate(path,self.half,res/2),
                                             *self.global_grid) for path in (self.plan,tangent))

    def on_rollout(self, msg):
        super().on_rollout(msg)
        if msg.header.frame_id == 'odom' and msg.poses:
            self.rollout_ns = Time.from_msg(msg.header.stamp).nanoseconds

    def rollout_valid(self, now):
        if self.rollout_ns is None or not self.pose_path:
            return False
        age_ns = round(now * 1_000_000_000) - self.rollout_ns
        if not 0 <= age_ns <= 150_000_000:
            return False
        return path_fits_grid(self.interpolate(self.pose_path,self.half,self.res/2),
                              self.local_tree,self.origin,self.extent,self.res)

    def step(self):
        now = self.now()
        shim = abs(self.request.linear.x) < 1e-6 and abs(self.request.angular.z) > .001
        fresh = all(0 <= now-self.stamps.get(key, -100) <= age for key, age in
                    [('command', .15), ('odom', .2), ('scan', .35), ('depth', .5),
                     ('global', 2.1), ('grid', 1.15)])
        # A missing DWB publication is not evidence that a replacement global
        # route is needed. Stop while waiting for one publication window. On
        # arrival, the normal guard still checks the complete new trajectory.
        if (self.hold_since is None and not self.latched and fresh and self.plan_valid
                and self.request.linear.x > 0 and
                (self.rollout_ns is None or round(now*1e9)-self.rollout_ns > 150_000_000)):
            if self.await_rollout_since is None:
                self.await_rollout_since = now
            if now-self.await_rollout_since < .15:
                self.approval = None
                self.pub.publish(Twist())
                self.publishing = True
                return
        else:
            self.await_rollout_since = None
        if (self.hold_since is None and not self.latched and shim and fresh
                and self.plan_valid and self.odom is not None
                and abs(self.odom.twist.twist.linear.x) >= .02):
            if self.shim_settling_since is None:
                self.shim_settling_since = now
            # DWB has no rollout while RotationShim owns a pure turn. Command
            # zero during drift. Persistent drift takes the usual recovery path.
            if now-self.shim_settling_since < .15:
                self.approval = None
                self.pub.publish(Twist())
                self.publishing = True
                return
        else:
            self.shim_settling_since = None
        super().step()

    def clear(self, x, y, heading, margin):
        # Keep a stopped body inside the footprint envelope used by the
        # recovery planner. Both local rectangle axes expand by the planner
        # padding, so a corner can move sqrt(2) times that distance.
        return super().clear(x, y, heading, margin + math.sqrt(2)*self.planning_padding)

    def turn_clear(self, command):
        pose = self.odom.pose.pose
        x, y = pose.position.x, pose.position.y
        heading = yaw(pose.orientation)
        try:
            tf = self.tf_buffer.lookup_transform('odom','map',
                    Time.from_msg(self.odom.header.stamp))
        except tf2_ros.TransformException:
            return False
        rotation = yaw(tf.transform.rotation)
        offset = tf.transform.translation
        c, s = math.cos(rotation), math.sin(rotation)
        points = self.plan[:,:2] @ np.array([[c,s],[-s,c]]) + [offset.x,offset.y]
        distance = np.linalg.norm(points-[x,y],axis=1)
        nearest = int(np.argmin(distance))
        forward = np.flatnonzero(distance[nearest:] >= .8)
        if len(forward):
            dx,dy = points[nearest+forward[0]] - [x,y]
            target = math.atan2(dy,dx)
        else:
            target = rotation+self.plan[-1,2]
        delta = math.remainder(target-heading,2*math.pi)
        # A shim can choose a different heading during goal alignment. Cover
        # the full directed half-turn in that case, until a new twist arrives.
        if delta*command.angular.z < 0:
            delta = math.copysign(math.pi, command.angular.z)
        poses = self.interpolate([[x,y,heading],[x,y,heading+delta]],self.half,self.res/2)
        return all(self.clear(px,py,h,self.res/4) for px,py,h in poses)

    def brake_corridor_clear(self, cmd, now):
        if abs(cmd.linear.x)<1e-6 and abs(cmd.angular.z)>.001 and not self.turn_clear(cmd):
            return False
        return super().brake_corridor_clear(cmd,now)


if __name__=='__main__':
    rclpy.init()
    node=ApproachGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
