"""Boundary regression from the private Gazebo guard transition at 22.05 s."""
import sys
from pathlib import Path
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'scripts/validation'))
from approach_guard import ApproachGuard, path_fits_grid
from rectangular_guard import cell_intersects
import numpy as np
from scipy.spatial import cKDTree


def test_rollout_exact_jazzy_timestamps():
    guard = SimpleNamespace(rollout_ns=21900000000, pose_path=[(0, 0, 0)],
                            local_tree=cKDTree([[1.,1.]]),origin=np.array([-2.,-2.]),
                            extent=np.array([2.,2.]),half=.36,res=.05,
                            interpolate=lambda points, *_: np.asarray(points))
    assert ApproachGuard.rollout_valid(guard, 22.05)
    assert not ApproachGuard.rollout_valid(guard, 22.050000001)
    assert not ApproachGuard.rollout_valid(guard, 21.899999999)
    guard.local_tree = cKDTree([[0.,0.]])
    assert not ApproachGuard.rollout_valid(guard, 22.05)


def test_drift_pause_commands_zero_and_expires(monkeypatch):
    from geometry_msgs.msg import Twist
    from rectangular_guard import RectangularGuard

    calls = []
    monkeypatch.setattr(RectangularGuard, 'step', lambda self: calls.append('normal guard'))
    guard = object.__new__(ApproachGuard)
    guard.request = Twist()
    guard.request.angular.z = -.4
    guard.stamps = {key: 1. for key in ('command','odom','scan','depth','global','grid')}
    guard.odom = SimpleNamespace(twist=SimpleNamespace(twist=Twist()))
    guard.odom.twist.twist.linear.x = .0212
    guard.pub = SimpleNamespace(publish=calls.append)
    guard.approval = ('old approval',)
    guard.plan_valid = True
    guard.hold_since = guard.shim_settling_since = None
    guard.latched = False
    guard.now = lambda: 1.
    guard.step()
    assert calls == [Twist()]
    assert guard.approval is None
    guard.stamps = dict.fromkeys(guard.stamps, 1.16)
    guard.now = lambda: 1.16
    guard.step()
    assert calls[-1] == 'normal guard'
    guard.shim_settling_since = None
    guard.latched = True
    guard.step()
    assert calls[-1] == 'normal guard'
    assert guard.shim_settling_since is None


def test_batched_grid_matches_exhaustive_rectangle_test():
    rng = np.random.default_rng(20261122)
    for heading in np.linspace(-np.pi,np.pi,41):
        poses = np.array([[0.,0.,heading],[.1,-.02,heading+.1]])
        for offset in [0.,.4,.6]:
            cells = rng.uniform(-.1,.1,(8,2))+offset
            expected = not any(cell_intersects(cells,pose,(.36,.35),.05,.0125) for pose in poses)
            assert path_fits_grid(poses,cKDTree(cells),np.array([-1,-1]),np.array([1,1]),.05) == expected


def test_missing_translating_rollout_holds_zero_for_bounded_time(monkeypatch):
    from geometry_msgs.msg import Twist
    from rectangular_guard import RectangularGuard
    calls = []
    monkeypatch.setattr(RectangularGuard, 'step', lambda self: calls.append('normal guard'))
    guard = object.__new__(ApproachGuard)
    guard.request = Twist()
    guard.request.linear.x = .3
    guard.stamps = dict.fromkeys(('command','odom','scan','depth','global','grid'), 1.)
    guard.now = lambda: 1.
    guard.hold_since = guard.await_rollout_since = guard.rollout_ns = None
    guard.latched = False
    guard.plan_valid = True
    guard.pub = SimpleNamespace(publish=calls.append)
    guard.step()
    assert calls == [Twist()] and guard.approval is None
    guard.now = lambda: 1.16
    guard.stamps = dict.fromkeys(guard.stamps,1.16)
    guard.step()
    assert calls[-1] == 'normal guard'


def test_braking_reserves_global_planner_padding():
    from rectangular_guard import RectangularGuard
    guard = object.__new__(ApproachGuard)
    guard.origin = np.array([-1., -1.])
    guard.extent = np.array([1., 1.])
    guard.res = .05
    guard.obstacles = np.array([[.42, 0.]])
    guard.planning_padding = .05 * (2**-.5 + .25)
    assert RectangularGuard.clear(guard, 0., 0., 0., 0.)
    assert not guard.clear(0., 0., 0., 0.)
    assert guard.clear(-.1, 0., 0., 0.)
