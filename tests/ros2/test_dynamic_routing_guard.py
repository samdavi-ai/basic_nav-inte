"""Geometry and veto regressions for the private Gazebo validation profile."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

pytest.importorskip("nav2_msgs.msg")
from geometry_msgs.msg import Twist
from scipy.spatial import cKDTree

file=Path(__file__).resolve().parents[2]/"scripts/validation/dynamic_guard.py"
spec=importlib.util.spec_from_file_location("fixture_guard", file)
guard=importlib.util.module_from_spec(spec)
spec.loader.exec_module(guard)


def test_full_footprint_corner_can_invalidate_center_clear_path():
    # A rotating footprint sweeps its corner over a cell between two sparse poses.
    fixture=SimpleNamespace(half=.36, global_grid=(cKDTree([[.47,0.]]),np.array([-2.,-2.]),
                            np.array([2.,2.]),.05),plan=np.array([[0.,0.,0.],[0.,0.,np.pi/2]]),
                            interpolate=guard.ObstacleGuard.interpolate)
    guard.ObstacleGuard.validate_plan(fixture)
    assert not fixture.plan_valid
    fixture.global_grid=(cKDTree([[.8,0.]]),*fixture.global_grid[1:])
    guard.ObstacleGuard.validate_plan(fixture)
    assert fixture.plan_valid


def test_stale_stream_zeroes_pending_forward_command_and_releases_idle_mux():
    cmd=Twist()
    cmd.linear.x=.5
    published=[]
    f=SimpleNamespace(recovery=False,half=.36,request=cmd,stamps=dict(odom=2.,grid=2.,scan=2.,depth=0.,command=2.),
                      plan_valid=False,now=lambda:2.,pub=SimpleNamespace(publish=published.append),publishing=True,state=None,
                      events=SimpleNamespace(publish=lambda _:None))
    guard.ObstacleGuard.step(f)
    assert published[-1].linear.x==0 and f.state=='STALE_OR_IDLE'
    f.now=lambda:3.
    guard.ObstacleGuard.step(f)
    guard.ObstacleGuard.step(f)
    assert len(published)==2 and not f.publishing


def test_governor_preserves_curvature_and_requires_current_validated_route():
    cmd=Twist()
    cmd.linear.x=.5
    cmd.angular.z=.25
    state=SimpleNamespace(govern_speed=True,validated_plan_stamp=1.,stamps={'plan':1.,'rollout':2.},
                          plan_valid=True,hold_since=None,pose_path=[(0.,0.,0.)],half=.36,res=.05,
                          interpolate=guard.ObstacleGuard.interpolate,clear=lambda *_:True,
                          brake_corridor_clear=lambda twist, _:twist.linear.x<=.15)
    slower=guard.ObstacleGuard.admissible_recovery_command(state,cmd,2.05)
    assert slower.linear.x==.125 and slower.angular.z==.0625
    state.stamps['plan']=2.
    assert guard.ObstacleGuard.admissible_recovery_command(state,cmd,2.05) is None
