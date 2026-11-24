"""Private velocity gate checks with a deterministic clock and map snapshots."""
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
pytest.importorskip("nav2_msgs.msg")
from geometry_msgs.msg import Twist

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts/validation'))
from generation_guard import GenerationGuard


def fixture():
    cmd=Twist()
    cmd.linear.x=.3
    out=[]
    g=SimpleNamespace(now=lambda:2.,request=cmd, stamps={k:2. for k in
         ('command','global','grid','odom','plan','rollout','scan','depth')},
         generation={'grid':2,'global':3},half=.36,approval=(1,2,1.,1.),
         planner_fence=True,planning=set(),plan_barrier_stamp=1.9,
         latched=False,hold_since=None,phase='NAVIGATING',publishing=True,
         plan_valid=True,clearance_recovery=False,rollout_valid=lambda _:True,
         brake_corridor_clear=lambda *_:True,pub=SimpleNamespace(publish=out.append),
         change=lambda *_:None)
    return g,out


@pytest.mark.parametrize("stream",['scan','depth','global','grid'])
def test_route_rejected_until_each_observation_covers_new_plan(stream):
    g,out=fixture()
    g.stamps[stream]=1.89
    GenerationGuard.step(g)
    assert out[-1].linear.x==0 and g.approval is None
    g.stamps[stream]=1.91
    GenerationGuard.step(g)
    assert out[-1].linear.x==.3 and g.approval==(2,3,2.,2.)


def test_changed_grid_invalidates_previous_command():
    g,out=fixture()
    GenerationGuard.step(g)
    g.generation['grid']+=1
    g.rollout_valid=lambda _:False
    GenerationGuard.step(g)
    assert out[-1].linear.x==0 and g.approval is None


@pytest.mark.parametrize("state",['latched','planning','expired_command'])
def test_no_forward_output(state):
    g,out=fixture()
    if state=='latched':
        g.latched=True
    elif state=='planning':
        g.planning.add(b'x')
    else:
        g.stamps['command']=1.7
    GenerationGuard.step(g)
    assert out[-1].linear.x==0


def test_rectangular_cell_axes_match_independent_configuration_space():
    import numpy as np
    from rectangular_guard import cell_intersects
    from clearance_oracle import clearance
    res=.05
    for yaw in (0.,.2,.7,1.5,3.14,-3.13):
        for center in ((.385,0),(.3851,0),(.3849,0),(.34,.36),(.01,.374),(-.42,-.02)):
            c=np.array(center)
            observed=cell_intersects(c[None,:],(0,0,yaw),(.36,.35),res,0.)
            expected=clearance([0,0,yaw],[*(c-res/2),*(c+res/2)]) <= 1e-12
            assert observed == expected


def test_braking_sweep_increases_with_map_age():
    from dynamic_guard import ObstacleGuard
    from nav_msgs.msg import Odometry
    cmd=Twist()
    cmd.linear.x=.5
    odom=Odometry()
    odom.pose.pose.orientation.w=1.
    g=SimpleNamespace(odom=odom, half=.36, stamps={'grid':1.99},
                      clear=lambda x,y,h,margin:x+margin<.45)
    assert ObstacleGuard.brake_corridor_clear(g,cmd,2.)
    g.stamps['grid']=1.45
    assert not ObstacleGuard.brake_corridor_clear(g,cmd,2.)
