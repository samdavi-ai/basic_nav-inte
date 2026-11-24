import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'scripts/validation'))
from clearance_oracle import clearance, sweep, RADIUS


@pytest.mark.parametrize("gap", [-.0001, 0., .0001])
def test_signed_edge_distance(gap):
    assert clearance([0,0,0],[.36+gap,-1,1,1]) == pytest.approx(gap,abs=1e-12)


def test_corner_distance():
    assert clearance([0,0,0],[.39,.39,1,1]) == pytest.approx(.05)


def test_rotation_between_endpoints():
    r=sweep(np.array([[0.,0,0,0],[1.,0,0,math.pi/2]]),[.44,-1,1,1])
    assert r['raw_min_m']>0
    assert r['lower_m'] <= .44-RADIUS <= r['upper_m'] + 1e-12


def test_angle_wrap_avoids_spurious_half_turn():
    r=sweep(np.array([[1.,0,0,math.pi-.001],[2.,0,0,-math.pi+.001]]),[.4,-1,1,1])
    assert r['lower_m']>.039


def test_rejects_clock_reset():
    with pytest.raises(ValueError):
        sweep(np.zeros((2,4)),[0,0,1,1])
