"""Analytic reference cases for the footprint acceptance calculation."""
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'scripts/validation'))
from analyze_routing import HALF, RADIUS, separation, sweep


def test_separation_handles_edges_corners_and_containment():
    box = ('fixture',0.,0.,2.,2.)
    poses = np.array([[1.4,0,0], [1.66,1.75,0], [0,0,0], [1.30,0,0]])
    np.testing.assert_allclose(separation(poses,box), [.04,.50,-1.35,-.06], atol=1e-12)


def test_rotation_sweep_checks_between_clear_endpoints():
    box = ('wall',.45,0.,.1,1.)
    poses = np.array([[0.,0.,0.], [0.,0.,np.pi/2]])
    assert min(separation(poses,box))>0
    result = sweep(poses,[box])
    exact = .4-RADIUS
    assert result['swept_lower_bound_m'] <= exact <= result['sample_min_m']+1e-6


def test_yaw_wrap_uses_short_turn():
    box = ('wall',.45,0.,.1,1.)
    poses = [[0.,0.,np.pi-.01], [0.,0.,-np.pi+.01]]
    assert sweep(poses,[box])['swept_lower_bound_m'] > 0
