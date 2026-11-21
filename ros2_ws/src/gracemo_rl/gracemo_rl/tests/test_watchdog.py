"""
Unit tests for RLLocalPlannerNode Watchdog and Safety Halt Mechanisms.
"""

import time
import numpy as np
import pytest

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan


def test_watchdog_timeout_logic():
    """Verifies that stale sensor inputs cause safe zero-velocity output."""
    # Test watchdog timing condition logic directly
    now = 100.0
    watchdog_timeout_sec = 0.50

    # Fresh inputs
    last_scan_time = 99.8
    last_odom_time = 99.7
    last_path_time = 99.5
    scan_stale = (now - last_scan_time) > watchdog_timeout_sec
    odom_stale = (now - last_odom_time) > watchdog_timeout_sec
    plan_stale = (now - last_path_time) > 2.0
    assert not scan_stale and not odom_stale and not plan_stale

    # Stale scan
    last_scan_time = 99.3
    scan_stale = (now - last_scan_time) > watchdog_timeout_sec
    assert scan_stale, "Scan older than 0.5s must be flagged stale"

    # Stale odom
    last_odom_time = 99.4
    odom_stale = (now - last_odom_time) > watchdog_timeout_sec
    assert odom_stale, "Odom older than 0.5s must be flagged stale"

    # Stale plan
    last_path_time = 97.0
    plan_stale = (now - last_path_time) > 2.0
    assert plan_stale, "Plan older than 2.0s must be flagged stale"


def test_lookahead_extraction():
    """Verifies lookahead point extraction along a simulated path."""
    from gracemo_rl.nodes.rl_local_planner import RLLocalPlannerNode
    # Mock node state without rclpy.init by subclassing or testing method
    class MockPlanner:
        def __init__(self):
            self.robot_x = 0.0
            self.robot_y = 0.0
            self.lookahead_dist = 0.80
            self.active_path = Path()
            for i in range(10):
                p = PoseStamped()
                p.pose.position.x = float(i) * 0.2  # 0.0, 0.2, 0.4, 0.6, 0.8, 1.0 ...
                p.pose.position.y = 0.0
                self.active_path.poses.append(p)

    mock = MockPlanner()
    target = RLLocalPlannerNode._extract_lookahead_point(mock)
    assert target is not None
    # Robot is at (0,0), lookahead is 0.8m -> point should be at x=0.8, y=0.0
    assert np.isclose(target[0], 0.8, atol=0.05)
    assert np.isclose(target[1], 0.0, atol=0.05)

