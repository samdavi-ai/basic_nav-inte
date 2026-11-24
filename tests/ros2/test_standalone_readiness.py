"""Cold-start regression: ROS action endpoints appear before Nav2 accepts goals."""
import threading
from concurrent.futures import Future
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

pytest.importorskip("std_srvs.srv")
from scripts.run_autonomous_mission import MissionMonitor


def response(active):
    future = Future()
    future.set_result(SimpleNamespace(success=active))
    return future


def test_inactive_manager_blocks_dispatch_until_active():
    client = Mock()
    client.call_async.side_effect = [response(False), response(True)]
    monitor = SimpleNamespace(node=SimpleNamespace(nav_activity=client), ready=threading.Event())
    monitor.ready.set()
    assert MissionMonitor.wait_until_ready(monitor, 2)
    assert client.call_async.call_count == 2


def test_lifecycle_response_cannot_override_lost_coordinator_readiness():
    client = Mock()
    monitor = SimpleNamespace(node=SimpleNamespace(nav_activity=client), ready=threading.Event())
    monitor.ready.set()

    def lose_readiness(_request):
        monitor.ready.clear()
        return response(True)

    client.call_async.side_effect = lose_readiness
    assert not MissionMonitor.wait_until_ready(monitor, 0.05)
