"""The Gazebo depth bytes must agree with the optical TF used by both costmaps."""
import struct
import sys
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest

pytest.importorskip('rclpy')
pytest.importorskip('gz.transport13')
from builtin_interfaces.msg import Time
from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked

sys.path.insert(0, str(Path(__file__).resolve().parents[2]/'ros2_ws/src/gracemo_bridge'))
from gracemo_bridge import sensor_bridge
from gracemo_bridge.sensor_bridge import SensorBridgeNode


def test_optical_axes_and_self_exclusion_preserve_packed_fields():
    # Forward external object, near body reflection, and an invalid depth.
    points = [(1.,.2,-.3), (.1,0.,-.1), (np.nan,np.nan,np.nan)]
    tail = b'abcdRGB!paddingbytes'
    rows = b''.join(struct.pack('<fff', *point)+tail for point in points)
    step = len(rows)//len(points)
    msg = PointCloudPacked(width=3,height=1,point_step=step,row_step=len(rows),data=rows,is_dense=False)
    pub = Mock()
    pub.get_subscription_count.return_value = 1
    bridge = SimpleNamespace(pub_depth_points=pub,_last_pc_time=0.,_to_ros_time=lambda _:Time(),pc_fields=[])
    SensorBridgeNode._on_gz_depth_points(bridge,msg)
    cloud = pub.publish.call_args.args[0]
    assert cloud.header.frame_id == 'camera_optical_link'
    assert not cloud.is_dense
    for i in range(3):
        xyz = struct.unpack_from('<fff',cloud.data,step*i)
        if i==0:
            np.testing.assert_allclose(xyz,[-.2,.3,1.])
        else:
            assert np.isnan(xyz).all()
        assert bytes(cloud.data[i*step+12:(i+1)*step]) == tail


@pytest.mark.parametrize("context_active", [True, False])
def test_spin_error_only_ignored_after_shutdown(monkeypatch, context_active):
    ros = Mock()
    ros.executors.ExternalShutdownException = type("ExternalShutdownException", (Exception,), {})
    ros.spin.side_effect = sensor_bridge.rclpy_implementation.RCLError("invalid wait set")
    node = Mock()
    node.context.ok.return_value = context_active
    monkeypatch.setattr(sensor_bridge, "rclpy", ros)
    monkeypatch.setattr(sensor_bridge, "SensorBridgeNode", lambda: node)
    if context_active:
        with pytest.raises(sensor_bridge.rclpy_implementation.RCLError):
            sensor_bridge.main()
    else:
        sensor_bridge.main()
    node.shutdown_gazebo_transport.assert_called_once()
    node.destroy_node.assert_called_once()
