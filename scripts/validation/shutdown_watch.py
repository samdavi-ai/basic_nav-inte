"""Observe the three private command streams across an owned launcher shutdown."""
import json
import sys
import time
from pathlib import Path
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def main(folder):
    rclpy.init()
    node=Node('gazebo_shutdown_observer')
    samples={k:[] for k in ('cmd_vel','cmd_vel_mux','cmd_vel_smoothed')}
    def observe(m,k):
        peak=max(abs(v) for vec in (m.linear,m.angular) for v in (vec.x,vec.y,vec.z))
        samples[k].append((time.monotonic(),peak))
    for k in samples:
        node.create_subscription(Twist,'/'+k,lambda m,k=k:observe(m,k),10)
    deadline=time.monotonic()+75
    armed=None
    gone=None
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.05)
        now=time.monotonic()
        if armed is None and all(seq and seq[-1][0]-seq[0][0]>=3 for seq in samples.values()):
            if any(v for seq in samples.values() for _,v in seq):
                break
            armed=now
            (folder/'shutdown.armed').touch(exist_ok=False)
        if armed is not None:
            if any(node.count_publishers('/'+k) for k in samples):
                gone=None
            elif gone is None:
                gone=now
            elif now-gone>=3:
                break
    peak={k:max((v for _,v in seq),default=None) for k,seq in samples.items()}
    record=dict(armed=armed,publishers_gone=gone,observed_until=time.monotonic(),peak=peak,
                passed=bool(armed and gone and time.monotonic()-gone>=3 and all(v==0 for v in peak.values())))
    (folder/'shutdown.json').write_text(json.dumps(record,indent=2)+'\n')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if record['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main(Path(sys.argv[1])))
