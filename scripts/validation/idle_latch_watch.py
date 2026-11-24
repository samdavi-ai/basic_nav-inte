"""Passive three-topic Gazebo idle check across a latched stop and explicit reset."""
import json
import sys
import time
from pathlib import Path
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


def main():
    rclpy.init()
    node=Node('gazebo_idle_latch_watch')
    samples={key:[] for key in ('cmd_vel','cmd_vel_mux','cmd_vel_smoothed','source')}
    transitions={}
    def record(command, key):
        peak=max(abs(v) for vec in (command.linear,command.angular) for v in (vec.x,vec.y,vec.z))
        samples[key].append((time.monotonic(),peak))
    for key in ('cmd_vel','cmd_vel_mux','cmd_vel_smoothed'):
        node.create_subscription(Twist,'/'+key,lambda m,k=key:record(m,k),10)
    node.create_subscription(String,'/cmd_vel_mux/status',lambda m:samples['source'].append((
        time.monotonic(),json.loads(m.data).get('active_source')!='NONE')),10)
    def estop(m):
        if m.data:
            transitions.setdefault('latch',time.monotonic())
        elif 'latch' in transitions:
            transitions.setdefault('reset',time.monotonic())
    node.create_subscription(Bool,'/terrain/emergency_stop',estop,10)
    deadline=time.monotonic()+110
    while time.monotonic()<deadline and time.monotonic()<transitions.get('reset',deadline)+3.1:
        rclpy.spin_once(node,timeout_sec=.05)
    result=dict(events=transitions)
    for label,stop in [('latched',transitions.get('reset')),('after_reset',None)]:
        if 'reset' not in transitions: continue
        start=stop-3 if stop else transitions['reset']
        if start<transitions['latch']: continue
        windows={k:[v for t,v in seq if start<=t<=start+3] for k,seq in samples.items()}
        result[label]=dict(samples={k:len(v) for k,v in windows.items()},
                           peak={k:max(v,default=None) for k,v in windows.items()},
                           passed=all(len(v)>=3 and max(v)==0 for v in windows.values()))
    Path(sys.argv[1]).write_text(json.dumps(result,indent=2)+'\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__=='__main__':
    main()
