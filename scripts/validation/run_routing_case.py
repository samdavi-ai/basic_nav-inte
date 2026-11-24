#!/usr/bin/env python3
"""Run a single headless Nav2/Gazebo routing fixture on a private simulation graph."""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
import uuid
from pathlib import Path


def spawn_fixture(box, movable=False, name='routing_fixture'):
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.empty_pb2 import Empty
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.scene_pb2 import Scene
    from gz.transport13 import Node

    x, y, width, depth = box
    geometry = f'<geometry><box><size>{width} {depth} 1</size></box></geometry>'
    control = ('<plugin filename="gz-sim-velocity-control-system" name="gz::sim::systems::VelocityControl">'
               '<topic>/fixture_driver/box_velocity</topic></plugin>') if movable else ''
    height = .505 if movable else .5
    model = (f'<sdf version="1.9"><model name="{name}"><pose>{x} {y} {height} 0 0 0</pose>'
             f'<static>{str(not movable).lower()}</static><link name="body"><gravity>false</gravity><collision name="collision">{geometry}</collision>'
             f'<visual name="visual">{geometry}</visual></link>{control}</model></sdf>')
    node = Node()
    ok, result = node.request('/world/gracemo_home/create', EntityFactory(sdf=model), EntityFactory, Boolean, 5000)
    if ok and not result.data:
        raise RuntimeError("Gazebo rejected the fixture")
    # A transport timeout can occur after insertion. Confirm the actual scene
    # before considering the request failed; never create a second model.
    for _ in range(20):
        ok, scene = node.request('/world/gracemo_home/scene/info', Empty(), Empty, Scene, 1000)
        if ok:
            for model in scene.model:
                p=model.pose.position
                if model.name==name and abs(p.x-x)<1e-5 and abs(p.y-y)<1e-5:
                    return dict(x=p.x,y=p.y,z=p.z)
        time.sleep(.2)
    raise RuntimeError("Fixture pose not confirmed by Gazebo")


def stop(proc, sig):
    if proc is None:
        return
    if proc.poll() is None:
        proc.send_signal(sig)
        try:
            proc.wait(15)
        except subprocess.TimeoutExpired:
            pass
    # All callers create a new session. The gz wrapper can exit before its
    # renderer child. Tear down that owned process group even after leader exit.
    for sig in (signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(proc.pid,sig)
        except ProcessLookupError:
            break
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            proc.poll()
            try:
                os.killpg(proc.pid,0)
            except ProcessLookupError:
                return
            time.sleep(.05)
    proc.wait(timeout=3)


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('out', type=Path)
    cli.add_argument('--box', nargs=4, type=float, required=True, metavar=('X','Y','WIDTH','DEPTH'))
    cli.add_argument('--goal', nargs=2, type=float, default=[4.5,0.])
    cli.add_argument('--during-navigation', action='store_true', help='Insert the box after DWB receives the original route')
    cli.add_argument('--settle', type=float, default=3., help='Seconds after fixture insertion before dispatch')
    args = cli.parse_args()
    args.out.mkdir(parents=True, exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(100 + os.getpid()%90), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      GZ_IP='127.0.0.1', GZ_PARTITION='vira-routing-'+uuid.uuid4().hex,
                      GZ_CONFIG_PATH='/usr/share/gz')
    (args.out/'fixture.json').write_text(json.dumps(dict(box=args.box,goal=args.goal,settle_s=args.settle,during_navigation=args.during_navigation))+'\n')
    logfile = args.out/'launch.log'
    recorder = mission = None
    with logfile.open('x') as log:
        launch = subprocess.Popen(['ros2','launch','gracemo_gazebo','vira_complete.launch.py',
                  'mode:=localization','headless:=true','use_rviz:=false','launch_interaction:=false',
                  'kernel_url:=http://127.0.0.1:17787'], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic()+55
            while 'Managed nodes are active' not in logfile.read_text(errors='replace') or 'Activating collision_monitor' not in logfile.read_text(errors='replace'):
                if launch.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError("Gazebo navigation startup failed")
                time.sleep(.25)
            if not args.during_navigation:
                spawn_fixture(args.box)
            recorder = subprocess.Popen([sys.executable,'scripts/validation/capture_routing.py',str(args.out/'trace.jsonl.gz')],start_new_session=True)
            time.sleep(args.settle)
            mission = subprocess.Popen([sys.executable,'scripts/run_gazebo_navigation_smoke.py','--goal-x',
                    str(args.goal[0]),'--goal-y',str(args.goal[1]),'--goal-timeout','115',
                    '--out',str(args.out/'mission.json')], start_new_session=True)
            if args.during_navigation:
                deadline = time.monotonic()+45
                while 'Received a goal, begin computing control effort' not in logfile.read_text(errors='replace'):
                    if mission.poll() is not None or time.monotonic()>deadline:
                        raise RuntimeError("No active controller goal for obstacle insertion")
                    time.sleep(.05)
                spawn_fixture(args.box)
            return mission.wait(timeout=180)
        finally:
            stop(mission, signal.SIGTERM)
            stop(recorder, signal.SIGTERM)
            stop(launch, signal.SIGINT)


if __name__ == '__main__':
    sys.exit(main())
