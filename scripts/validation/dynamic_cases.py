#!/usr/bin/env python3
"""Deterministic Gazebo-only obstacle campaign. Run one case on a private graph."""
import argparse
import fcntl
import json
import os
import random
import shutil
import signal
import tempfile
import subprocess
import sys
import time
import uuid
from pathlib import Path

from run_routing_case import spawn_fixture, stop


def scenario(index, seed, level=1):
    rng = random.Random((seed << 16) + index)
    kind = ('front', 'beside', 'crossing', 'doorway', 'replan')[index % 5]
    sign = 1 if index % 2 else -1
    w = round(rng.uniform(.27, .38), 3)
    x = {'front': 2., 'beside': 2., 'crossing': 1.9, 'doorway': 4.98, 'replan': 4.37}[kind]
    y = {'front': .40*sign, 'beside': .69*sign, 'crossing': .67*sign,
         'doorway': 1.05, 'replan': .4*sign}[kind]
    cfg = dict(index=index, seed=seed, kind=kind, box=[x,y,w,.35],
                goal=[4.5,3. if kind=='doorway' else 0.],
                min_robot_x=3.3 if kind=='doorway' else rng.uniform(.12,.34),
                min_plan_count=2 if kind=='replan' else 1,
                crossing_speed=round(rng.uniform(.25,.45),3) if kind=='crossing' else 0.,
                initial_pose=[0,0,0], world='apartment_floor.world',level=level)
    if level>1:
        cfg['staging_goal']=[round(rng.uniform(.35,.7),3),round(rng.uniform(-.18,.18),3)]
        cfg['goal'][0]+=round(rng.uniform(-.25,.3),3)
        cfg['goal'][1]+=round(rng.uniform(-.09,.09),3)
        cfg['min_robot_x']+=cfg['staging_goal'][0]
        cfg['box'][0]+=round(rng.uniform(-.12,.12),3)
        cfg['box'][3]=round(rng.uniform(.25,.42),3)
        cfg['fixed_geometry']=[
            ['corridor_liner',2.2,.9,2.,round(rng.uniform(.06,.2),3)],
            ['door_jamb',5.6,1.,round(rng.uniform(.08,.22),3),.2]]
    return cfg


def inject(folder, cfg):
    import rclpy
    from geometry_msgs.msg import Twist
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.msgs10.twist_pb2 import Twist as FixtureTwist
    from rosgraph_msgs.msg import Clock
    import threading
    from gz.transport13 import Node as GzNode
    from nav_msgs.msg import Odometry, Path as Plan
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data

    if not os.environ.get('GZ_PARTITION','').startswith('vira-dynamic-'):
        raise ValueError('A private dynamic-case Gazebo partition is required')
    rclpy.init()
    node = Node('gazebo_fixture_driver')
    gz = GzNode()
    observed = []
    def observe(m):
        for item in m.pose:
            if item.name == 'routing_fixture':
                pos = item.position
                observed.append(dict(t=m.header.stamp.sec+m.header.stamp.nsec*1e-9,
                                     x=pos.x, y=pos.y, z=pos.z))
                break
    if cfg['crossing_speed']:
        gz.subscribe(Pose_V, '/world/gracemo_home/dynamic_pose/info', observe)
    live = dict(t=0., x=0., v=0., plans=0)
    node.create_subscription(Clock, '/clock', lambda m: live.update(t=m.clock.sec+m.clock.nanosec*1e-9), qos_profile_sensor_data)
    node.create_subscription(Plan,'/plan',lambda _:live.update(plans=live['plans']+1),10)
    node.create_subscription(Odometry,'/odom',lambda m:live.update(x=m.pose.pose.position.x),qos_profile_sensor_data)
    node.create_subscription(Twist,'/cmd_vel',lambda m:live.update(v=m.linear.x),10)
    deadline=time.monotonic()+110
    while not (live['t'] and live['x']>=cfg['min_robot_x'] and live['v']>.02
               and live['plans']>=cfg['min_plan_count']
               and (not cfg.get('synchronize_replan') or (folder/'replan.entered').exists())):
        if cfg.get('synchronize_replan') and (folder/'replan.exited').exists():
            raise RuntimeError('Second planner invocation exited before fixture qualification')
        if time.monotonic()>deadline:
            raise TimeoutError('No qualifying motion/plan for fixture insertion')
        rclpy.spin_once(node,timeout_sec=.05)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    with (folder/'world.jsonl').open('x',buffering=1) as f:
        box=cfg['box']
        start=live['t']
        f.write(json.dumps(dict(request=start, robot_x=live['x'], velocity=live['v'], plans=live['plans']))+'\n')
        result=subprocess.run([sys.executable,__file__,str(folder),"--spawn"], check=True, timeout=35,
                              capture_output=True,text=True)
        actual=json.loads(result.stdout.splitlines()[-1])
        f.write(json.dumps(dict(t=live['t'],**actual))+'\n')
        if cfg.get('temporary'):
            from gz.msgs10.boolean_pb2 import Boolean
            from gz.msgs10.entity_pb2 import Entity
            from gz.msgs10.empty_pb2 import Empty
            from gz.msgs10.scene_pb2 import Scene
            remove_at = live['t'] + 2.
            deadline = time.monotonic() + 20
            while live['t'] < remove_at and time.monotonic() < deadline:
                time.sleep(.02)
            if live['t'] < remove_at:
                raise TimeoutError('Simulation clock stalled during temporary blockage')
            ok, reply = gz.request('/world/gracemo_home/remove',
                                   Entity(name='routing_fixture', type=Entity.MODEL),
                                   Entity, Boolean, 2000)
            if not ok or not reply.data:
                raise RuntimeError('Temporary fixture removal failed')
            for _ in range(20):
                ok, scene = gz.request('/world/gracemo_home/scene/info', Empty(), Empty, Scene, 1000)
                if ok and all(model.name != 'routing_fixture' for model in scene.model):
                    (folder/'removal.json').write_text(json.dumps({'confirmed_absent':live['t']}))
                    break
                time.sleep(.05)
            else:
                raise RuntimeError('Temporary fixture still present')
        if cfg.get('synchronize_replan'):
            if (folder/'replan.exited').exists():
                raise RuntimeError('Second planner invocation exited before scene confirmation')
            (folder/'fixture.confirmed').write_text(str(time.monotonic_ns()))
        speed=cfg['crossing_speed']
        if speed:
            direction=-1 if box[1]>0 else 1
            # This topic belongs exclusively to the spawned box model.
            control=gz.advertise('/fixture_driver/box_velocity',FixtureTwist)
            moving=FixtureTwist()
            moving.linear.y=direction*speed
            until=time.monotonic()+30
            try:
                while not observed or direction*(observed[-1]['y']+box[1])<-.01:
                    if time.monotonic()>until:
                        raise TimeoutError('Crossing fixture failed to reach its endpoint')
                    control.publish(moving)
                    time.sleep(.02)
            finally:
                for _ in range(5):
                    control.publish(FixtureTwist())
                    time.sleep(.02)
            for sample in observed:
                f.write(json.dumps(sample)+'\n')
            if not observed or abs(observed[-1]['y']+box[1])>.03:
                raise RuntimeError('Crossing endpoint not verified by Gazebo physics poses')
    if cfg['crossing_speed']:
        gz.unsubscribe('/world/gracemo_home/dynamic_pose/info')
    rclpy.shutdown()
    spin_thread.join()
    node.destroy_node()


REGRESSIONS = {
    'original': ([2.,-.5,.35,.35],[4.5,0.]),
    'mirrored': ([2.35,.5,.30,.40],[4.5,0.]),
    'larger': ([2.6,-.52,.45,.40],[4.5,0.]),
    'study': ([4.15,2.25,.45,.45],[4.5,3.]),
    'blocked': ([2.,0.,.35,1.65],[4.5,0.]),
}


def run(folder,cfg, guard=False, recovery=False, mission_timeout=115, regression=None):
    folder.mkdir(parents=True,exist_ok=False)
    (folder/'scenario.json').write_text(json.dumps(cfg,indent=2)+'\n')
    os.environ.update(ROS_DOMAIN_ID=str(105+os.getpid()%85), GZ_PARTITION='vira-dynamic-'+uuid.uuid4().hex,
                      ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', GZ_IP='127.0.0.1', GZ_CONFIG_PATH='/usr/share/gz')
    overlay = tempfile.TemporaryDirectory(prefix='vira-dynamic-profile-')
    state_dir = Path(overlay.name)/'state'
    state_dir.mkdir(mode=0o700)
    os.environ['GRACEMO_STATE_DIR']=str(state_dir)
    if cfg.get('planner_fence'):
        os.environ['VIRA_PLANNER_FENCE']='1'
    if cfg.get('dense_truth'):
        import xml.etree.ElementTree as ET
        from ament_index_python.packages import get_package_prefix
        gz_prefix=Path(overlay.name)/'gracemo_gazebo'
        shutil.copytree(get_package_prefix('gracemo_gazebo'),gz_prefix)
        world=gz_prefix/'share/gracemo_gazebo/worlds/apartment_floor.world'
        tree=ET.parse(world)
        broadcaster=tree.find(".//world/plugin[@name='gz::sim::systems::SceneBroadcaster']")
        ET.SubElement(broadcaster,'dynamic_pose_hertz').text='250'
        tree.write(world)
        os.environ['AMENT_PREFIX_PATH']=str(gz_prefix)+os.pathsep+os.environ['AMENT_PREFIX_PATH']
    if guard or recovery or cfg.get('synchronize_replan') or cfg.get('doorway_trial'):
        from ament_index_python.packages import get_package_prefix
        import yaml
        target = Path(overlay.name) / 'gracemo_nav2'
        shutil.copytree(get_package_prefix('gracemo_nav2'), target)
        os.environ['AMENT_PREFIX_PATH'] = str(target)+os.pathsep+os.environ['AMENT_PREFIX_PATH']
        config = target/'share/gracemo_nav2/config/nav2_params.yaml'
        params = yaml.safe_load(config.read_text())
        if cfg.get('turning_corridor'):
            import xml.etree.ElementTree as ET
            planner=params['planner_server']['ros__parameters']
            planner['planner_plugins'].append('TurningClearance')
            planner['TurningClearance']=dict(plugin='vira_validation::ClearanceNavfn',allow_unknown=False,tolerance=.05)
            if cfg.get('goal_budget'):
                controller=params['controller_server']['ros__parameters']
                cell=params['global_costmap']['global_costmap']['ros__parameters']['resolution']
                budget=controller['general_goal_checker']['xy_goal_tolerance']
                planner['TurningClearance']['tolerance']=budget-cell
                controller['general_goal_checker']['xy_goal_tolerance']=cell
                controller['FollowPath']['xy_goal_tolerance']=cell
            tree_path=target/'share/gracemo_nav2/behavior_trees/navigate_to_pose_gracemo.xml'
            tree=ET.parse(tree_path)
            tree.find('.//PlannerSelector').set('default_planner','TurningClearance')
            tree.write(tree_path)
            params['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml']=str(tree_path)
        if guard or recovery:
            params['cmd_vel_mux']['ros__parameters']['nav_topic'] = '/cmd_vel_guarded_nav'
        if cfg.get('synchronize_replan'):
            os.environ['VIRA_FIXTURE_BARRIER']=str(folder.resolve())
            params['planner_server']['ros__parameters']['GridBased']['plugin']='vira_validation::SynchronizedNavfn'
        lib = target/'lib/gracemo_nav2'
        executable = lib/'coordinator_node.py'
        if recovery or cfg.get('doorway_trial'):
            shutil.copy2(executable,lib/'baseline_coordinator.py')
        if cfg.get('doorway_trial'):
            trial = Path(__file__).with_name('exit_alignment_trial.py' if cfg.get('exit_alignment') else 'doorway_trial.py')
            shutil.copy2(trial,lib/'doorway_trial.py')
            if not recovery:
                shutil.copyfile(trial,executable)
        if recovery:
            import xml.etree.ElementTree as ET
            if cfg.get('swept_critic'):
                # A missing private overlay otherwise looks like a fixture timeout.
                from ament_index_python.resources import get_resource
                manifests, prefix=get_resource('dwb_core__pluginlib__plugin','vira_planner_fixture_barrier')
                critic_class='vira_validation::RectangularSweepCritic' if cfg.get('rectangular_guard') else 'vira_validation::SweptFootprintCritic'
                installed=False
                for manifest in manifests.splitlines():
                    library=ET.parse(Path(prefix)/manifest).getroot()
                    if library.find(f"./class[@type='{critic_class}']") is not None:
                        installed=(Path(prefix)/'lib'/('lib'+library.attrib['path']+'.so')).is_file()
                        if installed:
                            break
                if not installed:
                    raise RuntimeError('SweptFootprint critic export or shared library missing')
            planner=params['planner_server']['ros__parameters']
            planner['planner_plugins'].append('RecoveryPlanner')
            planner['RecoveryPlanner']=dict(plugin='nav2_smac_planner::SmacPlannerHybrid',
                    motion_model_for_search='DUBIN',minimum_turning_radius=.4,
                    tolerance=.05,allow_unknown=False,max_planning_time=2.,smooth_path=False)
            if cfg.get('clearance_recovery'):
                planner['RecoveryPlanner']=dict(plugin='vira_validation::ClearanceNavfn',allow_unknown=False,tolerance=.05)
                os.environ['VIRA_CLEARANCE_RECOVERY']='1'
            if cfg.get('approach'):
                from ament_index_python.packages import get_package_share_directory
                primitives=Path(get_package_share_directory('nav2_smac_planner'))/'sample_primitives/5cm_resolution/0.5m_turning_radius/diff/output.json'
                lattice=dict(plugin='nav2_smac_planner::SmacPlannerLattice',lattice_filepath=str(primitives),
                             allow_reverse_expansion=False,allow_unknown=False,smooth_path=False,tolerance=.05)
                planner['RecoveryPlanner']=lattice.copy()
                if not cfg.get('synchronize_replan'):
                    planner['GridBased']=lattice.copy()
                if cfg.get('planner_cell_margin'):
                    grid = params['global_costmap']['global_costmap']['ros__parameters']
                    cell = grid['resolution']
                    # Cover a cell's center-to-corner distance and the live
                    # guard's res/4 interpolation margin during planar search.
                    planning_padding = cell * (2**-.5 + .25)
                    grid['footprint_padding'] = grid.get('footprint_padding', .01) + planning_padding
            plugins=params['controller_server']['ros__parameters']['FollowPath']['critics']
            plugins.insert(plugins.index('BaseObstacle'),'ObstacleFootprint')
            if cfg.get('swept_critic'):
                plugins.insert(0,'SweptFootprint')
                params['controller_server']['ros__parameters']['FollowPath']['publish_evaluation']=True
                params['controller_server']['ros__parameters']['FollowPath']['SweptFootprint.class']=critic_class
            if cfg.get('match_braking'):
                params['controller_server']['ros__parameters']['FollowPath']['SweptFootprint.planning_padding']=planning_padding
            if cfg.get('policy_preference'):
                plugins.append('PolicyPreference')
                params['controller_server']['ros__parameters']['FollowPath']['PolicyPreference.class']='vira_validation::PolicyPreferenceCritic'
            coordinator = 'master_coordinator.py' if cfg.get('master_recovery') else 'recovery_coordinator.py'
            shutil.copyfile(Path(__file__).with_name(coordinator),executable)
            tree=Path(__file__).with_name('dynamic_replan.xml')
            shutil.copy2(tree,lib/tree.name)
            initial=ET.parse(tree)
            initial.find('.//ComputePathToPose').set('planner_id','GridBased')
            if cfg['kind']=='replan':
                # The normal trial sequence computes only once. Exercise a
                # real second planner invocation while FollowPath is active.
                initial = ET.parse(target/'share/gracemo_nav2/behavior_trees/navigate_to_pose_gracemo.xml')
            if cfg.get('master_recovery'):
                initial=ET.parse(target/'share/gracemo_nav2/behavior_trees/navigate_to_pose_gracemo.xml')
            initial.write(target/'share/gracemo_nav2/behavior_trees/navigate_to_pose_gracemo.xml')
        config.write_text(yaml.safe_dump(params, sort_keys=False))
        (folder/'candidate.yaml').write_text(config.read_text())
    processes=[]
    handles=[]
    def start(cmd,log,env=None):
        stream=(folder/log).open('x')
        handles.append(stream)
        proc=subprocess.Popen(cmd,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True,env=env)
        processes.append(proc)
        return proc
    try:
        # Older binaries ignore flags and start the standalone listener. Build
        # the matching workspace source before a private instance is started.
        with (folder/'kernel_build.log').open('x') as log:
            subprocess.run(['cargo','build','--offline','--manifest-path','kernel/Cargo.toml',
                            '-p','gracemo-kernel'],stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
        import secrets
        adapter_token=secrets.token_urlsafe(32)
        voice_token=secrets.token_urlsafe(32)
        grants=state_dir/'adapter-grants.json'
        with open(grants,'x',opener=lambda path,flags: os.open(path,flags,0o600)) as f:
            json.dump([dict(id="StandaloneMissionRunner",token=adapter_token,actions=["StartMission","Cancel"]),
                       dict(id="graceemo_interaction",token=voice_token,actions=["Stop"])],f)
        kernel=start(['kernel/target/debug/gracemo-kernel','--isolated-simulation'],'kernel.log',dict(os.environ,VIRA_ADAPTER_GRANTS=str(grants)))
        deadline=time.monotonic()+8
        port_file=state_dir/'listen.port'
        while not port_file.exists():
            if kernel.poll() is not None or time.monotonic()>deadline:
                raise RuntimeError('Private simulation kernel did not start; rebuild gracemo-kernel')
            time.sleep(.05)
        kernel_url='http://127.0.0.1:'+str(int(port_file.read_text()))
        if guard or recovery:
            guard_script='generation_guard.py' if cfg.get('master_recovery') else 'dynamic_guard.py'
            if cfg.get('rectangular_guard'):
                guard_script='rectangular_guard.py'
            if cfg.get('approach'):
                guard_script='approach_guard.py'
            start([sys.executable,str(Path(__file__).with_name(guard_script)),'--ros-args','-p','use_sim_time:=true',
                   *(['-p','planning_padding:='+str(planning_padding)] if cfg.get('planner_cell_margin') else []),
                   '-p','recovery:='+str(recovery).lower(),'-p',
                   'govern_recovery_speed:='+str(cfg.get('govern_speed',False)).lower(), *(['-p','interrupt_phase:='+cfg['interrupt_at']] if cfg.get('master_recovery') and cfg.get('interrupt_at') else [])],'guard.log')
        if cfg.get('restart_trial'):
            # Separate process groups only in this fixture, so the experiment
            # can restart one owned component without signalling other graphs.
            from ament_index_python.packages import get_package_prefix
            base=start(['ros2','launch','gracemo_gazebo','vira_complete.launch.py','headless:=true','use_rviz:=false',
                        'launch_nav2:=false','kernel_url:='+kernel_url],'launch.log')
            nav_cmd=['ros2','launch','gracemo_nav2','nav2.launch.py','headless:=true','use_rviz:=false']
            supervisor=[str(Path(get_package_prefix('gracemo_nav2'))/'lib/gracemo_nav2/coordinator_node.py'),
                        '--ros-args','-p','use_sim_time:=true','-p','use_room_doorway_waypoints:=true','-p','kernel_url:='+kernel_url]
            start([str(Path(get_package_prefix('gracemo_nav2'))/'lib/gracemo_nav2/terrain_monitor_node.py'),
                   '--ros-args','-p','use_sim_time:=true'],'terrain.log')
            nav=start(nav_cmd,'nav.log')
            coord=start(supervisor,'coordinator.log')
            def idle(label):
                proc=start([sys.executable,'scripts/validation/check_idle_zero.py'],label+'.json')
                if proc.wait(25):
                    raise RuntimeError('Idle command gate failed at '+label)
            def ready(log,token,proc):
                until=time.monotonic()+60
                while token not in (folder/log).read_text():
                    if proc.poll() is not None or time.monotonic()>until:
                        raise RuntimeError('Restart readiness failed in '+log)
                    time.sleep(.1)
            ready('coordinator.log','AUTONOMY_READY',coord)
            idle('idle_start')
            stop(coord,signal.SIGINT)
            coord=start(supervisor,'coordinator_restart.log')
            ready('coordinator_restart.log','AUTONOMY_READY',coord)
            idle('idle_coordinator_restart')
            stop(nav,signal.SIGINT)
            nav=start(nav_cmd,'nav_restart.log')
            ready('nav_restart.log','Managed nodes are active',nav)
            idle('idle_nav_restart')
            observer=start([sys.executable,'scripts/validation/idle_latch_watch.py',str(folder/'latch_idle.json')],'idle_watch.log')
            smoke=start([sys.executable,'scripts/run_gazebo_estop_smoke.py','--out',str(folder/'estop.json')],'estop.log')
            if smoke.wait(110):
                raise RuntimeError('Restarted stack E-stop test failed')
            observer.wait(15)
            idle('idle_post_reset')
            return
        launch=start(['ros2','launch','gracemo_gazebo','vira_complete.launch.py','headless:=true','use_rviz:=false','kernel_url:='+kernel_url],'launch.log')
        if cfg.get('record_rgbd'):
            start([sys.executable,str(Path(__file__).with_name('capture_rgbd.py')),str(folder/'rgbd')],'rgbd.log')
        if cfg.get('mapping_observers'):
            ready_deadline=time.monotonic()+90
            while not any('[lifecycle_manager_navigation]' in line and 'Managed nodes are active' in line
                          for line in (folder/'launch.log').read_text().splitlines()):
                if time.monotonic()>ready_deadline or launch.poll() is not None:
                    raise RuntimeError('Navigation lifecycle unavailable before visual mapping')
                time.sleep(.2)
            start(['ros2','launch',str(Path('ros2_ws/src/gracemo_nav2/launch/gazebo_rgbd_mapping.launch.py').resolve()),
                   'database:='+str((folder/'visual.db').resolve())],'mapping.log')
        if cfg.get('swept_critic'):
            ready_deadline=time.monotonic()+60
            while True:
                log_text=(folder/'launch.log').read_text(errors='replace')
                if (f'Using critic "SweptFootprint" ({critic_class})' in log_text
                        and 'Activating controller_server' in log_text):
                    break
                if launch.poll() is not None or time.monotonic()>ready_deadline:
                    raise RuntimeError('Controller did not configure and activate the private critic')
                time.sleep(.1)
        if cfg.get('staging_goal'):
            deadline=time.monotonic()+60
            while 'Managed nodes are active' not in (folder/'launch.log').read_text():
                if time.monotonic()>deadline:
                    raise RuntimeError('Failed to initialize varied world')
                time.sleep(.2)
            for name,*box in cfg['fixed_geometry']:
                spawn_fixture(box,name=name)
            staging=start([sys.executable,'scripts/run_autonomous_mission.py','--kernel-url',kernel_url,'--goal',*map(str,cfg['staging_goal']),
                          '--out',str(folder/'staging.json')],'staging.log',dict(os.environ,VIRA_ADAPTER_TOKEN=adapter_token))
            if staging.wait(120)!=0:
                raise RuntimeError('Robot failed to reach staging pose')
        actor=None
        if regression in REGRESSIONS:
            deadline=time.monotonic()+60
            while 'Activating collision_monitor' not in (folder/'launch.log').read_text():
                if time.monotonic()>deadline:
                    raise RuntimeError('Simulation failed to initialize')
                time.sleep(.2)
            spawn_fixture(cfg['box'])
            (folder/'fixture.json').write_text(json.dumps(dict(box=cfg['box'],goal=cfg['goal']))+'\n')
        elif not regression:
            actor=start([sys.executable,__file__,str(folder),'--inject'],'actor.log')
        recorder=start([sys.executable,'scripts/validation/capture_routing.py',str(folder/'trace.jsonl.gz'),'--duration',str(mission_timeout+120),'--dynamic',*(['--truth-file',str(folder/'truth.jsonl.gz')] if cfg.get('dense_truth') else [])],'recorder.log')
        if kernel.poll() is not None or launch.poll() is not None: raise RuntimeError('Startup failed')
        if regression=='safety':
            observer=start([sys.executable,'scripts/validation/idle_latch_watch.py',str(folder/'latch_idle.json')],'idle_watch.log')
            estop=start([sys.executable,'scripts/run_gazebo_estop_smoke.py','--out',str(folder/'estop.json')],'estop.log')
            estop.wait(110)
            idle=start([sys.executable,'scripts/validation/check_idle_zero.py'],'idle.json')
            idle.wait(25)
            observer.wait(20)
            shutdown=start([sys.executable,'scripts/validation/shutdown_watch.py',str(folder)],'shutdown.log')
            deadline=time.monotonic()+20
            while not (folder/'shutdown.armed').exists():
                if shutdown.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('Unable to verify idle before shutdown')
                time.sleep(.05)
            stop(launch,signal.SIGINT)
            if shutdown.wait(50):
                raise RuntimeError('Shutdown command gate failed')
            return
        boundary=None
        if cfg.get('authorization_check'):
            boundary=start([sys.executable, 'scripts/validation/authorization_boundary.py',
                            str(folder), kernel_url], 'boundary.log')
            deadline=time.monotonic()+25
            while not (folder/'boundary.ready').exists():
                if boundary.poll() is not None or time.monotonic()>deadline:
                    raise RuntimeError('ROS dispatch observer did not arm')
                time.sleep(.1)
        destinations=[] if regression=='patrol' else ['--goal',*map(str,cfg['goal'])]
        probe = None
        if cfg.get('live_voice'):
            probe=start([sys.executable,'scripts/validation/observe_voice_stop.py',str(folder)],'motion.log')
            start(['vira_interaction/.venv/bin/python','scripts/validation/microphone_stop.py',str(folder),kernel_url],
                  'microphone.log',dict(os.environ,VIRA_ADAPTER_TOKEN=voice_token))
        if cfg.get('interrupt_at'):
            probe = start([sys.executable, 'scripts/validation/interrupt_recovery.py',
                           cfg['interrupt_at'], str(folder/'interrupt.json')], 'interrupt.log')
        mission=start([sys.executable,'scripts/run_autonomous_mission.py','--kernel-url',kernel_url,*destinations,
                       '--readiness-timeout','90','--timeout',str(mission_timeout),'--out',str(folder/'mission.json')],'mission.log',dict(os.environ,VIRA_ADAPTER_TOKEN=adapter_token))
        deadline=time.monotonic()+mission_timeout+105
        while mission.poll() is None:
            if time.monotonic()>deadline:
                raise TimeoutError('Standalone runner did not exit')
            if '[ERROR] [gazebo-1]: process has died' in (folder/'launch.log').read_text(errors='replace'):
                raise RuntimeError('Gazebo exited before the mission finished')
            time.sleep(.5)
        if boundary is not None and boundary.wait(8) != 0:
            raise RuntimeError('ROS dispatch observation failed')
        if not (folder/'mission.json').is_file():
            raise RuntimeError(f'Mission runner exited {mission.returncode} without a result; see {folder}/mission.log')
        if cfg.get('verify_memory'):
            from kernel_context_demo import kernel as restart_kernel
            from gracemo_sdk import AdapterClient
            mission_result = json.loads((folder/'mission.json').read_text())
            trace = mission_result['trace_id']
            before = AdapterClient('memory_inspector',kernel_url).get_trace_history(trace)
        if probe:
            probe.wait(100)
        if cfg.get('synchronize_replan') and not (folder/'fixture.confirmed').exists():
            raise RuntimeError('Second planner invocation did not confirm insertion')
        if actor and not (probe and cfg['interrupt_at']=='NAVIGATING'):
            if actor.poll() is None: actor.wait(15)
            if actor.returncode: raise RuntimeError('Fixture injection failed')
        if cfg.get('synchronize_replan'):
            exited = folder/'replan.exited'
            confirmed = folder/'fixture.confirmed'
            if not exited.exists() or int(exited.read_text()) < int(confirmed.read_text()):
                raise RuntimeError('Insertion timing unverified; rebuild the synchronized fixture plugin')
        if recorder.poll() is not None:
            raise RuntimeError('Telemetry capture failed')
        if regression=='patrol':
            stop(recorder,signal.SIGTERM)
            time.sleep(2)
            idle=start([sys.executable,'scripts/validation/check_idle_zero.py'],'idle.json')
            idle.wait(25)
            estop=start([sys.executable,'scripts/run_gazebo_estop_smoke.py','--out',str(folder/'estop.json')],'estop.log')
            estop.wait(110)
    finally:
        for proc in reversed(processes):
            if cfg.get('verify_memory') and proc is locals().get('kernel') and 'before' in locals():
                # All mission producers have exited. Read the final consistent page.
                try:
                    before = AdapterClient('memory_inspector',kernel_url).get_trace_history(trace)
                finally:
                    stop(proc, signal.SIGINT)
            else:
                stop(proc,signal.SIGTERM if proc is locals().get('recorder') else signal.SIGINT)
        for stream in handles: stream.close()
        if not cfg.get('verify_memory') or sys.exc_info()[0] is not None:
            overlay.cleanup()
    try:
        if cfg.get('verify_memory'):
            with restart_kernel(state_dir) as url:
                client = AdapterClient('memory_inspector',url)
                after = client.get_trace_history(trace)
                unrelated = client.get_trace_history(str(uuid.uuid4()))
            terminal = mission_result['kernel_result']
            def matches(row):
                data = json.loads(row['payload']).get('data', {})
                return (data.get('mission_id') == mission_result['mission_id'] and
                        data.get('state') == terminal['state'])
            passed = bool(before and before == after and not unrelated and
                          all(row['trace_id']==trace for row in after) and any(map(matches,after)))
            (folder/'restart.json').write_text(json.dumps(dict(before=before,after=after,
                mission_id=mission_result['mission_id'],trace_id=trace,terminal=terminal['state'],
                unrelated=unrelated,passed=passed),indent=2))
            if not passed:
                raise RuntimeError('Mission history did not survive kernel restart')
    finally:
        overlay.cleanup()


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('folder',type=Path)
    cli.add_argument('--live-voice',action='store_true')
    cli.add_argument('--authorization-check',action='store_true')
    cli.add_argument('--verify-memory',action='store_true')
    cli.add_argument('--approach',action='store_true')
    cli.add_argument('--record-rgbd',action='store_true')
    cli.add_argument('--match-braking',action='store_true')
    cli.add_argument('--policy-preference',action='store_true')
    cli.add_argument('--mapping-observers',action='store_true')
    cli.add_argument('--planner-cell-margin',action='store_true')
    cli.add_argument('--rectangular-guard',action='store_true')
    cli.add_argument('--goal-budget',action='store_true')
    cli.add_argument('--turning-corridor',action='store_true')
    cli.add_argument('--restart-trial',action='store_true')
    cli.add_argument('--planner-fence',action='store_true')
    cli.add_argument('--clearance-recovery', action='store_true')
    cli.add_argument('--dense-truth', action='store_true')
    cli.add_argument('--master-recovery', action='store_true')
    cli.add_argument('--exit-alignment', action='store_true')
    cli.add_argument('--doorway-trial', action='store_true')
    cli.add_argument('--govern-speed',action='store_true')
    cli.add_argument('--interrupt-at', choices=('NAVIGATING','OBSTACLE_DETECTED','SAFETY_STOP','COSTMAP_UPDATE','ROUTE_REASSESSMENT','REPLANNING','PATH_VALIDATION','RESUMING'))
    cli.add_argument('--swept-critic',action='store_true')
    cli.add_argument('--synchronize-replan',action='store_true')
    cli.add_argument('--temporary', action='store_true')
    cli.add_argument('--level',type=int,choices=(1,2,3),default=1)
    cli.add_argument('--index',type=int,default=0)
    cli.add_argument('--seed',type=int,default=42)
    cli.add_argument('--inject',action='store_true')
    cli.add_argument('--regression',choices=(*REGRESSIONS,'patrol','safety'))
    cli.add_argument('--mission-timeout',type=float,default=115.)
    cli.add_argument('--recovery',action='store_true')
    cli.add_argument('--guard',action='store_true')
    cli.add_argument('--spawn',action='store_true')
    a=cli.parse_args()
    if a.spawn:
        cfg=json.loads((a.folder/'scenario.json').read_text())
        print(json.dumps(spawn_fixture(cfg['box'], movable=bool(cfg['crossing_speed']))))
    elif a.inject: inject(a.folder,json.loads((a.folder/'scenario.json').read_text()))
    else:
        cfg=scenario(a.index,a.seed,a.level)
        if a.verify_memory:
            cfg['verify_memory']=True
        if a.match_braking:
            a.planner_cell_margin=True
            cfg['match_braking']=True
        if a.policy_preference:
            cfg['policy_preference']=True
            a.mapping_observers=a.planner_cell_margin=True
        if a.record_rgbd:
            cfg['record_rgbd']=True
        if a.mapping_observers:
            cfg['mapping_observers']=True
        if a.planner_cell_margin:
            cfg['planner_cell_margin']=True
            a.approach=True
        if a.approach:
            cfg['approach']=True
            a.rectangular_guard=a.dense_truth=True
        if a.rectangular_guard:
            a.clearance_recovery=True
            cfg['rectangular_guard']=True
        if a.goal_budget:
            a.turning_corridor=True
            cfg['goal_budget']=True
        if a.turning_corridor:
            cfg['turning_corridor']=True
            a.doorway_trial=a.dense_truth=True
        if a.restart_trial:
            cfg['restart_trial']=True
        if a.planner_fence:
            cfg['planner_fence']=True
            a.clearance_recovery=True
        if a.clearance_recovery:
            a.master_recovery=True
            cfg['clearance_recovery']=True
        if a.dense_truth:
            cfg['dense_truth']=True
        if a.master_recovery:
            cfg['master_recovery'] = True
            a.doorway_trial = a.govern_speed = True
        if a.exit_alignment:
            a.doorway_trial = True
            cfg['exit_alignment'] = True
        if a.doorway_trial:
            cfg['doorway_trial'] = True
        if a.temporary:
            cfg.update(temporary=True, kind='temporary', box=[2.,0.,.35,1.65], crossing_speed=0.)
        if a.interrupt_at:
            cfg['interrupt_at'] = a.interrupt_at
            a.govern_speed = True
        if a.govern_speed:
            cfg['govern_speed']=True
            a.swept_critic=True
        if a.swept_critic:
            cfg['swept_critic']=True
            a.recovery=True
        if a.synchronize_replan:
            if cfg['kind']!='replan':
                cli.error('--synchronize-replan requires a replan scenario')
            cfg.update(synchronize_replan=True,min_plan_count=1)
        if a.live_voice:
            cfg["live_voice"]=True
        if a.authorization_check:
            cfg['authorization_check']=True
        if a.regression:
            cfg.update(kind=a.regression,box=None,goal=None,crossing_speed=0.)
            if a.regression in REGRESSIONS:
                cfg['box'],cfg['goal']=REGRESSIONS[a.regression]
        # DDS domain selection is bounded. Concurrent PID modulo collisions
        # can interleave independent /clock streams despite Gazebo partitions.
        lock_dir = Path(tempfile.gettempdir()) / f'vira-validation-{os.getuid()}'
        lock_dir.mkdir(mode=0o700, exist_ok=True)
        with (lock_dir/'dynamic.lock').open('a') as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            run(a.folder,cfg,a.guard,a.recovery,a.mission_timeout,a.regression)
