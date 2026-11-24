#!/usr/bin/env python3
"""Correlate ROS observations, decisions and commands with confirmed Gazebo fixture poses."""
import argparse
import base64
import gzip
import json
import math
import re
import zlib
from pathlib import Path

import numpy as np

from analyze_routing import HALF, densify, obstacles, separation, sweep


def metrics(folder):
    cfg=json.loads((folder/'scenario.json').read_text())
    manifest=[json.loads(s) for s in (folder/'world.jsonl').read_text().splitlines()]
    if len(manifest)<2: raise ValueError('Missing confirmed Gazebo scene pose')
    request=manifest[0]['request']
    samples=np.array([[r['t'],r['x'],r['y']] for r in manifest[1:]])
    confirmation=samples[0,0]
    removed = (json.loads((folder/'removal.json').read_text())['confirmed_absent']
               if cfg.get('temporary') else float('inf'))
    box=cfg['box']
    with gzip.open(folder/'trace.jsonl.gz','rt') as stream:
        rows=[json.loads(line) for line in stream]
    native = np.unique(np.array([r['pose'] for r in rows if r['kind']=='fixture_truth']),axis=0)
    if len(native):
        samples=native[:,:3]
    else:
        samples=samples[np.argsort(samples[:,0],kind='stable')]
    motion_verified = not cfg['crossing_speed'] or (len(native)>1 and np.ptp(native[:,2]) >= 2*abs(box[1])-.03
                           and np.max(abs(native[:,3])) < 1e-5 and np.all((native[:,4]-.5 < .15) & (native[:,4]+.5 > .15)))
    footprint = np.array(next(r['corners'] for r in rows if r['kind']=='local_footprint'))
    edges = np.linalg.norm(footprint-np.roll(footprint,1,axis=0),axis=1)
    np.testing.assert_allclose(np.sort(edges),np.sort(np.tile(HALF*2,2)),atol=1e-4)
    motions=[r for r in rows if r['kind']=='motion' and r['truth']]
    truth=np.unique(np.array([r['truth'] for r in motions]),axis=0)
    times=truth[:,0]
    yaw=np.unwrap(truth[:,3])

    def center(t):
        return np.array([np.interp(t,samples[:,0],samples[:,i]) for i in (1,2)]).T

    def robot(t):
        return np.array([np.interp(t,times,truth[:,1]),np.interp(t,times,truth[:,2]),np.interp(t,times,yaw)])

    def transform(xy,pose):
        c,s=np.cos(pose[2]),np.sin(pose[2])
        return xy@np.array([[c,s],[-s,c]])+pose[:2]

    def distance(t,poses):
        rel=np.array(poses,copy=True); rel[:,:2]-=center(t)
        return separation(rel,('fixture',0,0,*box[2:]))

    def match(t,xy,res):
        if t > removed:
            return False
        # Match points near a face, excluding the box interior and nearby wall surfaces.
        d=abs(xy-center(t))-np.array(box[2:])/2
        return np.count_nonzero((np.max(d,axis=1)<=res)&(np.max(d,axis=1)>=-res))>=3

    sense={}
    marks={}
    first_pre={}
    for r in rows:
        kind=r['kind']
        if kind=='sensor_hits':
            t=r['stamp']; label=r['source']
            if t>=request and label in sense: continue
            data=zlib.decompress(base64.b64decode(r['xy_zlib_b64']))
            xy=np.frombuffer(data,'<i2').reshape(-1,2)/50
            hit=match(t,transform(xy,robot(t)),.035)
            if hit and t>=request: sense[label]=t
            elif hit and request-1<t<request: first_pre[label]=t
        elif kind in ('global_costmap','local_costmap'):
            t=r.get('published') or r['stamp'] or r['t']
            if t>=request and kind in marks: continue
            grid=np.frombuffer(zlib.decompress(base64.b64decode(r['cells_zlib_b64'])),np.uint8).reshape(r['size'][::-1])
            y,x=np.where(grid==254)
            xy=(np.column_stack([x,y])+.5)*r['resolution']+r['origin'][:2]
            if r['frame']=='map':
                tf=next((m['map_odom'] for m in reversed(motions) if m['t']<=r['t'] and m['map_odom']),None)
                if tf is None: continue
                xy=transform(xy-tf[:2],[0,0,-tf[2]])
            if match(t,xy,r['resolution']):
                if t>=request: marks[kind]=t
                elif request-1<t<request: first_pre[kind]=t

    pts=truth[times>=request]
    knot_times=np.unique(np.r_[request,pts[:,0],samples[(samples[:,0]>=request)&(samples[:,0]<=times[-1]),0]])
    physical=np.asarray([robot(t) for t in knot_times])
    relative=physical.copy()
    relative[:,:2]-=center(knot_times)
    if np.isfinite(removed):
        at_removal = robot(removed)
        at_removal[:2] -= center(removed)
        relative = np.vstack((relative[knot_times < removed], at_removal))
    fixture_bound=sweep(relative,[('fixture',0.,0.,*box[2:])])['swept_lower_bound_m']
    static_scenery=obstacles(box)[1:]+cfg.get('fixed_geometry',[])
    static_bound=sweep(physical,static_scenery)['swept_lower_bound_m']
    apartment_bound = sweep(truth[:, 1:], static_scenery)['swept_lower_bound_m']
    plan_results=[r for r in rows if r['kind']=='planning_result']
    mins=distance(pts[:,0],pts[:,1:])
    mon=[r for r in rows if r['kind']=='collision_monitor' and r['t']>=request and r['action'] in (1,2,3,4)]
    guards=[r for r in rows if r['kind']=='guard' and r['state']=='SAFETY_STOP' and r['t']>=request]
    cutoff=next((r['t'] for r in mon if r['action']==1),None)
    monitor_cutoff=cutoff
    if guards: cutoff=min([guards[0]['t']]+([cutoff] if cutoff is not None else []))
    def zero(r):
        return r['kind']=='command' and r['topic']=='cmd_vel' and max(map(abs,r['twist']))<1e-4
    stop=next((r['t'] for r in rows if cutoff is not None and r['t']>=cutoff and zero(r)),None)
    sensor=min(sense.values(),default=None); mark=marks.get('local_costmap')
    elapsed=lambda a,b: None if a is None or b is None else round(b-a,4)
    collision_time=mon[0]['t'] if mon else None
    first_monitor_action=collision_time
    if guards:
        collision_time=min([guards[0]['t']]+([collision_time] if collision_time is not None else []))
    mfile=folder/'mission.json'
    m=json.loads(mfile.read_text()) if mfile.exists() else {}
    if not m:
        log=(folder/'mission.log').read_text()
        for key,label in [('mission_id','Mission ID'),('trace_id','Trace ID')]:
            found=re.search(rf'^{label}: ([a-z0-9-]+)$',log,re.MULTILINE)
            if found: m[key]=found.group(1)
        m['timed_out']='before timeout' in log
    history=m.get('coordinator_result') or {}
    terminal=(m.get('kernel_result') or {}).get('state')
    expected={'FAILED':'NAVIGATION_FAILURE','COMPLETED':'MISSION_COMPLETED','CANCELLED':'MISSION_CANCELLED'}.get(history.get('status'))
    kernel=m.get('kernel_result') or {}
    aligned=(expected is not None and m.get('trace_id') and terminal==expected and m.get('mission_id')
             and history.get('mission_id')==m['mission_id'] and kernel.get('mission_id')==m['mission_id']
             and kernel.get('trace_id')==m['trace_id'])
    statuses=[r['state'] for r in rows if r['kind']=='status']
    retries=sum(state=='OBSTACLE_HANDLING' and prev!=state for prev,state in zip([None]+statuses,statuses))
    replans=[]
    rollouts=[]
    for r in rows:
        if r['kind']=='local_plan' and r['points'] and request<=r['stamp']<=removed and r['frame']=='odom':
            dense,bound=densify(np.asarray(r['points']),.01)
            rollouts.append(round(float(distance(r['stamp'],dense).min()-bound),5))
        if r['kind']!='plan' or not r['points']:continue
        tf=next((m['map_odom'] for m in reversed(motions) if m['t']<=r['t'] and m['map_odom']),None)
        if tf is None:continue
        p=np.asarray(r['points']);p[:,:2]=transform(p[:,:2]-tf[:2],[0,0,-tf[2]])
        p[:,2]-=tf[2]
        if len(p)>1:
            dv=np.diff(p[:,:2],axis=0); tangent=np.arctan2(dv[:,1],dv[:,0])
            p[:-1,2]=np.where(np.linalg.norm(dv,axis=1)>1e-7,tangent,p[:-1,2])
        dense,bound=densify(p,.01)
        fixture_margin=round(float(distance(r['stamp'],dense).min()-bound),5) if r['stamp']<=removed else None
        scenery_margin=sweep(p,static_scenery)['swept_lower_bound_m']
        replans.append(dict(t=r['stamp'],points=len(p),frozen_fixture_clearance_m=fixture_margin,
                            frozen_scene_clearance_m=min(scenery_margin,fixture_margin) if fixture_margin is not None else scenery_margin))
    # A single zero from a controller does not establish an actual chassis stop.
    settled=None
    if stop is not None:
        begin=None
        for row in motions:
            if row['stamp']<stop: continue
            v,w=row['twist']
            if abs(v)<.02 and abs(w)<.05:
                if begin is None: begin=row['stamp']
                if row['stamp']-begin>=.20:
                    settled=begin
                    break
            else:
                begin=None
    if sensor is not None and settled is not None and settled>=sensor:
        interval=[r for r in motions if sensor<=r['stamp']<=settled]
        disp=max([0.]+[float(np.linalg.norm(robot(r['stamp'])[:2]-robot(sensor)[:2])) for r in interval])
    else:
        interval=[]; disp=None
    resumes=[r for r in rows if r['kind']=='guard' and r['state']=='RESUMING' and r['t']>=request]
    verified_resumes=[]
    for event in resumes:
        plan=next((p for p in replans if event.get('global_plan_stamp')==p['t']),None)
        local=next((r for r in rows if r['kind']=='local_plan' and r['stamp']==event['local_rollout_stamp']),None)
        if not local:continue
        points=np.asarray(local['points'])
        dense,pad=densify(points,.01)
        fixture_margin=float(distance(local['stamp'],dense).min()-pad) if local['stamp']<=removed else float('inf')
        static_margin=sweep(points,static_scenery)['swept_lower_bound_m']
        verified_resumes.append(dict(t=event['t'],global_clearance_m=plan['frozen_scene_clearance_m'] if plan else None,
            local_clearance_m=min(fixture_margin,static_margin),local_stamp=local['stamp']))
    commands=[r for r in rows if r['kind']=='command' and r['topic']=='cmd_vel']
    final_time=commands[-1]['t']
    last_moving=max((r['t'] for r in commands if not zero(r)),default=final_time)
    goal_error=math.dist(pts[-1,1:3],cfg['goal'])
    evaluations = [row for row in rows if row['kind'] == 'dwb_evaluation' and row['t'] >= request]
    return dict(case=cfg,mission_id=m.get('mission_id'), trace_id=m.get('trace_id'),matched_terminal=bool(aligned),
        dwb_evaluation_messages_after_request=len(evaluations),
        observed_dwb_rejections_after_request=sum(row['rejected'] for row in evaluations) if evaluations else None,
        route_resumptions=verified_resumes, runner_timed_out=m.get('timed_out',False), goal_within_035m=goal_error<=.35,
        final_zero_command_window_s=round(final_time-last_moving,4),
        planner_actions=plan_results, insertion_within_proven_planner_interval=any(
            r['accepted']<=request and confirmation<=r['path_stamp'] for r in plan_results if r['status']==4),
        state=history.get('status'),kernel_terminal=terminal,retries=retries,
        insert_requested=request, first_scene_confirmation=confirmation, last_scene_confirmation=samples[-1].tolist(),
        fixture_z_range_m=[float(native[:,4].min()),float(native[:,4].max())] if len(native) else None,
        confirmed_fixture_trajectory=bool(motion_verified), physical_stop_s=settled,
        risk_to_zero_s=elapsed(collision_time,stop), risk_to_chassis_stop_s=elapsed(collision_time,settled),
        selected_rollouts_after_insertion=len(rollouts),worst_frozen_rollout_fixture_m=min(rollouts,default=None),
        minimum_scene_interpolated_m=min(fixture_bound,static_bound) if motion_verified else None,
        whole_mission_interpolated_m=min(fixture_bound, apartment_bound) if motion_verified else None,
        minimum_fixture_interpolated_m=fixture_bound if motion_verified else None,
        sensor_transform_drops=sum(r['kind']=='missing_sensor_tf' for r in rows),
        costmap_time_source='header or metadata when set, otherwise recorder receipt',
        sensor_hits=sense,first_lethal_marks=marks,preexisting_matches=first_pre,
        first_monitor_action=first_monitor_action, first_risk_action=collision_time, monitor_stop=monitor_cutoff,
        guard_stop=guards[0]['t'] if guards else None,first_final_zero_after_stop_action=stop,
        sensor_to_local_mark_s=elapsed(sensor,mark),local_mark_to_stop_s=elapsed(mark,stop),sensor_to_stop_s=elapsed(sensor,stop),
        peak_linear_speed_until_stop_mps=max((abs(r['twist'][0]) for r in interval),default=None),
        max_displacement_until_stop_m=disp,frame_time_basis='Gazebo seconds; monitor/command receipt times',
        sampled_fixture_clearance_m=round(float(mins[pts[:,0]<=removed].min()),5) if motion_verified and np.any(pts[:,0]<=removed) else None,goal_error_m=round(goal_error,5),
        old_and_new_plans=replans,after_insertion_plan_count=sum(p['t']>=request for p in replans),
        confirmed_motion_m=round(float(np.linalg.norm(samples[-1,1:]-samples[0,1:])),4))


if __name__=='__main__':
    cli=argparse.ArgumentParser(description=__doc__)
    cli.add_argument('folder',type=Path)
    args=cli.parse_args()
    print(json.dumps(metrics(args.folder),indent=2))
