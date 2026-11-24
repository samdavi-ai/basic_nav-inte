#!/usr/bin/env python3
"""Compare complete padded footprints to Gazebo floor-plan collision geometry."""
import argparse
import base64
import collections
import gzip
import json
import re
import xml.etree.ElementTree as ET
import zlib
from pathlib import Path
import hashlib

import numpy as np

# Measured Nav2 published_footprint edge lengths are checked before analysis.
HALF = np.array([.36, .35])
RADIUS = np.linalg.norm(HALF)
SIGNS = np.array([[1, 1], [1, -1], [-1, -1], [-1, 1]])
WORLD = Path('ros2_ws/src/gracemo_gazebo/worlds/apartment_floor.world')


def obstacles(box):
    out = [('fixture', *box)] if box is not None else []
    for model in ET.parse(WORLD).findall('.//world/model'):
        if model.get('name') in ('gracemo_vira', 'floor'):
            continue
        for link in model.findall('link'):
            for collision in link.findall('collision'):
                xy = np.zeros(2)
                yaw = 0.
                for el in (model, link, collision):
                    x,y,_,roll,pitch,a = map(float, el.findtext('pose', '0 0 0 0 0 0').split())
                    if abs(roll)+abs(pitch)>1e-5:
                        raise ValueError('Nonplanar geometry')
                    xy += [np.cos(yaw)*x-np.sin(yaw)*y,np.sin(yaw)*x+np.cos(yaw)*y]
                    yaw += a
                geom = collision.find('geometry')
                if geom.find('box') is not None:
                    w,h,_ = map(float, geom.findtext('box/size').split())
                else:  # Desk legs are contained in the projected tabletop.
                    w = h = 2*float(geom.findtext('cylinder/radius'))
                if abs(np.sin(2*yaw))>1e-5:
                    raise ValueError('Static obstacle is not axis-aligned')
                if abs(np.sin(yaw))>.5:
                    w,h = h,w
                out.append((model.get('name'), *xy, w,h))
    # Cached, outward-rounded world XY bounding boxes of the apartment's Fuel
    # collision meshes and primitives. These also enclose non-axis-aligned props.
    includes = [
        ('master_bed', -5.843, 5.029, 2.062, 2.49),
        ('refrigerator', -1.5, -6.2, 0.736, 0.724),
        ('dining_table', -4.5, -4.0, 1.5, 0.8),
        ('chair_1', -4.459, -3.0, 0.514, 0.58),
        ('chair_2', -4.541, -5.0, 0.514, 0.58),
        ('living_sofa', 4.521, -5.2, 1.044, 1.802),
        ('bowl', 4.5, -3.5, 0.196, 0.196),
        ('bookshelf', 8.5, 5.307, 0.92, 0.406),
        ('cardboard_box', 1.5, 6.0, 0.5, 0.4),
        ('coke_can', 4.0, 5.5, 1.558, 1.558),
    ]
    entries = ET.parse(WORLD).findall(".//world/include")
    placements = " ".join(" ".join(el.findtext(k, "") for k in ("name","uri","pose")) for el in entries)
    if hashlib.sha256(placements.encode()).hexdigest() != "40b68f964963e6e1468165dcd5c27654cf24e6d246d8fe16392f9139da0cc165":
        raise ValueError("Recompute the included furniture bounds for the modified world")
    return out + includes


def densify(points, spacing):
    pts = np.array(points, dtype=float)
    pts[:,2] = np.unwrap(pts[:,2])
    delta = np.diff(pts,axis=0)
    span = np.linalg.norm(delta[:,:2],axis=1)+RADIUS*abs(delta[:,2])
    n = np.maximum(1,np.ceil(span/spacing).astype(int))
    starts = np.repeat(np.cumsum(n)-n,n)
    fraction = ((np.arange(sum(n))-starts)/np.repeat(n,n))[:,None]
    samples = np.repeat(pts[:-1],n,axis=0)+fraction*np.repeat(delta,n,axis=0)
    return np.vstack([samples,pts[-1]]), float(np.max(span/n,initial=0)/2)


def separation(pts, obstacle):
    _,x,y,width,depth = obstacle
    half = np.array([width,depth])/2
    d = pts[:,:2]-[x,y]
    c,s = np.cos(pts[:,2]),np.sin(pts[:,2])
    rotation = np.stack([c,-s,s,c],axis=1).reshape(-1,2,2)
    local = np.einsum('nj,nji->ni',d,rotation)
    gaps = np.column_stack([abs(d)-half-abs(rotation)@HALF,
            abs(local)-HALF-abs(rotation.transpose(0,2,1))@half])
    signed = np.max(gaps,axis=1)
    ext = signed>0
    # Closest features of two disjoint rectangles include a vertex and an edge.
    rot = rotation[ext]
    robot_corners = d[ext,None,:] + (SIGNS*HALF)@rot.transpose(0,2,1)
    obs_corners = (SIGNS*half-d[ext,None,:])@rot
    distances = [np.linalg.norm(np.maximum(abs(v)-h,0),axis=2).min(axis=1)
                 for v,h in ((robot_corners,half),(obs_corners,HALF))]
    signed[ext] = np.minimum(*distances)
    return signed


def sweep(points, scenery, spacing=.01):
    pts, bound = densify(points, spacing)
    best = np.inf
    hit = None
    for obs in scenery:
        _,x,y,w,h = obs
        # Discard remote geometry only when its bounding-circle lower bound exceeds best.
        near = np.linalg.norm(np.maximum(abs(pts[:,:2]-[x,y])-[w/2,h/2],0),axis=1)-RADIUS <= max(best,0)
        if not near.any():
            continue
        sub = pts[near]
        dist = separation(sub,obs)
        i = dist.argmin()
        if dist[i] < best:
            best,hit = dist[i],dict(obstacle=obs[0],pose=sub[i].round(6).tolist())
    if abs(best)<bound and spacing>.0001:
        return sweep(points, scenery, spacing/10)
    return dict(sample_min_m=round(best,6),swept_lower_bound_m=round(best-bound,6),**hit)


def analyze(folder, box, plan_kind="plan"):
    fixture = json.loads((folder/'fixture.json').read_text()) if (folder/'fixture.json').exists() else dict(box=box,goal=[4.5,0])
    scenario_path = folder / 'scenario.json'
    if scenario_path.exists() and json.loads(scenario_path.read_text()).get('kind') == 'patrol':
        fixture = dict(box=None, goal=None)
    scenery = obstacles(fixture['box'])
    with gzip.open(folder/'trace.jsonl.gz','rt') as f:
        rows = [json.loads(line) for line in f]
    footprint = np.array(next(r['corners'] for r in rows if r['kind']=='local_footprint'))
    edges = np.linalg.norm(footprint-np.roll(footprint,1,axis=0),axis=1)
    np.testing.assert_allclose(np.sort(edges),np.sort(np.tile(HALF*2,2)),atol=1e-4)
    truth = np.array([r['truth'] for r in rows if r['kind']=='motion' and r['truth']])
    native_truth = folder/'truth.jsonl.gz'
    if native_truth.is_file():
        with gzip.open(native_truth, 'rt') as stream:
            truth = np.array([json.loads(line) for line in stream])
    if np.any(np.diff(truth[:,0]) < 0):
        raise ValueError('Gazebo truth clock moved backwards')
    truth = truth[np.r_[True,np.diff(truth[:,0])>0]]
    observed = sweep(truth[:,1:],scenery)
    # Bound between physical samples independently of the interpolated yaw/path.
    gaps = np.diff(truth[:,0])
    node_clear = np.minimum.reduce([separation(truth[:,1:], obs) for obs in scenery])
    observed['raw_pose_min_m'] = round(float(np.min(node_clear)),6)
    observed['speed_bound_min_m'] = round(float(np.min(np.minimum(node_clear[:-1],node_clear[1:])
            - gaps*(.65+RADIUS*1.5)/2)),6)
    tf_rows = [r for r in rows if r['kind']=='motion' and r['map_odom']]
    tf_times = np.array([r['t'] for r in tf_rows])
    paths = []
    for r in rows:
        if r['kind']!=plan_kind or not r['points']:
            continue
        p = np.array(r['points'])
        if len(p)<2:
            continue
        given = sweep(p,scenery)
        dx,dy = np.diff(p[:,:2],axis=0).T
        angles = np.arctan2(dy,dx)
        translating = np.hypot(dx,dy) > 1e-8
        # In-place primitives have a defined yaw and no positional tangent.
        p[:-1,2] = np.where(translating, angles, p[:-1,2])
        if translating[-1]:
            p[-1,2] = angles[-1]
        # Invert the online map<-odom estimate to inspect the path in the
        # controller's frame as well as in nominal apartment-map coordinates.
        x,y,h = tf_rows[max(0,np.searchsorted(tf_times,r['t'],side='right')-1)]['map_odom']
        world_path = p.copy()
        world_path[:,:2] = (p[:,:2]-[x,y]) @ np.array([[np.cos(h),-np.sin(h)],
                                                        [np.sin(h),np.cos(h)]])
        world_path[:,2] -= h
        paths.append(dict(t=r['t'], points=len(p),published_yaw=given,tangent=sweep(p,scenery),
                     tangent_in_odom=sweep(world_path,scenery)))
    rollouts = [sweep(r["points"], scenery) for r in rows if r["kind"]=="local_plan"
                and r["frame"]=="odom" and r["points"]]
    costs = {}
    for name in (('global', 'local') if fixture['box'] is not None else ()):
        counts = []
        for r in rows:
            if r['kind']!=name+'_costmap':
                continue
            grid = np.frombuffer(zlib.decompress(base64.b64decode(r['cells_zlib_b64'])),np.uint8).reshape(r['size'][::-1])
            iy,ix = np.where(grid==254)
            x,y,w,h = fixture['box']
            # Local odom tracks Gazebo origin. Global map is nominal apartment coordinates.
            ins = [abs(r['origin'][i]+(idx+.5)*r['resolution']-ctr)<=half+r['resolution']
                   for i,idx,ctr,half in ((0,ix,x,w/2),(1,iy,y,h/2))]
            counts.append(int(np.count_nonzero(ins[0]&ins[1])))
        costs[name] = dict(snapshots=len(counts),first=counts[0],max=max(counts),median=float(np.median(counts)))
    log = (folder/'launch.log').read_text()
    reasons = collections.Counter(re.findall(r'\[DWBLocalPlanner\]: [\d.]+: (.+)',log))
    nav = [r['velocities']['cmd_vel_nav'] for r in rows if r['kind']=='motion' and 'cmd_vel_nav' in r['velocities']]
    mission = json.loads((folder/'mission.json').read_text())
    evaluations = [r for r in rows if r['kind']=='dwb_evaluation']
    last_map = mission.get("final_map_pose") or {}
    map_truth_error = (np.linalg.norm(truth[-1,1:3]-[last_map["x"], last_map["y"]])
                       if "x" in last_map and "y" in last_map else None)
    return dict(box=fixture['box'],map_truth_error_m=map_truth_error,plan_stream=plan_kind,footprint_edges_m=edges.round(5).tolist(),executed=observed,
            truth_final=truth[-1,1:].tolist(),truth_goal_error_m=(round(np.linalg.norm(truth[-1,1:3]-fixture['goal']),5) if fixture['goal'] is not None else None),
            max_truth_sample_gap_s=float(max(np.diff(truth[:,0]))),
            global_paths=len(paths),first_path=paths[0] if paths else None,
            worst_tangent_path=min(paths,key=lambda p:p['tangent']['sample_min_m']) if paths else None,
            worst_tangent_in_odom=min((p['tangent_in_odom']['swept_lower_bound_m'] for p in paths),
                                      default=None),
            plans=paths,lethal_cells_near_box=costs,local_plans=sum(r['kind']=='local_plan' for r in rows),
            worst_local_plan=min(rollouts,key=lambda q:q["swept_lower_bound_m"]) if rollouts else None,
            max_dwb_forward_mps=max((v[0] for v in nav),default=0),
            dwb_all_rejected_cycles=log.count('[DWBLocalPlanner]: No valid trajectories'),
            observed_dwb_rejections=sum(r['rejected'] for r in evaluations) if evaluations else None,
            evaluation_messages=len(evaluations),
            critic_reasons=reasons)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path)
    parser.add_argument('--smoothed', action='store_true', help='Evaluate the path returned by the smoother')
    parser.add_argument('--expect', choices=['arrive','blocked'])
    parser.add_argument('--check-global', action='store_true',
                        help='Require a positive tangent-swept margin on every global plan')
    args=parser.parse_args()
    result = analyze(args.folder,[2,-.5,.35,.35],"smoothed_plan" if args.smoothed else "plan")
    gates = {}
    if args.expect:
        mission = json.loads((args.folder/'mission.json').read_text())
        history = mission.get('mission_history') or {}
        gates['positive_executed_clearance'] = result['executed']['speed_bound_min_m'] > 0
        if args.expect == 'arrive':
            gates.update(
                matched_completion=(mission['mission_completed'] and history.get('success') is True and
                                    history.get('mission_id') == mission['mission_id'] and
                                    history.get('waypoints_completed') == history.get('total_waypoints')),
                map_goal=mission['goal_error_m'] <= .30,
                gazebo_goal=result['truth_goal_error_m'] <= .35,
                gazebo_yaw=abs(result['truth_final'][2]) <= .25,
                map_gazebo_agreement=result['map_truth_error_m'] <= .15,
                clear_chosen_rollouts=(result['worst_local_plan'] is not None and
                                       result['worst_local_plan']['swept_lower_bound_m'] > 0))
            if args.check_global:
                gates['clear_global_paths'] = (result['worst_tangent_path'] is not None and
                    result['worst_tangent_path']['tangent']['swept_lower_bound_m'] > 0 and
                    result['worst_tangent_in_odom'] > 0 and all(
                        p['published_yaw']['swept_lower_bound_m'] > 0 for p in result['plans']))
        else:
            gates.update(terminal_failure=mission['failure_state']=='NAVIGATION_FAILURE',
                         bounded=mission['wall_time_sec']<=115,
                         no_false_completion=not mission['mission_completed'] and
                             history.get('success') is False and history.get('waypoints_completed')==0,
                         stayed_on_start_side=result['truth_final'][0]<result['box'][0]-result['box'][2]/2)
        result['acceptance'] = {name: bool(value) for name, value in gates.items()}
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if all(gates.values()) else 1)
