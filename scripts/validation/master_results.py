"""Assemble saved records from the master repair campaign. No ROS publishers."""
import argparse
import gzip
import json
from pathlib import Path


def read(p):
    if p.is_file() and p.stat().st_size:
        return json.loads(p.read_text())
    return {}


def assess(p):
    cfg=read(p/'scenario.json')
    mission=read(p/'mission.json')
    kernel=mission.get('kernel_result') or {}
    coord=mission.get('coordinator_result') or {}
    correlated=bool(mission.get('trace_id') and kernel.get('trace_id')==mission['trace_id'] and
                    kernel.get('mission_id')==coord.get('mission_id')==mission.get('mission_id'))
    m=read(p/'metrics.json')
    g=read(p/'geometry.json')
    e=read(p/'interrupt.json')
    run=read(p.parent/(p.name+'_run.json'))
    geometry=g.get('executed',{})
    clearance=m.get('whole_mission_interpolated_m',geometry.get('swept_lower_bound_m'))
    speed_bound=geometry.get('speed_bound_min_m')
    timeline=[]
    trace=p/'trace.jsonl.gz'
    if trace.exists():
        with gzip.open(trace,'rt') as stream:
            for line in stream:
                row=json.loads(line)
                if row['kind']=='guard':
                    timeline.append(row)
    timing=m.get('insertion_within_proven_planner_interval') if cfg.get('synchronize_replan') else None
    if (p/'replan.exited').exists():
        confirmation=read_stamp(p/'fixture.confirmed')
        timing=bool(confirmation and confirmation<=read_stamp(p/'replan.exited') and timing)
    if cfg.get('kind')=='safety':
        checks=read(p/'latch_idle.json')
        verdict='PASS' if (read(p/'estop.json').get('result')=='PASS' and
                           read(p/'idle.json').get('passed') and read(p/'shutdown.json').get('passed') and
                           all(checks.get(k,{}).get('passed') for k in ('latched','after_reset'))) else 'FAIL'
    elif cfg.get('restart_trial'):
        verdict='PARTIAL' if all(read(p/(k+'.json')).get('passed') for k in
                                ('idle_coordinator_restart','idle_nav_restart')) else 'FAIL'
    elif clearance is not None and clearance<=0:
        verdict='UNSAFE FAILURE'
    elif e:
        verdict=e['status'] if correlated and not mission.get('timed_out') else 'PARTIAL'
    elif timing is False or run.get('exit',0)!=0 or not correlated:
        verdict='FAIL'
    elif kernel.get('state')=='NAVIGATION_FAILURE':
        verdict='SAFE TERMINAL FAILURE' if cfg.get('kind')=='blocked' and speed_bound and speed_bound>0 else 'FAIL'
    elif kernel.get('state')=='MISSION_COMPLETED' and clearance is not None and clearance>0:
        goal_error=m.get('goal_error_m',g.get('truth_goal_error_m'))
        all_waypoints=coord.get('success') and coord.get('waypoints_completed')==coord.get('total_waypoints')
        if not all_waypoints or (goal_error is not None and goal_error>.35):
            verdict='FAIL'
        else:
            verdict='PASS' if goal_error is not None and speed_bound is not None and speed_bound>0 else 'PARTIAL'
    else:
        verdict='NOT TESTED'
    log=(p/'launch.log').read_text() if (p/'launch.log').exists() else ''
    return dict(run=p.name,scenario=cfg,verdict=verdict,run_exit=run.get('exit'),mission_id=mission.get('mission_id'),
        trace_id=mission.get('trace_id'),correlated=correlated,coordinator=coord,kernel=kernel,timed_out=mission.get('timed_out'),
        swept_min_m=clearance,speed_bound_min_m=speed_bound,raw_min_m=m.get('sampled_fixture_clearance_m',geometry.get('raw_pose_min_m')),
        sensor_hits=m.get('sensor_hits'),map_marks=m.get('first_lethal_marks'),stop=m.get('guard_stop'),
        zero=m.get('first_final_zero_after_stop_action'),rest=m.get('physical_stop_s'),resumptions=m.get('route_resumptions'),
        retries=m.get('retries'),goal_error=m.get('goal_error_m',g.get('truth_goal_error_m')),
        planner_interval_verified=timing,insert_requested=m.get('insert_requested'),insert_confirmed=m.get('first_scene_confirmation'),
        plans=m.get('old_and_new_plans',g.get('plans')),worst_local_m=m.get('worst_frozen_rollout_fixture_m',g.get('worst_local_plan')),
        dwb_evaluations=m.get('dwb_evaluation_messages_after_request',g.get('evaluation_messages')),
        dwb_rejected=m.get('observed_dwb_rejections_after_request',g.get('observed_dwb_rejections')),
        transitions=timeline,interrupt=e or None,idle=read(p/'idle.json') or None,estop=read(p/'estop.json') or None,
        latch_idle=read(p/'latch_idle.json') or None,shutdown=read(p/'shutdown.json') or None,
        after_coordinator_restart=read(p/'idle_coordinator_restart.json') or None,
        after_nav_restart=read(p/'idle_nav_restart.json') or None,
        fixture_actor_aborted='terminate called' in ((p/'actor.log').read_text() if (p/'actor.log').exists() else ''),
        gazebo_shutdown_escalated='failed to terminate' in log)


def read_stamp(p):
    return int(p.read_text()) if p.exists() and p.stat().st_size else None


def render(root):
    records=[assess(f.parent) for f in sorted(root.glob('*/scenario.json')) ]
    (root/'results.json').write_text(json.dumps(records,indent=2)+'\n')
    rows=['# Deterministic Gazebo results','',
          'Clearance is the minimum for the executed interpolated footprint. A positive run with no independent positive intersample bound has partial geometry evidence. Interruption verdicts apply to their probes. Shutdown escalation is recorded independently in results.json.','',
          '| Run | Verdict | Kernel | Waypoints | Minimum (m) | Goal error (m) | DWB rejected |','|---|---|---|---:|---:|---:|---:|']
    stops=['# E-stop interruption matrix','','Times use the observer monotonic clock for latency and the ROS simulation clock for entry. Displacement limit 0.15 m. Final-second component limits 0.05 m/s and 0.05 rad/s.','',
           '| State | Probe | Entry (sim s) | Observe zero (ms) | Max displacement (m) | Final-second peak | Post-reset command peak | Terminal |','|---|---|---:|---:|---:|---:|---:|---|']
    def fmt(v): return '—' if v is None else f'{v:.4f}'
    for r in records:
        k=r['kernel'].get('state','—')
        if (e:=r['interrupt']):
            stops.append(f"| [{e['target']}]({r['run']}/interrupt.json) | {r['verdict']} | {fmt(e.get('trigger',{}).get('sim_time'))} | {fmt(1000*e['zero_latency_s']) if e.get('zero_latency_s') is not None else '—'} | {fmt(e.get('max_displacement_m'))} | {fmt(e.get('final_second_speed'))} | {fmt(e.get('post_reset_peak_command'))} | {k} |")
        w=r['coordinator']
        rows.append(f"| [{r['run']}]({r['run']}/scenario.json) | {r['verdict']} | {k} | {w.get('waypoints_completed','—')}/{w.get('total_waypoints','—')} | {fmt(r['swept_min_m'])} | {fmt(r['goal_error'])} | {r['dwb_rejected'] if r['dwb_rejected'] is not None else '—'} |")
    stops.extend(['','The stopped internal phases use a Gazebo-only pause at state entry. REPLANNING denotes the recovery FSM phase. This probe does not isolate an instruction inside the planner plugin. NAVIGATING can occur as the first drive command is issued, before measurable wheel speed. The separate moving E-stop smoke test exercises an already moving chassis. Each probe rejected a new request while latched and required explicit reset.',''])
    (root/'ESTOP_INTERRUPTION_MATRIX.md').write_text('\n'.join(stops))
    (root/'DETERMINISTIC_RESULTS.md').write_text('\n'.join(rows)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    render(parser.parse_args().root)
