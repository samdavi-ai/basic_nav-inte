"""Print acceptance evidence from private Gazebo runs, including incomplete runs."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]/'docs/validation/dynamic_obstacles'


def read(folder, name):
    path = folder/name
    return json.loads(path.read_text()) if path.exists() and path.stat().st_size else {}


def summarize(folder):
    mission = read(folder, 'mission.json')
    metrics = read(folder, 'metrics.json')
    geometry = read(folder, 'geometry.json')
    terminal = mission.get('kernel_result') or {}
    coordinator = mission.get('coordinator_result') or {}
    interruption = read(folder, 'interrupt.json')
    row = dict(run=folder.name, seed=read(folder,'scenario.json').get('seed'),
               mission_id=mission.get('mission_id'), trace_id=mission.get('trace_id'),
               coordinator=coordinator.get('status'), kernel=terminal.get('state'),
               ids_match=bool(mission.get('trace_id') and mission.get('mission_id') and
                              terminal.get('trace_id')==mission['trace_id'] and
                              coordinator.get('mission_id')==terminal.get('mission_id')==mission['mission_id']),
               interrupt=interruption or None)
    for key in ('insert_requested','first_scene_confirmation','sensor_hits','first_lethal_marks','first_risk_action','guard_stop','first_final_zero_after_stop_action',
                'physical_stop_s','whole_mission_interpolated_m','minimum_scene_interpolated_m','old_and_new_plans','route_resumptions',
                'retries','goal_error_m','observed_dwb_rejections_after_request'):
        row[key] = metrics.get(key)
    row['static_geometry'] = {k:geometry[k] for k in ('executed','truth_goal_error_m','worst_tangent_in_odom','worst_local_plan','observed_dwb_rejections') if k in geometry}
    row['first_stop_to_resume_s'] = None
    if row['guard_stop'] is not None and row['route_resumptions']:
        row['first_stop_to_resume_s'] = round(row['route_resumptions'][0]['t']-row['guard_stop'],3)
    row['runner_timed_out'] = mission.get('timed_out')
    row['waypoints_completed'] = coordinator.get('waypoints_completed')
    row['waypoints_total'] = coordinator.get('total_waypoints')
    row['terminal_reason'] = coordinator.get('reason')
    row['duration_s'] = coordinator.get('duration_sec')
    row['idle'] = read(folder, 'idle.json') or None
    row['estop_smoke'] = read(folder, 'estop.json') or None
    row['assessment'] = classify(folder.name, row)
    return row


def classify(name, row):
    excluded = {
        'governed_review_replan': 'fixture insertion unconfirmed',
        'governed_review_near_goal': 'fixture insertion unconfirmed',
        'governed_review_larger': 'invalid simulation clock',
        'governed_review_temporary': 'invalid simulation clock',
    }
    if name in excluded:
        return 'EXCLUDED: ' + excluded[name]
    if row['interrupt']:
        return 'E-stop probe ' + row['interrupt']['status']
    if not row['ids_match']:
        return 'terminal correlation unverified'
    clearance = row['whole_mission_interpolated_m']
    if clearance is None:
        clearance = row['static_geometry'].get('executed', {}).get('swept_lower_bound_m')
    if clearance is None:
        return 'executed clearance unverified'
    if clearance <= 0:
        return 'failed interpolated footprint-clearance gate'
    if name == 'governed_mirror_review':
        return 'completed with positive clearance; fixture teardown failed'
    if row['runner_timed_out']:
        return 'mission deadline expired'
    if row['kernel'] == 'NAVIGATION_FAILURE':
        return 'bounded terminal failure with positive interpolated clearance'
    if row['kernel'] == 'MISSION_COMPLETED':
        return 'completed with positive interpolated clearance'
    return 'unclassified terminal state'


if __name__=='__main__':
    folders = sorted({p for pattern in ('governed*review*','governed_serial_*','interrupt_review_*','baseline_patrol_review')
                      for p in ROOT.glob(pattern) if p.is_dir()})
    print(json.dumps([summarize(p) for p in folders], indent=2))
