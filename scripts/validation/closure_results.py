"""Summarize the private Gazebo patrols, preserving unavailable measurements."""
import argparse
import json
from pathlib import Path
import re

from master_results import assess, read


def summarize(folder):
    result = assess(folder)
    result['fixture_cleanup'] = 'no inserted obstacle'
    result['scenario_id'] = folder.name
    result['obstacle_trajectory'] = []
    result['collision_creation_s'] = None
    result['first_observable_s'] = None
    result['recovery_attempts'] = len([event for event in result['transitions']
                                       if event['state']=='REPLANNING'])
    log = (folder/'launch.log').read_text()
    result['controller_deadline_warnings'] = log.count('Control loop missed its desired rate')
    result['waypoint_dispatches'] = [line for line in log.splitlines()
                                     if re.search(r'Planning path to waypoint',line,re.I)]
    result['geometry'] = read(folder/'geometry.json').get('executed')
    result['build'] = ('Release' if folder.name=='patrol_optimized' else 'default CMake flags')
    result['active_profile'] = {
        'patrol': 'lattice and directed turn',
        'patrol_settle': 'lattice, directed turn and zero-command drift settling',
        'patrol_ns': 'drift settling and integer nanosecond rollout age',
        'patrol_margin': 'integer age and planner cell margin',
        'patrol_optimized': 'planner cell margin with optimized critic build',
    }.get(folder.name)
    return result


if __name__ == '__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root',type=Path)
    root=parser.parse_args().root
    print(json.dumps([summarize(f.parent) for f in sorted(root.glob('*/scenario.json'))],indent=2))
