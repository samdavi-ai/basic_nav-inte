"""Score only independently verifiable visual matches in fresh Gazebo launches."""
import json
import sqlite3
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from evaluate_registration import matrix, stats
from scipy.spatial.transform import Rotation


def load(folder):
    grouped=defaultdict(list)
    with (folder/'registration.jsonl').open() as f:
        for line in f:
            item=json.loads(line)
            grouped[item['kind']].append(item)
    return grouped


def nearest(truth,t):
    closest=min(truth,key=lambda row:abs(row['t']-t))
    return matrix(closest) if abs(closest['t']-t)<=.05 else None


def summarize(mapping,trial,source_ids):
    # One rigid alignment from the mapping session only. Never align to trial truth.
    good={r['t'] for r in mapping['info'] if not r['lost']}
    first=next(r for r in mapping['odom'] if r['t'] in good and nearest(mapping['truth'],r['t']) is not None)
    origin=nearest(mapping['truth'],first['t'])@np.linalg.inv(matrix(first))
    localized={r['t']:r for r in trial['localization']}
    qualified=[]
    for r in trial['graph']:
        saved_ids = [i for i in (r['loop'], r['proximity']) if i > 0 and i in source_ids]
        if not saved_ids or r['stats'].get('Loop/Visual_inliers/',0)<20:
            continue
        actual=nearest(trial['truth'],r['t'])
        if r['t'] not in localized or actual is None:
            continue
        diff=np.linalg.inv(actual)@origin@matrix(localized[r['t']])
        qualified.append(dict(t=r['t'],ref=r['ref'],loop=r['loop'],proximity=r['proximity'],
                              saved_node_ids=sorted(set(saved_ids)),
                              inliers=r['stats']['Loop/Visual_inliers/'],
                              translation_m=float(np.linalg.norm(diff[:3,3])),
                              rotation_deg=float(np.degrees(Rotation.from_matrix(diff[:3,:3]).magnitude()))))
    return dict(graph_messages=len(trial['graph']),pose_messages=len(trial['localization']),
                first_truth_xyz=trial['truth'][0]['xyz'] if trial['truth'] else None,
                all_matches_within_target=bool(qualified) and all(r['translation_m']<=.20 and
                             r['rotation_deg']<=5 for r in qualified),
                measured_matches=qualified,translation_m=stats([r['translation_m'] for r in qualified]),
                rotation_deg=stats([r['rotation_deg'] for r in qualified]))


if __name__=='__main__':
    source = Path(sys.argv[1])
    folder = Path(sys.argv[2])
    with sqlite3.connect((source/'map.db').resolve().as_uri()+'?mode=ro', uri=True) as db:
        ids = {row[0] for row in db.execute('SELECT id FROM Node')}
    trial = load(folder)
    result = summarize(load(source), trial, ids)
    if (folder/'occlusion.json').exists():
        events = json.loads((folder/'occlusion.json').read_text())
        start, end = events['inserted_rgb_stamp'], events['removed_rgb_stamp']
        # Leave three camera periods around service responses out of the interval counts.
        covered = [r for r in trial['info'] if start + .3 < r['t'] < end - .3]
        after = [r for r in trial['info'] if r['t'] > end + .3]
        result['occlusion'] = dict(**events, covered_updates=len(covered),
                covered_lost=sum(r['lost'] for r in covered), after_updates=len(after),
                after_nonlost=sum(not r['lost'] for r in after),
                first_loss_after_insert_s=next((r['t']-start for r in trial['info']
                        if start < r['t'] < end and r['lost']), None),
                first_nonlost_after_removal_s=next((r['t']-end for r in trial['info']
                        if r['t']>end and not r['lost']), None),
                first_verified_match_after_removal_s=next((r['t']-end for r in result['measured_matches']
                        if r['t']>end), None))
    print(json.dumps(result,indent=2))
